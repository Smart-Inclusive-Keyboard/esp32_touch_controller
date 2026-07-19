/*
 * main.c -- ESP32 Touch Controller
 *
 * Reads capacitive touch from an AXS5106L controller, interprets the
 * gestures, and sends 6-byte HID gamepad reports over a transmit-only
 * UART to an esp32s3_smart_keyboard receiver.
 *
 * Hardware (Waveshare ESP32-C6-Touch-LCD-1.47, verified):
 *   LCD  JD9853  SPI2   SCLK=1 MOSI=2 MISO=3  CS=14 DC=15 RST=22 BL=23
 *                172x320 portrait, column gap=34, colour-inversion on
 *   Touch AXS5106L  I2C0  SDA=18 SCL=19 INT=21 RST=20  addr=0x63
 *   UART  TX=GPIO13  115200 8-N-1  (gamepad HID output)
 *
 * Gesture -> HID mapping
 *   Slide up        negative axis impulse (-32767 for 100 ms then 0)
 *   Slide down      positive axis impulse (+32767 for 100 ms then 0)
 *   Short tap upper half  Button 1 press + release
 *   Short tap lower half  Button 0 press + release
 *   Long tap (>=TC_LONG_TAP_MS) toggle VERTICAL / HORIZONTAL mode
 *                          (fires as soon as the threshold elapses,
 *                          without waiting for the finger to lift)
 *
 * A slide spanning the full length of the screen emits several axis
 * events (CONFIG_TC_SLIDE_FULL_EVENTS, default 3); shorter slides emit
 * proportionally fewer events.
 *
 * In VERTICAL mode the impulse is sent on the Y axis; in HORIZONTAL
 * mode it is sent on the X axis.
 *
 * The screen shows a schematic navigation guide.  The current mode is
 * indicated by a bright edge line: along the right edge in VERTICAL
 * mode and along the top edge in HORIZONTAL mode.  While sliding the
 * active edge line is highlighted, and a tapped button label is
 * highlighted, using CONFIG_TC_COLOR_HIGHLIGHT.  Backlight brightness
 * and the interface colours are configurable (see Kconfig).
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_jd9853.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_axs5106.h"

#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "uart_gamepad.h"
#include "sdkconfig.h"

static const char *TAG = "tc";

/* ---------------------------------------------------------------------
 * Board-specific pin definitions
 * --------------------------------------------------------------------- */
#ifdef CONFIG_TC_BOARD_WAVESHARE_C6_TOUCH_LCD_147

/* LCD SPI (SPI2) */
#define LCD_SCLK         GPIO_NUM_1
#define LCD_MOSI         GPIO_NUM_2
#define LCD_MISO         GPIO_NUM_3
#define LCD_CS           GPIO_NUM_14
#define LCD_DC           GPIO_NUM_15
#define LCD_RST          GPIO_NUM_22
#define LCD_BL           GPIO_NUM_23
#define LCD_PIXEL_CLK_HZ (80 * 1000 * 1000)
#define LCD_H_RES        172
#define LCD_V_RES        320
#define LCD_COL_GAP      34   /* JD9853 x-offset at rotation 0 */

/* Touch I2C (I2C0) */
#define TP_SDA           GPIO_NUM_18
#define TP_SCL           GPIO_NUM_19
#define TP_INT           GPIO_NUM_21
#define TP_RST           GPIO_NUM_20

/* Backlight PWM */
#define BL_LEDC_TIMER    LEDC_TIMER_0
#define BL_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define BL_LEDC_CH       LEDC_CHANNEL_0
#define BL_LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define BL_LEDC_FREQ_HZ  5000

#endif /* CONFIG_TC_BOARD_WAVESHARE_C6_TOUCH_LCD_147 */

/* ---------------------------------------------------------------------
 * Application constants
 * --------------------------------------------------------------------- */
#define DRAW_BUF_LINES  50    /* LVGL DMA draw buffer height in lines */
#define TOUCH_POLL_MS   20    /* touch polling interval               */
#define IMPULSE_MS      100   /* axis impulse duration                */
#define BUTTON_MS       50    /* button press duration                */
#define HIGHLIGHT_MS    150   /* minimum on-screen highlight duration */

#define SLIDE_MIN_PX    CONFIG_TC_SLIDE_MIN_PX
#define TAP_MAX_MOVE_PX CONFIG_TC_TAP_MAX_MOVE_PX
#define LONG_TAP_MS     CONFIG_TC_LONG_TAP_MS
#define SLIDE_FULL_EVENTS CONFIG_TC_SLIDE_FULL_EVENTS

/* Interface colours (24-bit 0xRRGGBB, from Kconfig). */
#define COLOR_BG        lv_color_hex(CONFIG_TC_COLOR_BACKGROUND)
#define COLOR_FG        lv_color_hex(CONFIG_TC_COLOR_FOREGROUND)
#define COLOR_HL        lv_color_hex(CONFIG_TC_COLOR_HIGHLIGHT)

/* vertical/horizontal indicator line along the edge */
#define EDGE_INDICATOR_THICKNESS 4

typedef enum {
    MODE_VERTICAL,
    MODE_HORIZONTAL,
} tc_mode_t;

/* ---------------------------------------------------------------------
 * Module-level state
 * --------------------------------------------------------------------- */
static esp_lcd_panel_io_handle_t s_io_handle;
static esp_lcd_panel_handle_t    s_panel;
static esp_lcd_touch_handle_t    s_touch;
static lv_disp_t                *s_disp;
static lv_obj_t                 *s_line_right;  /* shown in VERTICAL mode   */
static lv_obj_t                 *s_line_top;    /* shown in HORIZONTAL mode */
static lv_obj_t                 *s_lbl_btn1;    /* upper-half button label  */
static lv_obj_t                 *s_lbl_btn0;    /* lower-half button label  */
static volatile tc_mode_t        s_mode = MODE_VERTICAL;

/* ---------------------------------------------------------------------
 * Hardware initialisation
 * --------------------------------------------------------------------- */
static void hw_lcd_init(void)
{
    /* SPI bus */
    const spi_bus_config_t buscfg = {
        .sclk_io_num     = LCD_SCLK,
        .mosi_io_num     = LCD_MOSI,
        .miso_io_num     = LCD_MISO,
        .quadhd_io_num   = -1,
        .quadwp_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * DRAW_BUF_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* Panel IO */
    esp_lcd_panel_io_spi_config_t io_cfg =
        JD9853_PANEL_IO_SPI_CONFIG(LCD_CS, LCD_DC, NULL, NULL);
    io_cfg.pclk_hz = LCD_PIXEL_CLK_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &s_io_handle));

    /* Panel */
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9853(s_io_handle, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, LCD_COL_GAP, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* Backlight -- brightness from CONFIG_TC_SCREEN_BRIGHTNESS (percent) */
    const ledc_timer_config_t ledc_timer = {
        .speed_mode      = BL_LEDC_MODE,
        .timer_num       = BL_LEDC_TIMER,
        .duty_resolution = BL_LEDC_DUTY_RES,
        .freq_hz         = BL_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    const uint32_t bl_max_duty = (1u << BL_LEDC_DUTY_RES) - 1u;
    const uint32_t bl_duty =
        (bl_max_duty * (uint32_t)CONFIG_TC_SCREEN_BRIGHTNESS) / 100u;

    const ledc_channel_config_t ledc_ch = {
        .speed_mode = BL_LEDC_MODE,
        .channel    = BL_LEDC_CH,
        .timer_sel  = BL_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LCD_BL,
        .duty       = bl_duty,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));
}

static void hw_touch_init(void)
{
    /* I2C master bus */
    i2c_master_bus_handle_t i2c_bus;
    const i2c_master_bus_config_t i2c_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = I2C_NUM_0,
        .scl_io_num                   = TP_SCL,
        .sda_io_num                   = TP_SDA,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    /* Add AXS5106 device */
    i2c_master_dev_handle_t dev;
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ESP_LCD_TOUCH_IO_I2C_AXS5106_ADDRESS,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev));

    /* Touch configuration for portrait rotation 0:
     *   mirror_x=1 maps raw x so that x=0 is at the left edge on screen. */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max        = LCD_H_RES,
        .y_max        = LCD_V_RES,
        .rst_gpio_num = TP_RST,
        .int_gpio_num = TP_INT,
        .levels = {
            .reset     = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy  = 0,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };

    /* Retry -- controller may be slow to wake after power-on. */
    for (int attempt = 1; attempt <= 6; attempt++) {
        esp_err_t err = esp_lcd_touch_new_i2c_axs5106(dev, &tp_cfg, &s_touch);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "AXS5106 ready (attempt %d)", attempt);
            return;
        }
        ESP_LOGW(TAG, "AXS5106 attempt %d: %s", attempt, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    s_touch = NULL;
    ESP_LOGW(TAG, "Touch unavailable -- gesture input disabled");
}

/* ---------------------------------------------------------------------
 * LVGL setup
 * --------------------------------------------------------------------- */
static void lvgl_setup(void)
{
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority     = 4,
        .task_stack        = 10 * 1024,
        .task_affinity     = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms   = 5,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = s_io_handle,
        .panel_handle  = s_panel,
        .buffer_size   = LCD_H_RES * DRAW_BUF_LINES,
        .double_buffer = true,
        .hres          = LCD_H_RES,
        .vres          = LCD_V_RES,
        .monochrome    = false,
        .rotation = {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma  = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        },
    };
    s_disp = lvgl_port_add_disp(&disp_cfg);
}

/* ---------------------------------------------------------------------
 * Navigation guide UI (schematic style: black background, thin lines)
 * --------------------------------------------------------------------- */

/*
 * Screen layout (172 x 320 portrait):
 *
 *  +-----------------------------+  y=0     top edge line = HORIZONTAL mode
 *  |            ^                |       up-arrow  (slide = negative axis)
 *  |                             |
 *  |         BTN 1               |       upper half -- tap triggers Button 1
 *  |                             ||      right edge line = VERTICAL mode
 *  +-----------------------------+  y=160  centre divider
 *  |                             ||
 *  |         BTN 0               |       lower half -- tap triggers Button 0
 *  |                             |
 *  |            v                |       down-arrow (slide = positive axis)
 *  +-----------------------------+  y=320
 *
 * The current mode is shown by a bright edge line: right edge in
 * VERTICAL mode, top edge in HORIZONTAL mode.
 */
static void ui_update_mode(tc_mode_t mode);

static void ui_create(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Configurable background */
    lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Shared style for thin foreground lines */
    static lv_style_t st_line;
    lv_style_init(&st_line);
    lv_style_set_line_color(&st_line, COLOR_FG);
    lv_style_set_line_width(&st_line, 1);
    lv_style_set_line_opa(&st_line, LV_OPA_COVER);

    /* Thicker style for the bright mode-indicator edge line */
    static lv_style_t st_edge;
    lv_style_init(&st_edge);
    lv_style_set_line_color(&st_edge, COLOR_FG);
    lv_style_set_line_width(&st_edge, EDGE_INDICATOR_THICKNESS);
    lv_style_set_line_opa(&st_edge, LV_OPA_COVER);

    /* -- Up arrow -- top of screen -- */
    lv_obj_t *lbl_up = lv_label_create(scr);
    lv_label_set_text(lbl_up, LV_SYMBOL_UP);
    lv_obj_set_style_text_color(lbl_up, COLOR_FG, 0);
    lv_obj_align(lbl_up, LV_ALIGN_TOP_MID, 0, 10);

    /* "BTN 1" label in upper half */
    s_lbl_btn1 = lv_label_create(scr);
    lv_label_set_text(s_lbl_btn1, "BTN 1");
    lv_obj_set_style_text_color(s_lbl_btn1, COLOR_FG, 0);
    lv_obj_align(s_lbl_btn1, LV_ALIGN_TOP_MID, 0, 70);

    /* -- Centre divider line -- */
    static lv_point_t div_pts[2] = {{0, 0}, {LCD_H_RES - 1, 0}};
    lv_obj_t *divider = lv_line_create(scr);
    lv_obj_add_style(divider, &st_line, 0);
    lv_line_set_points(divider, div_pts, 2);
    /* Place so the line sits exactly at y=160 (screen centre) */
    lv_obj_set_pos(divider, 0, LCD_V_RES / 2);

    /* "BTN 0" label in lower half */
    s_lbl_btn0 = lv_label_create(scr);
    lv_label_set_text(s_lbl_btn0, "BTN 0");
    lv_obj_set_style_text_color(s_lbl_btn0, COLOR_FG, 0);
    lv_obj_align(s_lbl_btn0, LV_ALIGN_CENTER, 0, 60);

    /* -- Down arrow -- bottom area -- */
    lv_obj_t *lbl_dn = lv_label_create(scr);
    lv_label_set_text(lbl_dn, LV_SYMBOL_DOWN);
    lv_obj_set_style_text_color(lbl_dn, COLOR_FG, 0);
    lv_obj_align(lbl_dn, LV_ALIGN_BOTTOM_MID, 0, -10);

    /* -- Mode indicator: line along the right edge (VERTICAL mode) -- */
    static lv_point_t right_pts[2] = {{0, 0}, {0, LCD_V_RES - EDGE_INDICATOR_THICKNESS}};
    s_line_right = lv_line_create(scr);
    lv_obj_add_style(s_line_right, &st_edge, 0);
    lv_line_set_points(s_line_right, right_pts, 2);
    lv_obj_set_pos(s_line_right, LCD_H_RES - EDGE_INDICATOR_THICKNESS, 0);

    /* -- Mode indicator: line along the top edge (HORIZONTAL mode) -- */
    static lv_point_t top_pts[2] = {{0, 0}, {LCD_H_RES - EDGE_INDICATOR_THICKNESS, 0}};
    s_line_top = lv_line_create(scr);
    lv_obj_add_style(s_line_top, &st_edge, 0);
    lv_line_set_points(s_line_top, top_pts, 2);
    lv_obj_set_pos(s_line_top, 0, 0);

    /* Initial mode is VERTICAL: show right edge, hide top edge. */
    ui_update_mode(s_mode);
}

static void ui_update_mode(tc_mode_t mode)
{
    /* Must be called with the LVGL port lock held. */
    if (mode == MODE_VERTICAL) {
        lv_obj_clear_flag(s_line_right, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_line_top, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_line_right, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_line_top, LV_OBJ_FLAG_HIDDEN);
    }
}

/* Set the colour of an edge line (foreground when not highlighted). */
static void ui_line_highlight(lv_obj_t *line, bool on)
{
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        lv_obj_set_style_line_color(line, on ? COLOR_HL : COLOR_FG, 0);
        lvgl_port_unlock();
    }
}

/* Set the colour of a button label (foreground when not highlighted). */
static void ui_label_highlight(lv_obj_t *label, bool on)
{
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        lv_obj_set_style_text_color(label, on ? COLOR_HL : COLOR_FG, 0);
        lvgl_port_unlock();
    }
}

/* ---------------------------------------------------------------------
 * Gamepad event dispatch helpers
 * --------------------------------------------------------------------- */

/* Send an impulse on the active axis: max value for IMPULSE_MS, then 0. */
static void send_axis_impulse(int16_t value)
{
    if (s_mode == MODE_VERTICAL) {
        uart_gamepad_report(0, value, 0);
        vTaskDelay(pdMS_TO_TICKS(IMPULSE_MS));
        uart_gamepad_report(0, 0, 0);
    } else {
        uart_gamepad_report(value, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(IMPULSE_MS));
        uart_gamepad_report(0, 0, 0);
    }
}

/*
 * Emit a run of axis impulses on the active axis.  A slide that covers
 * the full length of the screen produces SLIDE_FULL_EVENTS impulses;
 * shorter slides produce proportionally fewer (always at least one).
 * The consecutive impulses are separated by an idle gap so the receiver
 * registers them as distinct navigation steps.
 */
static void send_axis_impulses(int16_t value, int span_px, int full_px)
{
    int count = 1;
    if (full_px > 0) {
        count = (span_px * SLIDE_FULL_EVENTS) / full_px;
    }
    if (count < 1) {
        count = 1;
    }
    if (count > SLIDE_FULL_EVENTS) {
        count = SLIDE_FULL_EVENTS;
    }

    for (int i = 0; i < count; i++) {
        send_axis_impulse(value);
        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(IMPULSE_MS));
        }
    }
}

/* Press then release a single button (0-indexed). */
static void send_button(int btn_index)
{
    const uint16_t mask = (uint16_t)(1u << btn_index);
    uart_gamepad_report(0, 0, mask);
    vTaskDelay(pdMS_TO_TICKS(BUTTON_MS));
    uart_gamepad_report(0, 0, 0);
}

/* Toggle operating mode and update the UI label. */
static void toggle_mode(void)
{
    tc_mode_t new_mode =
        (s_mode == MODE_VERTICAL) ? MODE_HORIZONTAL : MODE_VERTICAL;
    s_mode = new_mode;

    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        ui_update_mode(new_mode);
        lvgl_port_unlock();
    }
    ESP_LOGI(TAG, "Mode: %s",
             (new_mode == MODE_VERTICAL) ? "VERTICAL" : "HORIZONTAL");
}

/* ---------------------------------------------------------------------
 * Touch gesture detection task
 *
 * State machine:
 *   IDLE  -> TOUCHING  on first touch sample
 *   TOUCHING -> IDLE   on touch release; gesture is classified then,
 *                      unless a long tap already fired during the hold.
 *
 * Gesture rules:
 *   displacement < TAP_MAX_MOVE_PX:
 *       held >= LONG_TAP_MS  -> mode toggle (fires immediately while the
 *                               finger is still down, then the touch is
 *                               consumed and ignored on release)
 *       released < LONG_TAP_MS -> short tap (label highlighted)
 *           start_y < LCD_V_RES/2  -> BTN1
 *           start_y >= LCD_V_RES/2  -> BTN0
 *   |dy| >= SLIDE_MIN_PX (and |dy| > |dx|):
 *       dy < 0 (up)   -> negative impulse(s)
 *       dy > 0 (down) -> positive impulse(s)
 *       The number of impulses scales with the slide length, up to
 *       SLIDE_FULL_EVENTS for a full-screen slide.  The active edge
 *       line is highlighted for the duration of the slide.
 * --------------------------------------------------------------------- */
static void touch_task(void *arg)
{
    bool     was_touching = false;
    bool     long_fired   = false;
    uint16_t start_x = 0, start_y = 0;
    uint16_t last_x  = 0, last_y  = 0;
    int64_t  start_ms = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));

        if (!s_touch) {
            continue;
        }

        esp_lcd_touch_read_data(s_touch);

        esp_lcd_touch_point_data_t point = {0};
        uint8_t  num_pts = 0;
        esp_lcd_touch_get_data(s_touch, &point, &num_pts, 1);
        bool touched = (num_pts > 0);
        uint16_t x = point.x, y = point.y;

        if (touched) {
            last_x = x;
            last_y = y;
        }

        if (touched && !was_touching) {
            /* -- Touch start -- */
            start_x  = x;
            start_y  = y;
            start_ms = esp_timer_get_time() / 1000LL;
            was_touching = true;
            long_fired   = false;
            ESP_LOGD(TAG, "touch start (%u,%u)", x, y);

        } else if (touched && was_touching && !long_fired) {
            /* -- Touch held -- fire long tap as soon as the threshold
             * elapses, without waiting for the finger to lift. -- */
            int dx   = (int)last_x - (int)start_x;
            int dy   = (int)last_y - (int)start_y;
            int adx  = dx < 0 ? -dx : dx;
            int ady  = dy < 0 ? -dy : dy;
            int dist = adx > ady ? adx : ady;
            int64_t held_ms = (esp_timer_get_time() / 1000LL) - start_ms;

            if (dist < TAP_MAX_MOVE_PX && held_ms >= (int64_t)LONG_TAP_MS) {
                toggle_mode();
                long_fired = true;
            }

        } else if (!touched && was_touching) {
            /* -- Touch end -- classify gesture -- */
            was_touching = false;

            if (long_fired) {
                /* Mode already switched during the hold; nothing to do. */
                continue;
            }

            int64_t dur_ms = (esp_timer_get_time() / 1000LL) - start_ms;

            int dx   = (int)last_x - (int)start_x;
            int dy   = (int)last_y - (int)start_y;
            int adx  = dx < 0 ? -dx : dx;
            int ady  = dy < 0 ? -dy : dy;
            int dist = adx > ady ? adx : ady;

            ESP_LOGD(TAG, "touch end (%u,%u) dur=%lldms dx=%d dy=%d",
                     last_x, last_y, dur_ms, dx, dy);

            if (dist < TAP_MAX_MOVE_PX) {
                /* Short tap: upper half -> BTN1, lower half -> BTN0.
                 * (Long taps are handled during the hold above.)     */
                lv_obj_t *lbl = (start_y < LCD_V_RES / 2)
                                ? s_lbl_btn1 : s_lbl_btn0;
                ui_label_highlight(lbl, true);
                send_button((start_y < LCD_V_RES / 2) ? 1 : 0);
                vTaskDelay(pdMS_TO_TICKS(HIGHLIGHT_MS));
                ui_label_highlight(lbl, false);
            } else if (ady >= SLIDE_MIN_PX && ady >= adx) {
                /* Vertical slide -- reversed navigation direction.
                 * Number of axis events scales with the slide length.
                 * Highlight the active mode's edge line meanwhile.   */
                lv_obj_t *edge = (s_mode == MODE_VERTICAL)
                                 ? s_line_right : s_line_top;
                ui_line_highlight(edge, true);
                if (dy < 0) {
                    /* Slide up -> negative axis */
                    send_axis_impulses(-32767, ady, LCD_V_RES);
                } else {
                    /* Slide down -> positive axis */
                    send_axis_impulses(32767, ady, LCD_V_RES);
                }
                ui_line_highlight(edge, false);
            }
            /* Mostly horizontal slides are ignored. */
        }
    }
}

/* ---------------------------------------------------------------------
 * app_main
 * --------------------------------------------------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Touch Controller starting");

    hw_lcd_init();
    hw_touch_init();
    lvgl_setup();

    /* Build UI inside the LVGL lock (-1 = wait indefinitely until LVGL task
     * gives up the mutex, which it does during its sleep phase).           */
    if (lvgl_port_lock(-1)) {
        ui_create();
        lvgl_port_unlock();
    }

    ESP_ERROR_CHECK(uart_gamepad_init());

    xTaskCreate(touch_task, "touch", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Running -- initial mode: VERTICAL");
}
