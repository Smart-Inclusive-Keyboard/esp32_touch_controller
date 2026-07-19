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
 * Button/mode inputs (configurable, pull-up, active low):
 *   GPIO for buttons 0-6  generate game controller buttons 0-6
 *   Mode GPIO             open = impulse mode, low = continuous mode
 *
 * Gesture -> HID mapping
 *   Slide up        negative axis impulse (impulse mode) or proportional
 *                   axis (continuous mode)
 *   Slide down      positive axis impulse (impulse mode) or proportional
 *                   axis (continuous mode)
 *   Long tap (>=TC_LONG_TAP_MS) toggle VERTICAL / HORIZONTAL mode
 *                          (fires as soon as the threshold elapses,
 *                          without waiting for the finger to lift)
 *
 * In impulse mode a slide spanning the full length of the screen emits
 * several axis impulse events (CONFIG_TC_SLIDE_FULL_EVENTS, default 3);
 * shorter slides emit proportionally fewer events.
 *
 * In continuous mode button 9 is held permanently on and the active axis
 * is set proportional to the sliding finger's distance from the middle
 * of the touch screen; the long tap switches the active axis just as in
 * impulse mode.
 *
 * In VERTICAL mode the axis output is sent on the Y axis; in HORIZONTAL
 * mode it is sent on the X axis.
 *
 * The screen shows a schematic navigation guide.  The current mode is
 * indicated by a bright edge line: along the right edge in VERTICAL
 * mode and along the top edge in HORIZONTAL mode.  In impulse mode the
 * active edge line is highlighted while sliding (CONFIG_TC_COLOR_HIGHLIGHT
 * over CONFIG_TC_COLOR_FOREGROUND).  In continuous mode the direction
 * indicator uses a dedicated colour pair: idle (CONFIG_TC_COLOR_CONTINUOUS_IDLE)
 * and active (CONFIG_TC_COLOR_CONTINUOUS_ACTIVE).  Backlight brightness
 * and the interface colours are configurable (see Kconfig).
 */

#include <string.h>
#include <stdio.h>
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

#define SLIDE_MIN_PX    CONFIG_TC_SLIDE_MIN_PX
#define TAP_MAX_MOVE_PX CONFIG_TC_TAP_MAX_MOVE_PX
#define LONG_TAP_MS     CONFIG_TC_LONG_TAP_MS
#define SLIDE_FULL_EVENTS CONFIG_TC_SLIDE_FULL_EVENTS

/* Button and mode input GPIOs (pull-up, active low, from Kconfig). */
#define BTN_GPIO_COUNT  7
#define MODE_GPIO       CONFIG_TC_MODE_GPIO

/* Button index that is held on permanently while in continuous mode. */
#define CONTINUOUS_BTN  9

/* Interface colours (24-bit 0xRRGGBB, from Kconfig). */
#define COLOR_BG        lv_color_hex(CONFIG_TC_COLOR_BACKGROUND)
#define COLOR_FG        lv_color_hex(CONFIG_TC_COLOR_FOREGROUND)
#define COLOR_HL        lv_color_hex(CONFIG_TC_COLOR_HIGHLIGHT)
#define COLOR_CONT_IDLE   lv_color_hex(CONFIG_TC_COLOR_CONTINUOUS_IDLE)
#define COLOR_CONT_ACTIVE lv_color_hex(CONFIG_TC_COLOR_CONTINUOUS_ACTIVE)

/* vertical/horizontal indicator line along the edge */
#define EDGE_INDICATOR_THICKNESS 4

typedef enum {
    MODE_VERTICAL,
    MODE_HORIZONTAL,
} tc_mode_t;

/* Axis output mode, selected by the mode GPIO. */
typedef enum {
    OUTPUT_IMPULSE,
    OUTPUT_CONTINUOUS,
} tc_output_mode_t;

/* ---------------------------------------------------------------------
 * Module-level state
 * --------------------------------------------------------------------- */
static esp_lcd_panel_io_handle_t s_io_handle;
static esp_lcd_panel_handle_t    s_panel;
static esp_lcd_touch_handle_t    s_touch;
static lv_disp_t                *s_disp;
static lv_obj_t                 *s_line_right;  /* shown in VERTICAL mode   */
static lv_obj_t                 *s_line_top;    /* shown in HORIZONTAL mode */
static volatile tc_mode_t        s_mode = MODE_VERTICAL;
static volatile tc_output_mode_t s_output_mode = OUTPUT_IMPULSE;

/* Button GPIOs (buttons 0-6). */
static const int s_btn_gpios[BTN_GPIO_COUNT] = {
    CONFIG_TC_BTN0_GPIO, CONFIG_TC_BTN1_GPIO, CONFIG_TC_BTN2_GPIO,
    CONFIG_TC_BTN3_GPIO, CONFIG_TC_BTN4_GPIO, CONFIG_TC_BTN5_GPIO,
    CONFIG_TC_BTN6_GPIO,
};

/* Latest button bitmask read from the button GPIOs (buttons 0-6). */
static volatile uint16_t s_gpio_buttons;

/* Latest axis values pushed to the gamepad report. */
static volatile int16_t s_axis_x;
static volatile int16_t s_axis_y;

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

/* Configure the button and mode GPIOs as pull-up inputs. */
static void hw_buttons_init(void)
{
    uint64_t pin_mask = 0;
    for (int i = 0; i < BTN_GPIO_COUNT; i++) {
        if (s_btn_gpios[i] >= 0) {
            pin_mask |= 1ULL << s_btn_gpios[i];
        }
    }
    pin_mask |= 1ULL << MODE_GPIO;

    const gpio_config_t cfg = {
        .pin_bit_mask = pin_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    char btn_list[80];
    int  pos = 0;
    for (int i = 0; i < BTN_GPIO_COUNT; i++) {
        int remaining = (int)sizeof(btn_list) - pos;
        if (remaining <= 0) {
            break;
        }
        int n;
        if (s_btn_gpios[i] >= 0) {
            n = snprintf(btn_list + pos, (size_t)remaining,
                         "%s%d", (i == 0) ? "" : ",", s_btn_gpios[i]);
        } else {
            n = snprintf(btn_list + pos, (size_t)remaining,
                         "%soff", (i == 0) ? "" : ",");
        }
        if (n < 0 || n >= remaining) {
            break;  /* encoding error or output truncated */
        }
        pos += n;
    }
    ESP_LOGI(TAG, "Buttons 0-6 GPIO %s  mode GPIO %d", btn_list, MODE_GPIO);
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
 *  |                             ||      right edge line = VERTICAL mode
 *  +-----------------------------+  y=160  centre divider
 *  |                             ||
 *  |                             |
 *  |            v                |       down-arrow (slide = positive axis)
 *  +-----------------------------+  y=320
 *
 * The current mode is shown by a bright edge line: right edge in
 * VERTICAL mode, top edge in HORIZONTAL mode.  Game controller buttons
 * come from dedicated GPIO inputs, not from on-screen taps.
 */
static void ui_update_mode(tc_mode_t mode);
static void ui_refresh_edges(void);

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

    /* -- Centre divider line -- */
    static lv_point_t div_pts[2] = {{0, 0}, {LCD_H_RES - 1, 0}};
    lv_obj_t *divider = lv_line_create(scr);
    lv_obj_add_style(divider, &st_line, 0);
    lv_line_set_points(divider, div_pts, 2);
    /* Place so the line sits exactly at y=160 (screen centre) */
    lv_obj_set_pos(divider, 0, LCD_V_RES / 2);

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

    /* Initial mode is VERTICAL: show right edge, hide top edge, and
     * paint the edge lines with the current output mode's base colour. */
    ui_update_mode(s_mode);
    ui_refresh_edges();
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

/* Base colour of the edge lines when no gesture is active: the plain
 * foreground in impulse mode, the idle indicator colour in continuous
 * mode. */
static lv_color_t edge_base_color(void)
{
    return (s_output_mode == OUTPUT_CONTINUOUS) ? COLOR_CONT_IDLE : COLOR_FG;
}

/* Colour of the active edge line while a slide gesture is in progress:
 * the highlight colour in impulse mode, the active indicator colour in
 * continuous mode. */
static lv_color_t edge_active_color(void)
{
    return (s_output_mode == OUTPUT_CONTINUOUS) ? COLOR_CONT_ACTIVE : COLOR_HL;
}

/* Highlight (or un-highlight) an edge line for the current output mode. */
static void ui_line_highlight(lv_obj_t *line, bool on)
{
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        lv_obj_set_style_line_color(
            line, on ? edge_active_color() : edge_base_color(), 0);
        lvgl_port_unlock();
    }
}

/* Repaint both edge lines with the current output mode's base colour. */
static void ui_refresh_edges(void)
{
    if (lvgl_port_lock(pdMS_TO_TICKS(100))) {
        lv_obj_set_style_line_color(s_line_right, edge_base_color(), 0);
        lv_obj_set_style_line_color(s_line_top, edge_base_color(), 0);
        lvgl_port_unlock();
    }
}

/* ---------------------------------------------------------------------
 * Gamepad event dispatch helpers
 * --------------------------------------------------------------------- */

/* Send a report with the current axis values, merging the button GPIO
 * bitmask (buttons 0-6) and, in continuous mode, the always-on button. */
static void gamepad_flush(void)
{
    uint16_t buttons = s_gpio_buttons & 0x7F;
    if (s_output_mode == OUTPUT_CONTINUOUS) {
        buttons |= (uint16_t)(1u << CONTINUOUS_BTN);
    }
    uart_gamepad_report(s_axis_x, s_axis_y, buttons);
}

/* Send an impulse on the active axis: max value for IMPULSE_MS, then 0. */
static void send_axis_impulse(int16_t value)
{
    if (s_mode == MODE_VERTICAL) {
        s_axis_x = 0;
        s_axis_y = value;
    } else {
        s_axis_x = value;
        s_axis_y = 0;
    }
    gamepad_flush();
    vTaskDelay(pdMS_TO_TICKS(IMPULSE_MS));
    s_axis_x = 0;
    s_axis_y = 0;
    gamepad_flush();
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

/*
 * Map a finger position to a signed axis value proportional to its
 * distance from the middle of the span (half of the screen dimension).
 * The result is clamped to the valid +-32767 range.
 */
static int16_t continuous_axis_value(int pos, int span_half)
{
    if (span_half <= 0) {
        return 0;
    }
    int v = ((pos - span_half) * 32767) / span_half;
    if (v > 32767) {
        v = 32767;
    } else if (v < -32767) {
        v = -32767;
    }
    return (int16_t)v;
}

/* Toggle operating mode and update the UI indicator. */
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
 * Each poll also samples the button GPIOs (buttons 0-6) and the mode
 * GPIO (impulse vs continuous) and forwards any change to the receiver.
 *
 * Gesture rules:
 *   displacement < TAP_MAX_MOVE_PX:
 *       held >= LONG_TAP_MS  -> mode toggle (fires immediately while the
 *                               finger is still down, then the touch is
 *                               consumed and ignored on release)
 *   |dy| >= SLIDE_MIN_PX (and |dy| > |dx|):
 *       impulse mode:
 *           dy < 0 (up)   -> negative impulse(s)
 *           dy > 0 (down) -> positive impulse(s)
 *           The number of impulses scales with the slide length, up to
 *           SLIDE_FULL_EVENTS for a full-screen slide.
 *       continuous mode:
 *           the active axis tracks the finger's distance from the middle
 *           of the screen while the slide lasts; button 9 stays on.
 *   The active edge line is highlighted for the duration of the slide.
 * --------------------------------------------------------------------- */
static void touch_task(void *arg)
{
    bool     was_touching = false;
    bool     long_fired   = false;
    bool     cont_active  = false;   /* continuous slide in progress */
    uint16_t start_x = 0, start_y = 0;
    uint16_t last_x  = 0, last_y  = 0;
    int64_t  start_ms = 0;

    uint16_t         prev_buttons = 0;
    tc_output_mode_t prev_output  = s_output_mode;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TOUCH_POLL_MS));

        /* -- Sample button GPIOs (buttons 0-6, active low). -- */
        uint16_t btn_mask = 0;
        for (int i = 0; i < BTN_GPIO_COUNT; i++) {
            if (s_btn_gpios[i] >= 0 && gpio_get_level(s_btn_gpios[i]) == 0) {
                btn_mask |= (uint16_t)(1u << i);
            }
        }
        s_gpio_buttons = btn_mask;

        /* -- Sample the mode GPIO (low = continuous, high = impulse). -- */
        tc_output_mode_t out_mode = (gpio_get_level(MODE_GPIO) == 0)
                                    ? OUTPUT_CONTINUOUS : OUTPUT_IMPULSE;
        if (out_mode != prev_output) {
            s_output_mode = out_mode;
            prev_output   = out_mode;
            cont_active   = false;
            s_axis_x = 0;
            s_axis_y = 0;
            ui_refresh_edges();
            gamepad_flush();
            ESP_LOGI(TAG, "Output mode: %s",
                     (out_mode == OUTPUT_CONTINUOUS) ? "CONTINUOUS"
                                                     : "IMPULSE");
        }

        /* -- Forward button changes immediately. -- */
        if (btn_mask != prev_buttons) {
            prev_buttons = btn_mask;
            gamepad_flush();
        }

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
            cont_active  = false;
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
                ui_refresh_edges();
                long_fired = true;

            } else if (s_output_mode == OUTPUT_CONTINUOUS) {
                /* Continuous mode: track the finger's distance from the
                 * middle of the screen on the active axis. */
                lv_obj_t *edge = (s_mode == MODE_VERTICAL)
                                 ? s_line_right : s_line_top;
                int adisp     = (s_mode == MODE_VERTICAL) ? ady : adx;
                int pos       = (s_mode == MODE_VERTICAL) ? last_y : last_x;
                int span_half = (s_mode == MODE_VERTICAL)
                                ? LCD_V_RES / 2 : LCD_H_RES / 2;

                if (adisp >= SLIDE_MIN_PX) {
                    int16_t val = continuous_axis_value(pos, span_half);
                    if (s_mode == MODE_VERTICAL) {
                        s_axis_x = 0;
                        s_axis_y = val;
                    } else {
                        s_axis_x = val;
                        s_axis_y = 0;
                    }
                    gamepad_flush();
                    if (!cont_active) {
                        ui_line_highlight(edge, true);
                        cont_active = true;
                    }
                } else if (cont_active) {
                    s_axis_x = 0;
                    s_axis_y = 0;
                    gamepad_flush();
                    ui_line_highlight(edge, false);
                    cont_active = false;
                }
            }

        } else if (!touched && was_touching) {
            /* -- Touch end -- classify gesture -- */
            was_touching = false;

            if (long_fired) {
                /* Mode already switched during the hold; nothing to do. */
                continue;
            }

            if (s_output_mode == OUTPUT_CONTINUOUS) {
                /* Release the proportional axis; leave button 9 on. */
                if (cont_active) {
                    lv_obj_t *edge = (s_mode == MODE_VERTICAL)
                                     ? s_line_right : s_line_top;
                    ui_line_highlight(edge, false);
                    cont_active = false;
                }
                s_axis_x = 0;
                s_axis_y = 0;
                gamepad_flush();
                continue;
            }

            int64_t dur_ms = (esp_timer_get_time() / 1000LL) - start_ms;

            int dx   = (int)last_x - (int)start_x;
            int dy   = (int)last_y - (int)start_y;
            int adx  = dx < 0 ? -dx : dx;
            int ady  = dy < 0 ? -dy : dy;

            ESP_LOGD(TAG, "touch end (%u,%u) dur=%lldms dx=%d dy=%d",
                     last_x, last_y, dur_ms, dx, dy);

            if (ady >= SLIDE_MIN_PX && ady >= adx) {
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
            /* Stationary taps and mostly horizontal slides are ignored. */
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
    hw_buttons_init();
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
