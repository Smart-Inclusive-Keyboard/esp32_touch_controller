# esp32_touch_controller

Firmware for the **Waveshare ESP32-C6-Touch-LCD-1.47** development board
that turns the capacitive touch screen into a simple gamepad-style
navigation controller. Touch gestures and dedicated GPIO buttons are
interpreted on-device and streamed as 6-byte HID gamepad reports over a
transmit-only UART to an `esp32s3_smart_keyboard` receiver.

## Overview

The board polls its AXS5106L capacitive touch controller, classifies the
touch as a long tap or a slide, samples a set of pull-up button GPIOs,
and emits the corresponding gamepad event on UART1. The 1.47" JD9853 LCD
displays a schematic navigation guide and a bright edge line that
indicates the current axis mode.

## Hardware

Waveshare ESP32-C6-Touch-LCD-1.47 (172x320 portrait IPS):

| Function | Bus | Pins |
|----------|-----|------|
| LCD (JD9853) | SPI2 | SCLK=1, MOSI=2, MISO=3, CS=14, DC=15, RST=22, BL=23 |
| Touch (AXS5106L) | I2C0 | SDA=18, SCL=19, INT=21, RST=20 (addr 0x63) |
| Gamepad output | UART1 | TX=9, 115200 8-N-1, transmit-only |

## Button and mode inputs

Up to seven pull-up button inputs plus a mode input are configured
(defaults below, all configurable in `menuconfig`).  Buttons 4-6 are
unassigned by default; set a button GPIO to `-1` to leave it unassigned:

| Input | Default GPIO | Action |
|-------|--------------|--------|
| Button 0 | 4 | Game controller button 0 while pulled low |
| Button 1 | 5 | Game controller button 1 while pulled low |
| Button 2 | 6 | Game controller button 2 while pulled low |
| Button 3 | 7 | Game controller button 3 while pulled low |
| Button 4 | -1 (unassigned) | Game controller button 4 while pulled low |
| Button 5 | -1 (unassigned) | Game controller button 5 while pulled low |
| Button 6 | -1 (unassigned) | Game controller button 6 while pulled low |
| Mode | 8 | Select axis output mode (see below) |

Each button GPIO is active low: pulling it to ground reports the
corresponding game controller button as pressed.  A button set to `-1`
is skipped and never reports as pressed.

The **mode** GPIO is an active-low push button that toggles the axis
output mode: each press switches between **impulse** and **continuous**
mode. The button does not need to be held; the controller starts in
impulse mode.

> Note: the button GPIOs default to GPIO4-GPIO7 (buttons 0-3, with
> buttons 4-6 unassigned) and the mode GPIO to GPIO8; remap them in
> `menuconfig` to free pins as required.

## Gesture mapping

| Gesture | Action |
|---------|--------|
| Slide up | Negative axis output |
| Slide down | Positive axis output |
| Long tap (>= 500 ms) | Toggle VERTICAL / HORIZONTAL mode |

Short taps are not used; on-screen gamepad buttons come exclusively from
the button GPIOs.

In **VERTICAL** mode the axis output is sent on the Y axis; in
**HORIZONTAL** mode it is sent on the X axis.

### Impulse mode (default)

A slide that spans the full length of the screen produces several axis
events; shorter slides produce proportionally fewer events (always at
least one). The number of events for a full-length slide is configurable
via `CONFIG_TC_SLIDE_FULL_EVENTS` (default 3).

### Continuous mode

When continuous mode is selected, button 9 is held permanently on and
the active axis is set proportional to the sliding finger's distance
from the middle of the touch screen (fully deflected at the edges,
centred at the middle). Releasing the finger returns the axis to centre
while button 9 stays on. The long tap still switches between horizontal
and vertical movement, exactly as in impulse mode.

A long tap toggles the mode as soon as `CONFIG_TC_LONG_TAP_MS` elapses
while the finger is still held down -- it does not wait for the finger to
be lifted. Once the toggle has fired, the rest of that touch (including
the eventual release) is ignored. A long tap is only recognized when the
finger stays within `CONFIG_TC_TAP_MAX_MOVE_PX` of the initial touch; if
the finger slides beyond that distance the touch is treated as a slide
and holding it still afterwards will not trigger a tap.

## Mode indicator

The current mode is shown by a bright line along one edge of the screen:

- **VERTICAL mode** -- line along the **right** edge.
- **HORIZONTAL mode** -- line along the **top** edge.

While sliding, the active mode's edge line is highlighted. In impulse
mode the highlight uses `CONFIG_TC_COLOR_HIGHLIGHT` over
`CONFIG_TC_COLOR_FOREGROUND`. In continuous mode the direction indicator
uses a dedicated colour pair: `CONFIG_TC_COLOR_CONTINUOUS_IDLE` (yellow)
while idling and `CONFIG_TC_COLOR_CONTINUOUS_ACTIVE` (orange) while a
sliding gesture is detected.

## UART gamepad report

Every event sends a 6-byte report, identical to the format used by
`esp32s3_dual_foc_gp` and consumed by `esp32s3_smart_keyboard`:

```
byte 0    : buttons 0-7   (bit n = button n pressed)
byte 1    : buttons 8-9   (bits 0-1) + 6-bit padding (must be 0)
bytes 2-3 : X axis, signed 16-bit little-endian (0 = centre)
bytes 4-5 : Y axis, signed 16-bit little-endian (0 = centre)
```

## Configuration

Tunable options are exposed under **Touch Controller** in `menuconfig`:

| Option | Default | Description |
|--------|---------|-------------|
| `TC_SCREEN_BRIGHTNESS` | 50 | Backlight brightness (percent) |
| `TC_COLOR_BACKGROUND` | 0x000000 | Background colour (0xRRGGBB) |
| `TC_COLOR_FOREGROUND` | 0x33ccff | Foreground colour (lines, labels) |
| `TC_COLOR_HIGHLIGHT` | 0x3366ff | Impulse-mode slide highlight colour |
| `TC_COLOR_CONTINUOUS_IDLE` | 0xffff00 | Continuous-mode idle indicator (yellow) |
| `TC_COLOR_CONTINUOUS_ACTIVE` | 0xff8000 | Continuous-mode active indicator (orange) |
| `TC_UART_TX_GPIO` | 9 | UART TX GPIO number |
| `TC_UART_BAUD` | 115200 | UART baud rate |
| `TC_BTN0_GPIO` .. `TC_BTN6_GPIO` | 4, 5, 6, 7, -1, -1, -1 | Button 0-6 input GPIO numbers (-1 = unassigned) |
| `TC_MODE_GPIO` | 8 | Impulse/continuous mode input GPIO |
| `TC_SLIDE_MIN_PX` | 25 | Minimum travel to classify a slide |
| `TC_TAP_MAX_MOVE_PX` | 15 | Maximum movement for a long tap |
| `TC_LONG_TAP_MS` | 500 | Long-tap threshold (mode toggle) |
| `TC_SLIDE_FULL_EVENTS` | 3 | Axis events for a full-length slide (impulse mode) |

## Building and flashing

Requires ESP-IDF 5.3 or newer.

```sh
idf.py set-target esp32c6
idf.py menuconfig      # optional: adjust Touch Controller settings
idf.py build
idf.py -p <PORT> flash monitor
```

Dependencies (`espressif/esp_lcd_touch`, `espressif/esp_lvgl_port`,
`lvgl/lvgl`) are resolved automatically by the IDF component manager.

## Repository layout

```
main/                  application code (main.c, uart_gamepad.*)
main/Kconfig.projbuild  menuconfig options
components/             board LCD and touch drivers
sdkconfig.defaults      default build configuration
```

## Contributing

All source and text files in this repository must use ASCII characters
only. See [AGENTS.md](AGENTS.md) for the full guideline and a
verification command.
