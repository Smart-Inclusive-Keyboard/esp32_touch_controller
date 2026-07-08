# esp32_touch_controller

Firmware for the **Waveshare ESP32-C6-Touch-LCD-1.47** development board
that turns the capacitive touch screen into a simple gamepad-style
navigation controller. Touch gestures are interpreted on-device and
streamed as 6-byte HID gamepad reports over a transmit-only UART to an
`esp32s3_smart_keyboard` receiver.

## Overview

The board polls its AXS5106L capacitive touch controller, classifies the
touch as a tap or a slide, and emits the corresponding gamepad event on
UART1. The 1.47" JD9853 LCD displays a schematic navigation guide and a
bright edge line that indicates the current axis mode.

## Hardware

Waveshare ESP32-C6-Touch-LCD-1.47 (172x320 portrait IPS):

| Function | Bus | Pins |
|----------|-----|------|
| LCD (JD9853) | SPI2 | SCLK=1, MOSI=2, MISO=3, CS=14, DC=15, RST=22, BL=23 |
| Touch (AXS5106L) | I2C0 | SDA=18, SCL=19, INT=21, RST=20 (addr 0x63) |
| Gamepad output | UART1 | TX=13, 115200 8-N-1, transmit-only |

## Gesture mapping

| Gesture | Action |
|---------|--------|
| Slide up | Negative axis impulse(s) |
| Slide down | Positive axis impulse(s) |
| Short tap, upper half | Button 1 press + release |
| Short tap, lower half | Button 0 press + release |
| Long tap (>= 500 ms) | Toggle VERTICAL / HORIZONTAL mode |

In **VERTICAL** mode the impulse is sent on the Y axis; in **HORIZONTAL**
mode it is sent on the X axis.

A slide that spans the full length of the screen produces several axis
events; shorter slides produce proportionally fewer events (always at
least one). The number of events for a full-length slide is configurable
via `CONFIG_TC_SLIDE_FULL_EVENTS` (default 3).

## Mode indicator

The current mode is shown by a bright line along one edge of the screen:

- **VERTICAL mode** -- line along the **right** edge.
- **HORIZONTAL mode** -- line along the **top** edge.

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
| `TC_UART_TX_GPIO` | 13 | UART TX GPIO number |
| `TC_UART_BAUD` | 115200 | UART baud rate |
| `TC_SLIDE_MIN_PX` | 25 | Minimum travel to classify a slide |
| `TC_TAP_MAX_MOVE_PX` | 15 | Maximum movement for a tap |
| `TC_LONG_TAP_MS` | 500 | Long-tap threshold (mode toggle) |
| `TC_SLIDE_FULL_EVENTS` | 3 | Axis events for a full-length slide |

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
