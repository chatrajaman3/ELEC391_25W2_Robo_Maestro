# ILI9341 2.8" TFT LCD Driver Package

A portable, hardware-agnostic C driver for the **ILI9341** display controller covering:

| File | Purpose |
|---|---|
| `ili9341.h / .c` | LCD driver – SPI init, shapes, text |
| `xpt2046.h / .c` | Resistive touch controller |
| `sd_spi.h / .c` | SD card block I/O over SPI |
| `example_main.c` | Porting guide (STM32 HAL + Arduino) |

---

## Wiring (shared SPI bus)

```
MCU         Shield
────        ──────
MOSI  ───── SDA / MOSI
MISO  ───── SDO / MISO
SCK   ───── SCL / SCK
PA4   ───── LCD_CS
PA3   ───── LCD_RST
PA2   ───── LCD_DC
PB0   ───── T_CS   (Touch chip-select)
PB1   ───── T_IRQ  (optional)
PB2   ───── SD_CS
3.3V  ───── VCC, LED
GND   ───── GND
```

All three devices share MOSI/MISO/SCK. Each has its own CS line to
allow multiplexing. **Ensure only one CS is active at a time.**

---

## Porting: implement 3 callbacks per device

### LCD (`ILI9341_Config`)

```c
ILI9341_Config cfg = {
    .spi_txrx = my_spi_transfer,   // uint8_t fn(uint8_t)
    .set_cs   = my_lcd_cs,         // void fn(int level)
    .set_dc   = my_lcd_dc,
    .set_rst  = my_lcd_rst,
    .delay_ms = my_delay,          // void fn(uint32_t ms)
    .rotation = ROTATION_0,
};
ili9341_init(&cfg);
```

### Touch (`XPT2046_Config`)

```c
XPT2046_Config tch = {
    .spi_txrx = my_spi_transfer,
    .set_cs   = my_tch_cs,
    .read_irq = my_irq_read,       // int fn(void), NULL if unused
};
xpt2046_init(&tch);
xpt2046_default_cal(240, 320);    // or call xpt2046_set_cal() after calibrating
```

### SD Card (`SD_Config`)

```c
SD_Config sd = {
    .spi_txrx    = my_spi_transfer,
    .spi_set_slow = my_400khz,     // called during init
    .spi_set_fast = my_25mhz,      // called after init
    .set_cs      = my_sd_cs,
    .delay_ms    = my_delay,
};
SD_Info info;
sd_init(&sd, &info);
```

---

## LCD API Quick Reference

```c
// Screen
ili9341_fill_screen(COLOR_BLACK);
ili9341_set_rotation(ROTATION_90);

// Shapes
ili9341_fill_rect(x, y, w, h, COLOR_BLUE);
ili9341_draw_rect(x, y, w, h, COLOR_WHITE);
ili9341_fill_circle(cx, cy, r, COLOR_RED);
ili9341_draw_circle(cx, cy, r, COLOR_WHITE);
ili9341_fill_triangle(x0,y0, x1,y1, x2,y2, COLOR_GREEN);
ili9341_draw_line(x0, y0, x1, y1, COLOR_GRAY);
ili9341_fill_rounded_rect(x, y, w, h, radius, COLOR_MAGENTA);

// Text (built-in 5×7 bitmap font)
ili9341_set_font_scale(2);                      // 2× scale
ili9341_set_text_color(COLOR_WHITE, COLOR_BLACK);
ili9341_draw_string(10, 10, "Hello World");
ili9341_draw_string_wrap(10, 100, 200, long_str);  // wraps at 200 px

// Pixel colours  – RGB565 macro
uint16_t orange = RGB565(255, 165, 0);
```

---

## Touch API

```c
TouchPoint tp;
if (xpt2046_read(&tp)) {
    // tp.x, tp.y  in screen pixels
    // tp.z        pressure (higher = harder)
}
```

### Calibration

1. Call `xpt2046_raw(&rx, &ry, &rz)` while the user touches two known screen corners.
2. Fill an `XPT2046_Cal` struct with the measured extremes and screen dimensions.
3. Call `xpt2046_set_cal(&cal)`.

Or use `xpt2046_default_cal(w, h)` for typical 2.8" panel defaults.

---

## SD API

```c
uint8_t buf[512];

// Single block
sd_read_block(block_addr, buf);
sd_write_block(block_addr, buf);

// Multi-block (faster)
sd_read_blocks(start_block, buf, count);
sd_write_blocks(start_block, buf, count);

// Card info
const SD_Info *info = sd_get_info();
// info->capacity_mb, info->block_count, info->type
```

SD_Error return codes: `SD_OK`, `SD_ERR_TIMEOUT`, `SD_ERR_INIT`, `SD_ERR_CMD`,
`SD_ERR_CRC`, `SD_ERR_PARAM`, `SD_ERR_WRITE`, `SD_ERR_READ`.

Use `sd_error_str(err)` for a human-readable description.

---

## Predefined colours

```c
COLOR_BLACK  COLOR_WHITE  COLOR_RED    COLOR_GREEN
COLOR_BLUE   COLOR_YELLOW COLOR_CYAN   COLOR_MAGENTA
COLOR_ORANGE COLOR_GRAY   COLOR_DARKGRAY
RGB565(r, g, b)   // custom 16-bit colour
```

---

## Rotations

| Constant | Orientation |
|---|---|
| `ROTATION_0`   | Portrait  (240×320) – USB at bottom |
| `ROTATION_90`  | Landscape (320×240) – USB at right |
| `ROTATION_180` | Portrait  (240×320) – USB at top   |
| `ROTATION_270` | Landscape (320×240) – USB at left  |

Set `swap_xy = true` in `XPT2046_Cal` for landscape touch modes.

---

## Notes

- **SPI speed**: ILI9341 supports up to ~40 MHz writes; XPT2046 is capped at
  ~2 MHz; SD card data mode up to 25 MHz. If sharing one SPI peripheral,
  set the clock for the slowest device in use, or reconfigure per transfer.
- **3.3 V logic**: All three ICs require 3.3 V. Use a level shifter if your
  MCU is 5 V.
- **SD file system**: This driver provides raw block access. Layer FatFs
  (ELM Chan) or LittleFS on top for file-level access.
- **Thread safety**: Not built-in. Wrap calls in a mutex if using an RTOS.
