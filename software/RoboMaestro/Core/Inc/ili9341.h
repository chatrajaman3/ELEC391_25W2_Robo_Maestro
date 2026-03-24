#ifndef ILI9341_H
#define ILI9341_H

/**
 * ILI9341 2.8" TFT LCD Driver
 * Interface: 4-wire SPI (240x320, 262K colors)
 *
 * Wiring (Arduino / STM32 HAL compatible):
 *   LCD_CS   -> chip select for ILI9341
 *   LCD_DC   -> data/command select (D/CX)
 *   LCD_RST  -> hardware reset (RESX)
 *   LCD_MOSI -> SPI MOSI (SDA)
 *   LCD_MISO -> SPI MISO (SDO)
 *   LCD_SCK  -> SPI clock (SCL / D/CX pin)
 *
 *   TOUCH_CS -> chip select for XPT2046 touch controller
 *   SD_CS    -> chip select for SD card
 *
 * All three devices share the same SPI bus; CS lines differentiate them.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ── Display dimensions ─────────────────────────────────────────────────── */
#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320

/* ── ILI9341 command set (from datasheet §8) ────────────────────────────── */
#define ILI9341_NOP          0x00
#define ILI9341_SWRESET      0x01
#define ILI9341_RDDID        0x04
#define ILI9341_RDDST        0x09
#define ILI9341_SLPIN        0x10
#define ILI9341_SLPOUT       0x11
#define ILI9341_PTLON        0x12
#define ILI9341_NORON        0x13
#define ILI9341_INVOFF       0x20
#define ILI9341_INVON        0x21
#define ILI9341_GAMMASET     0x26
#define ILI9341_DISPOFF      0x28
#define ILI9341_DISPON       0x29
#define ILI9341_CASET        0x2A  /* Column Address Set   */
#define ILI9341_PASET        0x2B  /* Page (Row) Address Set */
#define ILI9341_RAMWR        0x2C  /* Memory Write         */
#define ILI9341_RAMRD        0x2E  /* Memory Read          */
#define ILI9341_PTLAR        0x30
#define ILI9341_VSCRDEF      0x33
#define ILI9341_MADCTL       0x36  /* Memory Access Control */
#define ILI9341_VSCRSADD     0x37
#define ILI9341_PIXFMT       0x3A  /* Pixel Format Set     */
#define ILI9341_WRDISBV      0x51
#define ILI9341_RDDISBV      0x52
#define ILI9341_WRCTRLD      0x53
#define ILI9341_FRMCTR1      0xB1
#define ILI9341_FRMCTR2      0xB2
#define ILI9341_FRMCTR3      0xB3
#define ILI9341_INVCTR       0xB4
#define ILI9341_DFUNCTR      0xB6
#define ILI9341_PWCTR1       0xC0
#define ILI9341_PWCTR2       0xC1
#define ILI9341_VMCTR1       0xC5
#define ILI9341_VMCTR2       0xC7
#define ILI9341_RDID4        0xD3
#define ILI9341_GMCTRP1      0xE0  /* Positive Gamma Correction */
#define ILI9341_GMCTRN1      0xE1  /* Negative Gamma Correction */
#define ILI9341_PWCTRA       0xCB
#define ILI9341_PWCTRB       0xCF
#define ILI9341_DTCTRA       0xE8
#define ILI9341_DTCTRB       0xEA
#define ILI9341_PWSEQCTR     0xED
#define ILI9341_ENABLE3G     0xF2
#define ILI9341_PUMPRTCTR    0xF7

/* ── MADCTL rotation bits ───────────────────────────────────────────────── */
#define MADCTL_MY   0x80   /* Row address order    */
#define MADCTL_MX   0x40   /* Column address order */
#define MADCTL_MV   0x20   /* Row/Col exchange     */
#define MADCTL_ML   0x10   /* Vertical refresh     */
#define MADCTL_BGR  0x08   /* Blue-Green-Red order */
#define MADCTL_MH   0x04   /* Horizontal refresh   */

/* ── 16-bit RGB565 color helpers ────────────────────────────────────────── */
#define RGB565(r, g, b) \
    ((uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)))

/* Common colors */
#define COLOR_BLACK    0x0000
#define COLOR_WHITE    0xFFFF
#define COLOR_RED      0xF800
#define COLOR_GREEN    0x07E0
#define COLOR_BLUE     0x001F
#define COLOR_YELLOW   0xFFE0
#define COLOR_CYAN     0x07FF
#define COLOR_MAGENTA  0xF81F
#define COLOR_ORANGE   0xFD20
#define COLOR_GRAY     0x8410
#define COLOR_DARKGRAY 0x4208

/* ── Screen rotation ────────────────────────────────────────────────────── */
typedef enum {
    ROTATION_0   = 0,   /* Portrait,  USB at bottom */
    ROTATION_90  = 1,   /* Landscape, USB at right  */
    ROTATION_180 = 2,   /* Portrait,  USB at top    */
    ROTATION_270 = 3,   /* Landscape, USB at left   */
} ILI9341_Rotation;

/* ── Driver configuration (fill before calling ili9341_init) ────────────── */
typedef struct {
    /* SPI send/receive byte – must be implemented by the user */
    uint8_t  (*spi_txrx)(uint8_t byte);

    /* GPIO control – 1 = high, 0 = low */
    void     (*set_cs)(int level);   /* LCD chip select  */
    void     (*set_dc)(int level);   /* Data / Command   */
    void     (*set_rst)(int level);  /* Hardware reset   */

    /* Millisecond delay */
    void     (*delay_ms)(uint32_t ms);

    ILI9341_Rotation rotation;
} ILI9341_Config;

/* ── Public API ─────────────────────────────────────────────────────────── */

/* Initialisation */
void ili9341_init(const ILI9341_Config *cfg);
void ili9341_set_rotation(ILI9341_Rotation r);

/* Low-level pixel operations */
void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void ili9341_write_pixel(uint16_t x, uint16_t y, uint16_t color);
void ili9341_flood(uint16_t color, uint32_t count);

/* Shapes */
void ili9341_fill_screen(uint16_t color);
void ili9341_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ili9341_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ili9341_draw_circle(int16_t cx, int16_t cy, int16_t r, uint16_t color);
void ili9341_fill_circle(int16_t cx, int16_t cy, int16_t r, uint16_t color);
void ili9341_draw_triangle(int16_t x0, int16_t y0,
                           int16_t x1, int16_t y1,
                           int16_t x2, int16_t y2, uint16_t color);
void ili9341_fill_triangle(int16_t x0, int16_t y0,
                           int16_t x1, int16_t y1,
                           int16_t x2, int16_t y2, uint16_t color);
void ili9341_draw_rounded_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               uint16_t r, uint16_t color);
void ili9341_fill_rounded_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               uint16_t r, uint16_t color);

/* Text */
void ili9341_set_font_scale(uint8_t scale);
void ili9341_set_text_color(uint16_t fg, uint16_t bg);
void ili9341_draw_char(uint16_t x, uint16_t y, char c);
void ili9341_draw_string(uint16_t x, uint16_t y, const char *str);
void ili9341_draw_string_wrap(uint16_t x, uint16_t y, uint16_t max_w,
                              const char *str);
uint16_t ili9341_string_width(const char *str);

/* Display control */
void ili9341_display_on(void);
void ili9341_display_off(void);
void ili9341_invert(bool on);
void ili9341_set_brightness(uint8_t level);
void ili9341_sleep(bool on);
void ili9341_scroll(uint16_t vsp);

/* Screen dimensions (change with rotation) */
uint16_t ili9341_get_width(void);
uint16_t ili9341_get_height(void);

#endif /* ILI9341_H */
