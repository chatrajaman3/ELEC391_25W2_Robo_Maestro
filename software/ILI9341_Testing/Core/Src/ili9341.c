/**
 * ILI9341 2.8" TFT LCD Driver – Implementation
 *
 * Covers (per ILI9341 datasheet V1.11):
 *   – 4-wire SPI initialisation sequence  (§7.1.8 / §8)
 *   – CASET / PASET / RAMWR window addressing  (§8.2.20–22)
 *   – MADCTL rotation  (§8.2.29)
 *   – Full shape & text rendering
 *
 * Built-in 5×7 pixel font (ASCII 0x20–0x7E).
 */


#include "ili9341.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ── Internal state ──────────────────────────────────────────────────────── */
static ILI9341_Config  _cfg;
static uint16_t        _width  = ILI9341_WIDTH;
static uint16_t        _height = ILI9341_HEIGHT;
static uint8_t         _font_scale = 1;
static uint16_t        _fg_color   = COLOR_WHITE;
static uint16_t        _bg_color   = COLOR_BLACK;

/* ── Built-in 5×7 font (printable ASCII 0x20–0x7E) ──────────────────────── */
#define FONT_W 5
#define FONT_H 7
static const uint8_t font5x7[][FONT_W] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x10,0x08,0x08,0x10,0x08}, /* '~' */
};

/* ── Low-level SPI helpers ───────────────────────────────────────────────── */

static inline void _write_byte(uint8_t b)
{
    _cfg.spi_txrx(b);
}

static inline void _write_cmd(uint8_t cmd)
{
    _cfg.set_dc(0);          /* Command mode (D/CX = 0) */
    _cfg.set_cs(0);
    _write_byte(cmd);
    _cfg.set_cs(1);
}

static inline void _write_data8(uint8_t data)
{
    _cfg.set_dc(1);          /* Data mode (D/CX = 1) */
    _cfg.set_cs(0);
    _write_byte(data);
    _cfg.set_cs(1);
}

static inline void _write_data16(uint16_t data)
{
    _cfg.set_dc(1);
    _cfg.set_cs(0);
    _write_byte(data >> 8);
    _write_byte(data & 0xFF);
    _cfg.set_cs(1);
}

/* Send a command followed by N data bytes (no CS toggling between them). */
static void _cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    _cfg.set_dc(0);
    _cfg.set_cs(0);
    _write_byte(cmd);
    _cfg.set_dc(1);
    for (size_t i = 0; i < len; i++) _write_byte(data[i]);
    _cfg.set_cs(1);
}

/* ── Initialisation sequence ─────────────────────────────────────────────── */
/*
 * Register values derived from ILI9341 datasheet §8.3 / §8.4
 * and the widely validated "Adafruit" init sequence for 2.8" screens.
 */
void ili9341_init(const ILI9341_Config *cfg)
{
    _cfg = *cfg;
    _width  = ILI9341_WIDTH;
    _height = ILI9341_HEIGHT;

    /* Hardware reset – active-low RESX pulse  (§15.4) */
    _cfg.set_rst(1);
    _cfg.delay_ms(5);
    _cfg.set_rst(0);
    _cfg.delay_ms(20);
    _cfg.set_rst(1);
    _cfg.delay_ms(150);

    /* Extended power / timing registers (§8.4) */
    _cmd_data(ILI9341_PWCTRA,  (uint8_t[]){0x39,0x2C,0x00,0x34,0x02}, 5);
    _cmd_data(ILI9341_PWCTRB,  (uint8_t[]){0x00,0xC1,0x30},           3);
    _cmd_data(ILI9341_DTCTRA,  (uint8_t[]){0x85,0x00,0x78},           3);
    _cmd_data(ILI9341_DTCTRB,  (uint8_t[]){0x00,0x00},                2);
    _cmd_data(ILI9341_PWSEQCTR,(uint8_t[]){0x64,0x03,0x12,0x81},      4);
    _cmd_data(ILI9341_ENABLE3G,(uint8_t[]){0x00},                      1);
    _cmd_data(ILI9341_PUMPRTCTR,(uint8_t[]){0x20},                     1);

    /* Power control §8.3.16–18 */
    _cmd_data(ILI9341_PWCTR1,  (uint8_t[]){0x23},             1);  /* VRH=4.60V  */
    _cmd_data(ILI9341_PWCTR2,  (uint8_t[]){0x10},             1);  /* SAP, BT=4  */
    _cmd_data(ILI9341_VMCTR1,  (uint8_t[]){0x3E,0x28},        2);  /* VCOMH/L    */
    _cmd_data(ILI9341_VMCTR2,  (uint8_t[]){0x86},             1);  /* –58        */

    /* Memory Access Control – portrait, BGR order §8.2.29 */
    _cmd_data(ILI9341_MADCTL,  (uint8_t[]){0x48},             1);

    /* Pixel format: 16-bit RGB565  §8.2.33 */
    _cmd_data(ILI9341_PIXFMT,  (uint8_t[]){0x55},             1);

    /* Frame rate: ~70 Hz  §8.3.2 */
    _cmd_data(ILI9341_FRMCTR1, (uint8_t[]){0x00,0x18},        2);

    /* Display function control §8.3.7 */
    _cmd_data(ILI9341_DFUNCTR, (uint8_t[]){0x08,0x82,0x27},   3);

    /* Gamma §8.2.17 */
    _cmd_data(ILI9341_GAMMASET,(uint8_t[]){0x01},              1);

    /* Positive gamma §8.3.24 */
    _cmd_data(ILI9341_GMCTRP1,
        (uint8_t[]){0x0F,0x31,0x2B,0x0C,0x0E,0x08,
                    0x4E,0xF1,0x37,0x07,0x10,0x03,
                    0x0E,0x09,0x00}, 15);

    /* Negative gamma §8.3.25 */
    _cmd_data(ILI9341_GMCTRN1,
        (uint8_t[]){0x00,0x0E,0x14,0x03,0x11,0x07,
                    0x31,0xC1,0x48,0x08,0x0F,0x0C,
                    0x31,0x36,0x0F}, 15);

    /* Sleep out (§8.2.12) – must wait 120 ms before sending more commands */
    _write_cmd(ILI9341_SLPOUT);
    _cfg.delay_ms(120);

    /* Display on (§8.2.19) */
    _write_cmd(ILI9341_DISPON);
    _cfg.delay_ms(10);

    /* Apply user-selected rotation */
    ili9341_set_rotation(cfg->rotation);
}

/* ── Rotation ────────────────────────────────────────────────────────────── */
void ili9341_set_rotation(ILI9341_Rotation r)
{
    uint8_t madctl = MADCTL_BGR;
    switch (r) {
        case ROTATION_0:
            madctl |= MADCTL_MX;
            _width  = ILI9341_WIDTH;
            _height = ILI9341_HEIGHT;
            break;
        case ROTATION_90:
            madctl |= MADCTL_MV;
            _width  = ILI9341_HEIGHT;
            _height = ILI9341_WIDTH;
            break;
        case ROTATION_180:
            madctl |= MADCTL_MY;
            _width  = ILI9341_WIDTH;
            _height = ILI9341_HEIGHT;
            break;
        case ROTATION_270:
            madctl |= MADCTL_MX | MADCTL_MY | MADCTL_MV;
            _width  = ILI9341_HEIGHT;
            _height = ILI9341_WIDTH;
            break;
    }
    _cmd_data(ILI9341_MADCTL, &madctl, 1);
}

uint16_t ili9341_get_width(void)  { return _width;  }
uint16_t ili9341_get_height(void) { return _height; }

/* ── Window & pixel helpers ──────────────────────────────────────────────── */

/*
 * CASET / PASET define the rectangular write window (§8.2.20–21).
 * After this, RAMWR pixel data fills the window left-to-right, top-to-bottom.
 */
void ili9341_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t col_data[4] = { x0>>8, x0&0xFF, x1>>8, x1&0xFF };
    uint8_t row_data[4] = { y0>>8, y0&0xFF, y1>>8, y1&0xFF };
    _cmd_data(ILI9341_CASET, col_data, 4);
    _cmd_data(ILI9341_PASET, row_data, 4);
    _write_cmd(ILI9341_RAMWR);
    _cfg.set_dc(1);   /* stay in data mode for the following pixel writes */
    _cfg.set_cs(0);
}

/* Write 'count' copies of 'color' to GRAM; CS must already be low. */
void ili9341_flood(uint16_t color, uint32_t count)
{
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (count--) {
        _write_byte(hi);
        _write_byte(lo);
    }
    _cfg.set_cs(1);
}

void ili9341_write_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= _width || y >= _height) return;
    ili9341_set_window(x, y, x, y);
    _write_byte(color >> 8);
    _write_byte(color & 0xFF);
    _cfg.set_cs(1);
}

/* ── Fill operations ─────────────────────────────────────────────────────── */

void ili9341_fill_screen(uint16_t color)
{
    ili9341_set_window(0, 0, _width - 1, _height - 1);
    ili9341_flood(color, (uint32_t)_width * _height);
}

void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= _width || y >= _height) return;
    if (x + w > _width)  w = _width  - x;
    if (y + h > _height) h = _height - y;
    ili9341_set_window(x, y, x + w - 1, y + h - 1);
    ili9341_flood(color, (uint32_t)w * h);
}

/* ── Lines ───────────────────────────────────────────────────────────────── */

/* Bresenham's line algorithm */
void ili9341_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int16_t dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int16_t err = dx + dy;
    while (1) {
        ili9341_write_pixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int16_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* ── Rectangles ──────────────────────────────────────────────────────────── */

void ili9341_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    ili9341_draw_line(x,         y,         x + w - 1, y,         color);
    ili9341_draw_line(x,         y + h - 1, x + w - 1, y + h - 1, color);
    ili9341_draw_line(x,         y,         x,         y + h - 1, color);
    ili9341_draw_line(x + w - 1, y,         x + w - 1, y + h - 1, color);
}

/* ── Circles ─────────────────────────────────────────────────────────────── */

/* Midpoint circle – helper draws 8-way symmetric points */
static void _circle_pts(int16_t cx, int16_t cy, int16_t x, int16_t y, uint16_t color)
{
    ili9341_write_pixel(cx+x, cy+y, color);
    ili9341_write_pixel(cx-x, cy+y, color);
    ili9341_write_pixel(cx+x, cy-y, color);
    ili9341_write_pixel(cx-x, cy-y, color);
    ili9341_write_pixel(cx+y, cy+x, color);
    ili9341_write_pixel(cx-y, cy+x, color);
    ili9341_write_pixel(cx+y, cy-x, color);
    ili9341_write_pixel(cx-y, cy-x, color);
}

void ili9341_draw_circle(int16_t cx, int16_t cy, int16_t r, uint16_t color)
{
    int16_t x = 0, y = r, d = 3 - 2 * r;
    while (x <= y) {
        _circle_pts(cx, cy, x, y, color);
        if (d < 0) d += 4*x + 6;
        else       { d += 4*(x - y) + 10; y--; }
        x++;
    }
}

void ili9341_fill_circle(int16_t cx, int16_t cy, int16_t r, uint16_t color)
{
    for (int16_t dy = -r; dy <= r; dy++) {
        int16_t dx = (int16_t)sqrt((double)(r*r - dy*dy));
        ili9341_fill_rect(cx - dx, cy + dy, 2*dx + 1, 1, color);
    }
}

/* ── Triangles ───────────────────────────────────────────────────────────── */

void ili9341_draw_triangle(int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1,
                            int16_t x2, int16_t y2, uint16_t color)
{
    ili9341_draw_line(x0, y0, x1, y1, color);
    ili9341_draw_line(x1, y1, x2, y2, color);
    ili9341_draw_line(x2, y2, x0, y0, color);
}

/* Flat-bottom / flat-top scanline fill */
static void _fill_triangle_half(int16_t x0, int16_t y0,
                                 int16_t x1, int16_t y1,
                                 int16_t x2, int16_t y2, uint16_t color)
{
    /* Assumes y1 == y2 (flat bottom) or y0 == y1 (flat top) */
    int16_t dy  = y2 - y0;
    for (int16_t y = y0; y <= y2; y++) {
        int16_t xa = x0 + (x1 - x0) * (y - y0) / (y1 - y0 + (y1==y0));
        int16_t xb = x0 + (x2 - x0) * (y - y0) / (dy ? dy : 1);
        if (xa > xb) { int16_t t = xa; xa = xb; xb = t; }
        ili9341_fill_rect(xa, y, xb - xa + 1, 1, color);
    }
}

void ili9341_fill_triangle(int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1,
                            int16_t x2, int16_t y2, uint16_t color)
{
    /* Sort vertices by y (bubble) */
    if (y0 > y1) { int16_t t; t=y0;y0=y1;y1=t; t=x0;x0=x1;x1=t; }
    if (y1 > y2) { int16_t t; t=y1;y1=y2;y2=t; t=x1;x1=x2;x2=t; }
    if (y0 > y1) { int16_t t; t=y0;y0=y1;y1=t; t=x0;x0=x1;x1=t; }

    if (y0 == y2) {   /* Degenerate – horizontal line */
        int16_t minX = x0, maxX = x0;
        if (x1 < minX) minX = x1; if (x1 > maxX) maxX = x1;
        if (x2 < minX) minX = x2; if (x2 > maxX) maxX = x2;
        ili9341_fill_rect(minX, y0, maxX - minX + 1, 1, color);
        return;
    }

    /* Split into flat-bottom + flat-top */
    int16_t x3 = x0 + (int32_t)(y1 - y0) * (x2 - x0) / (y2 - y0);

    /* Flat-bottom triangle */
    {
        int16_t xa = x1, xb = x3;
        for (int16_t y = y0; y <= y1; y++) {
            int16_t a = x0 + (xa - x0) * (y - y0) / ((y1 - y0) ? (y1-y0) : 1);
            int16_t b = x0 + (xb - x0) * (y - y0) / ((y1 - y0) ? (y1-y0) : 1);
            if (a > b) { int16_t t = a; a = b; b = t; }
            ili9341_fill_rect(a, y, b - a + 1, 1, color);
        }
    }
    /* Flat-top triangle */
    {
        int16_t xa = x1, xb = x3;
        for (int16_t y = y1; y <= y2; y++) {
            int16_t a = xa + (x2 - xa) * (y - y1) / ((y2 - y1) ? (y2-y1) : 1);
            int16_t b = xb + (x2 - xb) * (y - y1) / ((y2 - y1) ? (y2-y1) : 1);
            if (a > b) { int16_t t = a; a = b; b = t; }
            ili9341_fill_rect(a, y, b - a + 1, 1, color);
        }
    }
}

/* ── Rounded rectangles ──────────────────────────────────────────────────── */

/* Quarter-circle helper – draws pixels in the specified octant(s) */
static void _round_corner(int16_t cx, int16_t cy, int16_t r,
                           uint8_t corner, uint16_t color)
{
    int16_t x = 0, y = r, d = 3 - 2 * r;
    while (x <= y) {
        if (corner & 0x1) {
            ili9341_write_pixel(cx + x, cy - y, color);
            ili9341_write_pixel(cx + y, cy - x, color);
        }
        if (corner & 0x2) {
            ili9341_write_pixel(cx - x, cy - y, color);
            ili9341_write_pixel(cx - y, cy - x, color);
        }
        if (corner & 0x4) {
            ili9341_write_pixel(cx - x, cy + y, color);
            ili9341_write_pixel(cx - y, cy + x, color);
        }
        if (corner & 0x8) {
            ili9341_write_pixel(cx + x, cy + y, color);
            ili9341_write_pixel(cx + y, cy + x, color);
        }
        if (d < 0) d += 4*x + 6;
        else       { d += 4*(x - y) + 10; y--; }
        x++;
    }
}

static void _fill_round_corner(int16_t cx, int16_t cy, int16_t r,
                                uint8_t corner, uint16_t color)
{
    int16_t x = 0, y = r, d = 3 - 2 * r;
    while (x <= y) {
        if (corner & 0x1) {
            ili9341_fill_rect(cx,     cy - y, x + 1, 1, color);
            ili9341_fill_rect(cx,     cy - x, y + 1, 1, color);
        }
        if (corner & 0x2) {
            ili9341_fill_rect(cx - x, cy - y, x + 1, 1, color);
            ili9341_fill_rect(cx - y, cy - x, y + 1, 1, color);
        }
        if (corner & 0x4) {
            ili9341_fill_rect(cx - x, cy,     x + 1, 1, color);
            ili9341_fill_rect(cx - y, cy,     y + 1, 1, color);
        }
        if (corner & 0x8) {
            ili9341_fill_rect(cx,     cy,     x + 1, 1, color);
            ili9341_fill_rect(cx,     cy,     y + 1, 1, color);
        }
        if (d < 0) d += 4*x + 6;
        else       { d += 4*(x - y) + 10; y--; }
        x++;
    }
}

void ili9341_draw_rounded_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                uint16_t r, uint16_t color)
{
    ili9341_draw_line(x+r,   y,       x+w-r-1, y,       color);
    ili9341_draw_line(x+r,   y+h-1,   x+w-r-1, y+h-1,   color);
    ili9341_draw_line(x,     y+r,     x,       y+h-r-1, color);
    ili9341_draw_line(x+w-1, y+r,     x+w-1,   y+h-r-1, color);
    _round_corner(x+r,     y+r,     r, 0x2, color);
    _round_corner(x+w-r-1, y+r,     r, 0x1, color);
    _round_corner(x+w-r-1, y+h-r-1, r, 0x8, color);
    _round_corner(x+r,     y+h-r-1, r, 0x4, color);
}

void ili9341_fill_rounded_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                                uint16_t r, uint16_t color)
{
    ili9341_fill_rect(x+r, y,   w-2*r, h,   color);  /* Centre */
    _fill_round_corner(x+r,     y+r,     r, 0x2, color);
    _fill_round_corner(x+w-r-1, y+r,     r, 0x1, color);
    _fill_round_corner(x+w-r-1, y+h-r-1, r, 0x8, color);
    _fill_round_corner(x+r,     y+h-r-1, r, 0x4, color);
    ili9341_fill_rect(x, y+r, r, h-2*r, color);
    ili9341_fill_rect(x+w-r, y+r, r, h-2*r, color);
}

/* ── Text ────────────────────────────────────────────────────────────────── */

void ili9341_set_font_scale(uint8_t scale) { _font_scale = scale ? scale : 1; }
void ili9341_set_text_color(uint16_t fg, uint16_t bg)
{
    _fg_color = fg;
    _bg_color = bg;
}

/*
 * Each glyph is FONT_W columns × FONT_H rows, 1 bit per pixel.
 * font5x7[ch][col] bit 0 = top row, bit 6 = bottom row.
 */
void ili9341_draw_char(uint16_t x, uint16_t y, char c)
{
    if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) c = '?';
    const uint8_t *glyph = font5x7[(uint8_t)c - 0x20];

    uint16_t sw = (FONT_W + 1) * _font_scale;  /* +1 for inter-char gap */
    uint16_t sh = FONT_H * _font_scale;

    ili9341_set_window(x, y, x + sw - 1, y + sh - 1);

    for (uint8_t row = 0; row < FONT_H; row++) {
        for (uint8_t rs = 0; rs < _font_scale; rs++) {
            for (uint8_t col = 0; col < FONT_W; col++) {
                uint16_t pix = (glyph[col] >> row) & 1 ? _fg_color : _bg_color;
                for (uint8_t cs = 0; cs < _font_scale; cs++) {
                    _write_byte(pix >> 8);
                    _write_byte(pix & 0xFF);
                }
            }
            /* Inter-character column gap */
            for (uint8_t cs = 0; cs < _font_scale; cs++) {
                _write_byte(_bg_color >> 8);
                _write_byte(_bg_color & 0xFF);
            }
        }
    }
    _cfg.set_cs(1);
}

uint16_t ili9341_string_width(const char *str)
{
    return (FONT_W + 1) * _font_scale * (uint16_t)strlen(str);
}

void ili9341_draw_string(uint16_t x, uint16_t y, const char *str)
{
    uint16_t cx = x;
    while (*str) {
        ili9341_draw_char(cx, y, *str++);
        cx += (FONT_W + 1) * _font_scale;
    }
}

void ili9341_draw_string_wrap(uint16_t x, uint16_t y, uint16_t max_w,
                               const char *str)
{
    uint16_t cx = x, cy = y;
    uint16_t char_w = (FONT_W + 1) * _font_scale;
    uint16_t char_h = (FONT_H + 1) * _font_scale;

    while (*str) {
        if (*str == '\n' || cx + char_w > x + max_w) {
            cx = x;
            cy += char_h;
            if (*str == '\n') { str++; continue; }
        }
        ili9341_draw_char(cx, cy, *str++);
        cx += char_w;
    }
}

/* ── Display control ─────────────────────────────────────────────────────── */

void ili9341_display_on(void)   { _write_cmd(ILI9341_DISPON);  }
void ili9341_display_off(void)  { _write_cmd(ILI9341_DISPOFF); }

void ili9341_invert(bool on)
{
    _write_cmd(on ? ILI9341_INVON : ILI9341_INVOFF);
}

void ili9341_set_brightness(uint8_t level)
{
    _cmd_data(ILI9341_WRDISBV, &level, 1);
}

void ili9341_sleep(bool on)
{
    _write_cmd(on ? ILI9341_SLPIN : ILI9341_SLPOUT);
    _cfg.delay_ms(120);
}

void ili9341_scroll(uint16_t vsp)
{
    uint8_t d[2] = { vsp >> 8, vsp & 0xFF };
    _cmd_data(ILI9341_VSCRSADD, d, 2);
}
