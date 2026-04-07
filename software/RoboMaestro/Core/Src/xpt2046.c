/**
 * XPT2046 Resistive Touch Controller – Implementation
 *
 * Protocol (SPI Mode 0):
 *   1. Assert CS low.
 *   2. Send 8-bit control byte (start bit, channel, mode, power-down).
 *   3. Clock out 16 bits: 12-bit ADC result in bits [14:3] of the 16-bit reply.
 *   4. Deassert CS.
 *
 * Averaging SAMPLES readings filters the ADC noise common on resistive panels.
 */

#include "xpt2046.h"
#include <stdlib.h>

static XPT2046_Config _cfg;

/* ── Default calibration constants for common 2.8" shields ─────────────── */
/*
 * These are typical values; your panel may differ by ±10 %.
 * Use xpt2046_raw() with xpt2046_set_cal() for precise results.
 */
#define DEFAULT_RAW_X_MIN  240
#define DEFAULT_RAW_X_MAX  3840
#define DEFAULT_RAW_Y_MIN  285
#define DEFAULT_RAW_Y_MAX  3965

void xpt2046_init(const XPT2046_Config *cfg)
{
    _cfg = *cfg;
}

void xpt2046_default_cal(uint16_t screen_w, uint16_t screen_h)
{
    _cfg.cal.raw_x_min = DEFAULT_RAW_X_MIN;
    _cfg.cal.raw_y_min = DEFAULT_RAW_Y_MIN;
    _cfg.cal.dx        = DEFAULT_RAW_X_MAX - DEFAULT_RAW_X_MIN;
    _cfg.cal.dy        = DEFAULT_RAW_Y_MAX - DEFAULT_RAW_Y_MIN;
    _cfg.cal.screen_w  = screen_w;
    _cfg.cal.screen_h  = screen_h;
    _cfg.cal.swap_xy   = false;
    _cfg.cal.invert_x  = false;
    _cfg.cal.invert_y  = true;
}

void xpt2046_set_cal(const XPT2046_Cal *cal)
{
    _cfg.cal = *cal;
}

/* ── SPI helpers ─────────────────────────────────────────────────────────── */

/* Read one 12-bit ADC channel.  Returns value in range 0–4095. */
static uint16_t _read_channel(uint8_t cmd)
{
    _cfg.set_cs(0);
    _cfg.spi_txrx(cmd);           /* Send control byte       */
    uint16_t hi = _cfg.spi_txrx(0x00);   /* MSB of 16-bit reply */
    uint16_t lo = _cfg.spi_txrx(0x00);   /* LSB of 16-bit reply */
    _cfg.set_cs(1);
    /* ADC result is in bits [14:3] of the 16-bit word */
    return ((hi << 8 | lo) >> 3) & 0x0FFF;
}

/* Median of SAMPLES – removes extreme outliers better than simple average */
static int _cmp_u16(const void *a, const void *b)
{
    return (int)(*(const uint16_t *)a) - (int)(*(const uint16_t *)b);
}

static uint16_t _sample_channel(uint8_t cmd)
{
    uint16_t buf[XPT2046_SAMPLES];
    for (int i = 0; i < XPT2046_SAMPLES; i++)
        buf[i] = _read_channel(cmd);
    qsort(buf, XPT2046_SAMPLES, sizeof(buf[0]), _cmp_u16);
    /* Return average of middle half */
    uint32_t sum = 0;
    int lo = XPT2046_SAMPLES / 4, hi = XPT2046_SAMPLES * 3 / 4;
    for (int i = lo; i < hi; i++) sum += buf[i];
    return (uint16_t)(sum / (hi - lo));
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void xpt2046_raw(uint16_t *rx, uint16_t *ry, uint16_t *rz)
{
    *rx = _sample_channel(XPT2046_CMD_X);
    *ry = _sample_channel(XPT2046_CMD_Y);

    /* Pressure = (Z1/Z2 – 1) × Rx/4096 – simplified to Z1+4096–Z2 */
    uint16_t z1 = _read_channel(XPT2046_CMD_Z1);
    uint16_t z2 = _read_channel(XPT2046_CMD_Z2);
    *rz = (z1 != 0) ? (uint16_t)(z1 + 4096 - z2) : 0;
}

bool xpt2046_read(TouchPoint *tp)
{
    tp->touched = false;

    /* Fast check via IRQ line if available */
    if (_cfg.read_irq && _cfg.read_irq() != 0)
        return false;

    uint16_t rx, ry, rz;
    xpt2046_raw(&rx, &ry, &rz);

    if (rz < XPT2046_PRESSURE_THRESH)
        return false;

    /* Map raw ADC to screen coordinates using calibration */
    XPT2046_Cal *c = &_cfg.cal;

    int32_t sx = (int32_t)(rx - c->raw_x_min) * c->screen_w / c->dx;
    int32_t sy = (int32_t)(ry - c->raw_y_min) * c->screen_h / c->dy;

    /* Clamp */
    if (sx < 0) sx = 0;
    if (sy < 0) sy = 0;
    if (sx >= c->screen_w)  sx = c->screen_w  - 1;
    if (sy >= c->screen_h)  sy = c->screen_h - 1;

    /* Invert axes if needed */
    if (c->invert_x) sx = c->screen_w  - 1 - sx;
    if (c->invert_y) sy = c->screen_h - 1 - sy;

    if (c->swap_xy) {
        tp->x = (uint16_t)sy;
        tp->y = (uint16_t)sx;
    } else {
        tp->x = (uint16_t)sx;
        tp->y = (uint16_t)sy;
    }
    tp->z       = rz;
    tp->touched = true;
    return true;
}
