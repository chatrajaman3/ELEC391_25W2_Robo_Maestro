#ifndef XPT2046_H
#define XPT2046_H

/**
 * XPT2046 Resistive Touch Controller Driver
 *
 * The 2.8" ILI9341 shield uses the XPT2046 (or ADS7843-compatible) touch IC.
 * It shares the SPI bus with the LCD; select it via TOUCH_CS.
 *
 * SPI mode: CPOL=0, CPHA=0 (Mode 0), max 2 MHz recommended for touch.
 *
 * Calibration:
 *   Raw ADC values must be mapped to screen pixels using a 2-point calibration.
 *   Call xpt2046_calibrate() with known screen points and their raw ADC values,
 *   or hard-code calibration constants for your specific panel.
 */

#include <stdint.h>
#include <stdbool.h>

/* ── XPT2046 control byte commands ──────────────────────────────────────── */
#define XPT2046_CMD_X    0xD0  /* Measure X  (12-bit, differential, PD=00) */
#define XPT2046_CMD_Y    0x90  /* Measure Y  (12-bit, differential, PD=00) */
#define XPT2046_CMD_Z1   0xB0  /* Measure Z1 (pressure) */
#define XPT2046_CMD_Z2   0xC0  /* Measure Z2 (pressure) */

/* Pressure threshold – lower value = lighter touch detected */
#define XPT2046_PRESSURE_THRESH  400

/* Number of samples to average for noise reduction */
#define XPT2046_SAMPLES  8

/* ── Calibration data ────────────────────────────────────────────────────── */
typedef struct {
    int32_t  dx;     /* (raw_x_max - raw_x_min) */
    int32_t  dy;
    int32_t  raw_x_min;
    int32_t  raw_y_min;
    uint16_t screen_w;
    uint16_t screen_h;
    bool     swap_xy;   /* Set true when screen is in landscape mode */
    bool     invert_x;  /* Mirror X axis after mapping */
    bool     invert_y;  /* Mirror Y axis after mapping */
} XPT2046_Cal;

/* ── Driver configuration ────────────────────────────────────────────────── */
typedef struct {
    uint8_t  (*spi_txrx)(uint8_t byte);  /* Same SPI bus as LCD */
    void     (*set_cs)(int level);       /* TOUCH_CS pin */
    /* Optional: read the T_IRQ line (active low when touched).
       Set to NULL to skip IRQ check and poll pressure instead. */
    int      (*read_irq)(void);

    XPT2046_Cal cal;
} XPT2046_Config;

/* ── Touch point ─────────────────────────────────────────────────────────── */
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;      /* Pressure (higher = harder press) */
    bool     touched;
} TouchPoint;

/* ── Public API ──────────────────────────────────────────────────────────── */

void       xpt2046_init(const XPT2046_Config *cfg);

/* Returns true and fills *tp if a valid touch is detected */
bool       xpt2046_read(TouchPoint *tp);

/* Read raw ADC values (for calibration) */
void       xpt2046_raw(uint16_t *rx, uint16_t *ry, uint16_t *rz);

/* Set calibration after measurement */
void       xpt2046_set_cal(const XPT2046_Cal *cal);

/* Default calibration for common 2.8" ILI9341 shields (portrait) */
void       xpt2046_default_cal(uint16_t screen_w, uint16_t screen_h);

#endif /* XPT2046_H */
