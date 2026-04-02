/**
 * @file    as5600_position.c
 * @brief   Multi-turn absolute position tracker – AS5600 + STM32 Flash
 *          See as5600_position.h for full documentation.
 */
/** 
AS5600 Pinout SOIC-8 Pin-Out
─────────────────────────────────────────
        ┌───────────┐
 VDD5V ─┤ 1       8 ├── DIR
 VDD3V3─┤ 2       7 ├── SCL
  OUT  ─┤ 3       6 ├── SDA
  GND  ─┤ 4       5 ├── PGO
        └───────────┘

Pin Descriptions
─────────────────────────────────────────
1  VDD5V  5V supply    ─┐ tie together,
2  VDD3V3 3.3V supply  ─┘ connect to 3.3V
3  OUT    Analog/PWM output (unused if using I2C)
4  GND    Ground
5  PGO    OTP programming (leave unconnected)
6  SDA    I2C data (3k ohm pull-up)
7  SCL    I2C clock (3k ohm pull-up)
8  DIR    Direction select (GND = CW↑, 3.3V = CCW↑)
*/

#include "as5600_position.h"
#include <string.h>
#include <stdio.h>

/* ── AS5600 registers ───────────────────────────────────────────────────── */
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_ANGLE_H      0x0E
#define AS5600_REG_ANGLE_L      0x0F

#define AS5600_STATUS_MH        (1 << 3)   /* Field too strong  */
#define AS5600_STATUS_ML        (1 << 4)   /* Field too weak    */
#define AS5600_STATUS_MD        (1 << 5)   /* Magnet detected   */

/* ── Private state ──────────────────────────────────────────────────────── */
static AS5600_Position_t s_pos;
static uint16_t          s_prev_angle;
static bool              s_initialised = false;

/* ── Private: AS5600 helpers ────────────────────────────────────────────── */

static HAL_StatusTypeDef as5600_read_reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE,
                                   AS5600_I2C_ADDR, &reg, 1, 10);
    if (ret != HAL_OK) return ret;
    return HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE,
                                   AS5600_I2C_ADDR, buf, len, 10);
}

static HAL_StatusTypeDef as5600_read_angle(uint16_t *angle)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret = as5600_read_reg(AS5600_REG_ANGLE_H, buf, 2);
    if (ret != HAL_OK) return ret;
    *angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return HAL_OK;
}

static HAL_StatusTypeDef as5600_read_status(uint8_t *status)
{
    return as5600_read_reg(AS5600_REG_STATUS, status, 1);
}

/* ── Private: position helpers ──────────────────────────────────────────── */

static void update_absolute(void)
{
    s_pos.absolute_ticks = (int64_t)s_pos.turns * 4096
                         + (int64_t)s_pos.raw_angle;
}

static void detect_and_apply_rollover(uint16_t current_angle)
{
    int32_t delta = (int32_t)current_angle - (int32_t)s_prev_angle;

    if (delta < -ROLLOVER_THRESHOLD)
    {
        /* Large negative jump: crossed 4095 → 0 clockwise */
        s_pos.turns++;
    }
    else if (delta > ROLLOVER_THRESHOLD)
    {
        /* Large positive jump: crossed 0 → 4095 counter-clockwise */
        s_pos.turns--;
    }

    s_pos.raw_angle = current_angle;
    s_prev_angle    = current_angle;
    update_absolute();
}

/* ── Public API ─────────────────────────────────────────────────────────── */

HAL_StatusTypeDef AS5600_Init(void)
{
    /* 1. Check magnet */
    if (!AS5600_IsMagnetDetected())
    {
        memset(&s_pos, 0, sizeof(s_pos));
        s_prev_angle  = 0;
        s_initialised = true;
        return HAL_ERROR;
    }

    /* 2. Start from zero */
    s_pos.turns     = 0;
    s_pos.raw_angle = 0;

    /* 3. Read current sensor angle */
    uint16_t current_angle;
    HAL_StatusTypeDef ret = as5600_read_angle(&current_angle);
    if (ret != HAL_OK) return ret;

    s_pos.raw_angle = current_angle;
    s_prev_angle    = current_angle;
    update_absolute();
    s_initialised   = true;

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Update(void)
{
    if (!s_initialised) return HAL_ERROR;

    uint16_t current_angle;
    HAL_StatusTypeDef ret = as5600_read_angle(&current_angle);
    if (ret != HAL_OK) return ret;

    detect_and_apply_rollover(current_angle);
    return HAL_OK;
}

void AS5600_GetPosition(AS5600_Position_t *pos)
{
    if (pos) *pos = s_pos;
}

void AS5600_ResetPosition(void)
{
    s_pos.turns          = 0;
    s_pos.raw_angle      = 0;
    s_pos.absolute_ticks = 0;

    uint16_t current_angle;
    if (as5600_read_angle(&current_angle) == HAL_OK)
    {
        s_prev_angle    = current_angle;
        s_pos.raw_angle = current_angle;
        update_absolute();
    }
}

bool AS5600_IsMagnetDetected(void)
{
    uint8_t status = 0;
    if (as5600_read_status(&status) != HAL_OK) return false;
    return ((status & AS5600_STATUS_MD) != 0) &&
           ((status & AS5600_STATUS_MH) == 0) &&
           ((status & AS5600_STATUS_ML) == 0);
}

void AS5600_PrintMagnetStatus(void)
{
    uint8_t status = 0;
    if (as5600_read_status(&status) != HAL_OK)
    {
        printf("AS5600: I2C error\r\n");
        return;
    }

    uint8_t md = (status & AS5600_STATUS_MD) != 0;  /* magnet detected */
    uint8_t ml = (status & AS5600_STATUS_ML) != 0;  /* too weak        */
    uint8_t mh = (status & AS5600_STATUS_MH) != 0;  /* too strong      */

    if (!md)
        printf("AS5600: NO MAGNET DETECTED\r\n");
    else if (ml)
        printf("AS5600: TOO WEAK  - move magnet closer\r\n");
    else if (mh)
        printf("AS5600: TOO STRONG - move magnet further away\r\n");
    else
        printf("AS5600: GOOD\r\n");
}
