/**
 * @file    as5600_position.c
 * @brief   Multi-turn absolute position tracker using AS5600 magnetic encoder.
 *
 * Tracks full rotations by detecting rollovers in the 12-bit angle reading.
 * Absolute position = turns * 4096 + raw_angle (in ticks).
 */

#include "as5600_position.h"

/* AS5600 register addresses */
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_ANGLE_H  0x0E
#define AS5600_REG_ANGLE_L  0x0F

/* STATUS register bits */
#define AS5600_STATUS_MD    (1 << 5)  /* Magnet Detected  */
#define AS5600_STATUS_ML    (1 << 4)  /* Magnet too Low   */
#define AS5600_STATUS_MH    (1 << 3)  /* Magnet too High  */

/* ── Private state ────────────────────────────────────────────────────────── */

static AS5600_Position_t g_pos = {0, 0, 0};
static uint16_t          g_prev_raw = 0;
static uint8_t           g_initialised = 0;

/* ── Private helpers ──────────────────────────────────────────────────────── */

static HAL_StatusTypeDef ReadRawAngle(uint16_t *angle)
{
    uint8_t buf[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &AS5600_I2C_HANDLE,
        AS5600_I2C_ADDR,
        AS5600_REG_ANGLE_H,
        I2C_MEMADD_SIZE_8BIT,
        buf, 2,
        HAL_MAX_DELAY);

    if (status == HAL_OK)
        *angle = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];

    return status;
}

static HAL_StatusTypeDef ReadStatus(uint8_t *status_reg)
{
    return HAL_I2C_Mem_Read(
        &AS5600_I2C_HANDLE,
        AS5600_I2C_ADDR,
        AS5600_REG_STATUS,
        I2C_MEMADD_SIZE_8BIT,
        status_reg, 1,
        HAL_MAX_DELAY);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

HAL_StatusTypeDef AS5600_Init(void)
{
    uint16_t raw;
    HAL_StatusTypeDef status = ReadRawAngle(&raw);
    if (status != HAL_OK)
        return HAL_ERROR;

    g_prev_raw          = raw;
    g_pos.raw_angle     = raw;
    g_pos.turns         = 0;
    g_pos.absolute_ticks = (int64_t)raw;
    g_initialised       = 1;

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Update(void)
{
    if (!g_initialised)
        return HAL_ERROR;

    uint16_t raw;
    HAL_StatusTypeDef status = ReadRawAngle(&raw);
    if (status != HAL_OK)
        return HAL_ERROR;

    int32_t delta = (int32_t)raw - (int32_t)g_prev_raw;

    /* Rollover detection: wrap delta into [-2048, +2047] */
    if (delta >  (int32_t)ROLLOVER_THRESHOLD) delta -= 4096;
    if (delta < -(int32_t)ROLLOVER_THRESHOLD) delta += 4096;

    g_pos.absolute_ticks += delta;
    g_pos.turns           = (int32_t)(g_pos.absolute_ticks / 4096);
    g_pos.raw_angle       = raw;
    g_prev_raw            = raw;

    return HAL_OK;
}

void AS5600_GetPosition(AS5600_Position_t *pos)
{
    *pos = g_pos;
}

void AS5600_ResetPosition(void)
{
    g_pos.turns          = 0;
    g_pos.absolute_ticks = (int64_t)g_pos.raw_angle;
}

bool AS5600_IsMagnetDetected(void)
{
    uint8_t status_reg = 0;
    if (ReadStatus(&status_reg) != HAL_OK)
        return false;
    return (status_reg & AS5600_STATUS_MD) != 0;
}

