/**
 * @file    as5600_position.c
 * @brief   Multi-turn absolute position tracker – AS5600 + STM32 Flash
 *          See as5600_position.h for full documentation.
 */

#include "as5600_position.h"
#include <string.h>

/* ── AS5600 registers ───────────────────────────────────────────────────── */
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_ANGLE_H      0x0E
#define AS5600_REG_ANGLE_L      0x0F

#define AS5600_STATUS_MH        (1 << 3)   /* Field too strong  */
#define AS5600_STATUS_ML        (1 << 4)   /* Field too weak    */
#define AS5600_STATUS_MD        (1 << 5)   /* Magnet detected   */

/* ── Flash record layout ────────────────────────────────────────────────── */
/*
 * We store 4 words (16 bytes) at FLASH_SAVE_ADDR:
 *
 *  Offset  Size  Content
 *  0x00    4 B   FLASH_MAGIC  (0xA55A1234)
 *  0x04    4 B   turns        (int32_t, little-endian via word write)
 *  0x08    4 B   raw_angle    (uint16_t padded to uint32_t)
 *  0x0C    4 B   ~FLASH_MAGIC (bitwise NOT, used as integrity check)
 *
 * The NOT-magic at the end lets us catch a torn write: if power dies
 * after word 0 but before word 3, the record is invalid.
 */
#define FLASH_RECORD_WORDS      4          /* 4 × uint32_t = 16 bytes       */

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

/* ── Private: Flash helpers ─────────────────────────────────────────────── */

/**
 * Read the saved record from Flash.
 * Returns true if the record is valid (magic + NOT-magic intact).
 */
static bool flash_load(int32_t *turns, uint16_t *raw_angle)
{
    uint32_t *base = (uint32_t *)FLASH_SAVE_ADDR;

    uint32_t magic     = base[0];
    uint32_t t_raw     = base[1];
    uint32_t a_raw     = base[2];
    uint32_t not_magic = base[3];

    /* Integrity check: magic must be correct and complemented by word 3 */
    if (magic != FLASH_MAGIC) return false;
    if (not_magic != ~FLASH_MAGIC) return false;

    *turns     = (int32_t)t_raw;
    *raw_angle = (uint16_t)(a_raw & 0xFFFF);
    return true;
}

/**
 * Erase sector then write the 4-word record.
 * Flash must be erased to 0xFF before programming.
 * Note: this blocks for the duration of the erase (~1–2 s for 128 KB).
 * If that is unacceptable, run it from a low-priority task or RTOS thread.
 */
static HAL_StatusTypeDef flash_save(int32_t turns, uint16_t raw_angle)
{
    HAL_StatusTypeDef ret;

    ret = HAL_FLASH_Unlock();
    if (ret != HAL_OK) return ret;

    /* Erase the sector first */
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_SAVE_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3   /* 2.7V–3.6V supply */
    };
    uint32_t sector_error = 0;
    ret = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (ret != HAL_OK) { HAL_FLASH_Lock(); return ret; }

    /* Write 4 words */
    uint32_t words[FLASH_RECORD_WORDS] = {
        FLASH_MAGIC,
        (uint32_t)turns,
        (uint32_t)raw_angle,
        ~FLASH_MAGIC
    };

    for (int i = 0; i < FLASH_RECORD_WORDS; i++)
    {
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                 FLASH_SAVE_ADDR + (i * 4),
                                 words[i]);
        if (ret != HAL_OK) { HAL_FLASH_Lock(); return ret; }
    }

    HAL_FLASH_Lock();
    return HAL_OK;
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

    /* 2. Try to restore from Flash */
    int32_t  saved_turns = 0;
    uint16_t saved_angle = 0;
    bool     restored    = flash_load(&saved_turns, &saved_angle);

    if (restored)
    {
        s_pos.turns     = saved_turns;
        s_pos.raw_angle = saved_angle;
    }
    else
    {
        /* First boot – Flash blank or corrupt, start from zero */
        s_pos.turns     = 0;
        s_pos.raw_angle = 0;
    }

    /* 3. Read current sensor angle */
    uint16_t current_angle;
    HAL_StatusTypeDef ret = as5600_read_angle(&current_angle);
    if (ret != HAL_OK) return ret;

    /*
     * 4. Reconcile restored angle with current sensor reading.
     *
     * If the robot did not move while unpowered, angles should match.
     * If they differ by more than ROLLOVER_THRESHOLD, exactly one
     * rollover occurred while the MCU was off — correct for it.
     * (If the robot moved many full turns while off, this cannot be
     *  detected without power; in that case use the homing approach.)
     */
    int32_t delta = (int32_t)current_angle - (int32_t)s_pos.raw_angle;
    if (delta < -ROLLOVER_THRESHOLD) s_pos.turns++;
    else if (delta > ROLLOVER_THRESHOLD) s_pos.turns--;

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

HAL_StatusTypeDef AS5600_SavePosition(void)
{
    if (!s_initialised) return HAL_ERROR;
    return flash_save(s_pos.turns, s_pos.raw_angle);
}

void AS5600_GetPosition(AS5600_Position_t *pos)
{
    if (pos) *pos = s_pos;
}

HAL_StatusTypeDef AS5600_ResetPosition(void)
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

    return AS5600_SavePosition();
}

bool AS5600_IsMagnetDetected(void)
{
    uint8_t status = 0;
    if (as5600_read_status(&status) != HAL_OK) return false;
    return ((status & AS5600_STATUS_MD) != 0) &&
           ((status & AS5600_STATUS_MH) == 0) &&
           ((status & AS5600_STATUS_ML) == 0);
}
