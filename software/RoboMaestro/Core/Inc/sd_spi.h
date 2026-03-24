#ifndef SD_SPI_H
#define SD_SPI_H

/**
 * SD Card SPI Driver (SD, SDHC, SDXC)
 *
 * Implements the SD SPI mode protocol using only the 6 mandatory commands
 * needed for block read/write:
 *
 *   CMD0  – GO_IDLE_STATE       (software reset to SPI mode)
 *   CMD1  – SEND_OP_COND        (init for SD v1)
 *   CMD8  – SEND_IF_COND        (version detection, SD v2)
 *   CMD9  – SEND_CSD            (read card-specific data)
 *   CMD16 – SET_BLOCKLEN        (force 512-byte blocks on SDSC)
 *   CMD17 – READ_SINGLE_BLOCK
 *   CMD24 – WRITE_BLOCK
 *   CMD55 – APP_CMD             (prefix for ACMD)
 *   CMD58 – READ_OCR
 *   ACMD41– SD_SEND_OP_COND    (init for SD v2 / SDHC)
 *
 * Wiring:
 *   SD_CS   -> dedicated chip select for SD
 *   SD_MOSI -> shared SPI MOSI
 *   SD_MISO -> shared SPI MISO
 *   SD_SCK  -> shared SPI clock
 *
 * The SD card requires:
 *   – Slow init clock (~400 kHz) then faster operational clock (up to 25 MHz).
 *   – 3.3 V logic.  Do NOT connect 5V Arduino signals directly.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Block size is always 512 bytes in SPI mode */
#define SD_BLOCK_SIZE  512

/* Error codes */
typedef enum {
    SD_OK            = 0,
    SD_ERR_TIMEOUT   = 1,
    SD_ERR_INIT      = 2,
    SD_ERR_CMD       = 3,
    SD_ERR_CRC       = 4,
    SD_ERR_PARAM     = 5,
    SD_ERR_WRITE     = 6,
    SD_ERR_READ      = 7,
} SD_Error;

/* Card type */
typedef enum {
    SD_TYPE_UNKNOWN = 0,
    SD_TYPE_SD1     = 1,  /* Standard capacity v1 */
    SD_TYPE_SD2     = 2,  /* Standard capacity v2 */
    SD_TYPE_SDHC    = 3,  /* High / Extended capacity */
} SD_Type;

/* Driver config */
typedef struct {
    /* SPI – can run at different speeds for init vs operation */
    uint8_t  (*spi_txrx)(uint8_t byte);
    void     (*spi_set_slow)(void);      /* ~400 kHz for init */
    void     (*spi_set_fast)(void);      /* up to 25 MHz for data */
    void     (*set_cs)(int level);       /* SD chip select */
    void     (*delay_ms)(uint32_t ms);
    uint32_t (*get_ms)(void);            /* monotonic millisecond counter (e.g. HAL_GetTick) */
    void     (*log)(const char *msg);    /* optional debug log (NULL = disabled) */
} SD_Config;

/* Card information */
typedef struct {
    SD_Type   type;
    uint32_t  block_count;   /* Total blocks (block_count × 512 = capacity) */
    uint64_t  capacity_mb;
} SD_Info;

/* ── Public API ──────────────────────────────────────────────────────────── */

SD_Error sd_init(const SD_Config *cfg, SD_Info *info_out);

/* Read/write single 512-byte block.  'block_addr' is the block number. */
SD_Error sd_read_block(uint32_t block_addr, uint8_t *buf);
SD_Error sd_write_block(uint32_t block_addr, const uint8_t *buf);

/* Multi-block read/write (faster; avoids per-block CMD overhead) */
SD_Error sd_read_blocks(uint32_t block_addr, uint8_t *buf, uint32_t count);
SD_Error sd_write_blocks(uint32_t block_addr, const uint8_t *buf, uint32_t count);

/* Return last cached card info */
const SD_Info *sd_get_info(void);

/* Human-readable error string */
const char *sd_error_str(SD_Error err);

#endif /* SD_SPI_H */
