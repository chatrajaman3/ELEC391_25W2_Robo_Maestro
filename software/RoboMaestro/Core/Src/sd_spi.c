/**
 * SD Card SPI Driver – Implementation
 *
 * Protocol reference: "SD Specifications Part 1: Physical Layer Specification"
 * (Simplified version freely available from the SD Association).
 *
 * SPI command frame: 6 bytes
 *   [0] 0x40 | cmd_index
 *   [1..4]   argument (big-endian)
 *   [5]      CRC7 << 1 | 0x01  (only CMD0 and CMD8 need valid CRC in SPI mode)
 *
 * R1 response: 1 byte, 0x00 = OK, bit set = error/status.
 * Data token for read  : 0xFE
 * Data token for write : 0xFE
 * Data response token  : 0bxxx00101 = accepted
 */

#include "sd_spi.h"
#include <string.h>
#include <stdio.h>

/* ── Internal state ──────────────────────────────────────────────────────── */
static SD_Config _cfg;
static SD_Info   _info;
static bool      _initialized = false;

/* ── Timeouts ────────────────────────────────────────────────────────────── */
#define SD_TIMEOUT_INIT_MS  30000  /* Allow up to 30 s for ACMD41 to complete */
#define SD_TIMEOUT_CMD      200
#define SD_TIMEOUT_DATA     200000  /* ~570 ms at 2.8 MHz; SD spec allows 250 ms+ */

/* ── SPI low-level ───────────────────────────────────────────────────────── */

static inline uint8_t _spi(uint8_t b)  { return _cfg.spi_txrx(b); }
static inline void    _cs_low(void)    { _cfg.set_cs(0); }
static inline void    _cs_high(void)   { _cfg.set_cs(1); _spi(0xFF); }

/* Send n clock pulses with MOSI high (required by SD spec §6.4.1) */
static void _spi_clock_cycles(int n)
{
    for (int i = 0; i < n; i++) _spi(0xFF);
}

/* ── Command engine ──────────────────────────────────────────────────────── */

/*
 * Send an SD command and return the R1 response byte.
 * CS must be asserted (low) by the caller before calling.
 */
static uint8_t _send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t frame[6] = {
        0x40 | cmd,
        (uint8_t)(arg >> 24),
        (uint8_t)(arg >> 16),
        (uint8_t)(arg >> 8),
        (uint8_t)(arg),
        0x01  /* stop bit; dummy CRC for non-CRC commands */
    };

    /* CMD0 and CMD8 require valid CRC in SPI mode */
    if (cmd == 0)  frame[5] = 0x95;
    if (cmd == 8)  frame[5] = 0x87;

    /* One dummy byte to allow card to finish any prior operation */
    _spi(0xFF);

    for (int i = 0; i < 6; i++) _spi(frame[i]);

    /* Wait for R1 response — spec allows up to 8 bytes; use 64 to handle
     * slow SDXC cards and SanDisk cards that respond late.               */
    uint8_t r = 0xFF;
    for (int i = 0; i < 64; i++) {
        r = _spi(0xFF);
        if (!(r & 0x80)) break;  /* MSB=0 means valid R1 */
    }
    return r;
}

/* Send ACMD (CMD55 prefix + the application command) */
static uint8_t _send_acmd(uint8_t acmd, uint32_t arg)
{
    /* CMD55 must be sent with CS low; re-assert CS for the actual ACMD */
    uint8_t r55 = _send_cmd(55, 0);
    /* If CMD55 returns something other than 0x00 or 0x01 the card is unhappy */
    if (r55 > 0x01) return r55;
    return _send_cmd(acmd, arg);
}

/* Wait for card to release the bus (MISO=0xFF means card is idle/ready) */
static bool _wait_ready(uint32_t timeout)
{
    uint8_t r;
    while (timeout--) {
        r = _spi(0xFF);
        if (r == 0xFF) return true;
    }
    return false;
}

/* ── Data block I/O ──────────────────────────────────────────────────────── */

/* Wait for the 0xFE data token that precedes a data block */
static bool _wait_data_token(void)
{
    for (uint32_t i = 0; i < SD_TIMEOUT_DATA; i++) {
        uint8_t b = _spi(0xFF);
        if (b == 0xFE) return true;
        if (b != 0xFF) return false;  /* Any other non-FF byte is an error token */
    }
    return false;
}

/* Read one 512-byte block (CS already low, CMD17/18 already sent and ACKed) */
static SD_Error _read_data(uint8_t *buf)
{
    if (!_wait_data_token()) return SD_ERR_TIMEOUT;

    for (int i = 0; i < SD_BLOCK_SIZE; i++)
        buf[i] = _spi(0xFF);

    _spi(0xFF);  /* CRC byte 1 (ignored in SPI mode) */
    _spi(0xFF);  /* CRC byte 2 */
    return SD_OK;
}

/* Write one 512-byte block (CS already low) */
static SD_Error _write_data(const uint8_t *buf, uint8_t token)
{
    if (!_wait_ready(SD_TIMEOUT_DATA)) return SD_ERR_TIMEOUT;

    _spi(token);  /* Data token: 0xFE for single block, 0xFC for multi-block */
    for (int i = 0; i < SD_BLOCK_SIZE; i++) _spi(buf[i]);
    _spi(0xFF);   /* Dummy CRC high */
    _spi(0xFF);   /* Dummy CRC low  */

    /* Data response token: lower 5 bits = 0b00101 (0x05) means accepted */
    uint8_t resp = 0xFF;
    for (int i = 0; i < 8; i++) {
        resp = _spi(0xFF);
        if (resp != 0xFF) break;
    }
    if ((resp & 0x1F) != 0x05) return SD_ERR_WRITE;

    /* Wait for card to finish internal programming (MISO held low while busy) */
    if (!_wait_ready(SD_TIMEOUT_DATA)) return SD_ERR_TIMEOUT;
    return SD_OK;
}

/* ── CSD parsing: extract total block count ──────────────────────────────── */
static uint32_t _parse_csd_block_count(const uint8_t *csd)
{
    uint8_t csd_structure = (csd[0] >> 6) & 0x03;
    if (csd_structure == 1) {
        /* CSD v2 (SDHC/SDXC): fixed 512-byte blocks, C_SIZE in bytes 7–9 */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16)
                        | ((uint32_t)csd[8] << 8)
                        |  (uint32_t)csd[9];
        return (c_size + 1) * 1024;
    } else {
        /* CSD v1 (SDSC) */
        uint32_t c_size      = ((uint32_t)(csd[6] & 0x03) << 10)
                             | ((uint32_t)csd[7] << 2)
                             | ((uint32_t)(csd[8] >> 6) & 0x03);
        uint32_t c_size_mult = ((uint32_t)(csd[9]  & 0x03) << 1)
                             | ((uint32_t)(csd[10] >> 7) & 0x01);
        uint32_t read_bl_len = csd[5] & 0x0F;
        uint32_t block_len   = 1u << read_bl_len;
        uint32_t mult        = 1u << (c_size_mult + 2);
        uint32_t capacity    = (c_size + 1) * mult * block_len;
        return capacity / SD_BLOCK_SIZE;
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

SD_Error sd_init(const SD_Config *cfg, SD_Info *info_out)
{
    _cfg         = *cfg;
    _initialized = false;
    memset(&_info, 0, sizeof(_info));

    /* ── Phase 1: Enter SPI mode ──────────────────────────────────────────
     * SD spec §6.4.1: card needs ≥74 clock cycles with CS=HIGH before CMD0.
     * We use 80 bytes (640 clocks) and a generous power-on delay.          */
    _cfg.spi_set_slow();
    _cs_high();                 /* ensure CS is deasserted (high)           */
    _cfg.delay_ms(500);         /* allow card power rail to stabilise       */
    _spi_clock_cycles(160);     /* 1280 clocks; some SanDisk cards need more */

    /* ── CMD0: Software reset — retry until card returns 0x01 (in-idle) ──
     * 100 attempts × 10 ms = 1 second maximum.                             */
    uint8_t r = 0xFF;
    for (int attempt = 0; attempt < 100; attempt++) {
        _cs_low();
        r = _send_cmd(0, 0);
        _cs_high();
        if (r == 0x01) break;
        _cfg.delay_ms(10);
    }
    if (r != 0x01) return SD_ERR_INIT;

    /* ── CMD8: Detect SD v2 (SDHC/SDXC capable) ─────────────────────────
     * Argument: VHS=0x01 (2.7–3.6 V) | check pattern 0xAA.
     * V2 cards echo back the check pattern; V1/MMC cards return 0x05.      */
    _cs_low();
    r = _send_cmd(8, 0x000001AA);
    uint8_t ocr[4];
    for (int i = 0; i < 4; i++) ocr[i] = _spi(0xFF);
    _cs_high();
    _cfg.delay_ms(100);  /* SanDisk 2GB SDSC needs ~75ms after CMD8 */

    bool is_v2 = (r == 0x01 && ocr[3] == 0xAA);

    /* ── Phase 2: Initialisation loop ────────────────────────────────────
     * Always start with HCS=0 (no host high-capacity support bit).
     * This is required for SanDisk 2 GB SDSC v2.0 cards: they accept CMD8
     * but reject ACMD41 with HCS=1 as an illegal command, causing the card
     * to stop responding.  HCS=0 works for ALL card types — SDHC/SDXC
     * cards also accept it; after ACMD41 completes we use CMD58 to detect
     * SDHC via the CCS bit.
     *
     * CMD55 and ACMD41 are sent in a single CS-low window; releasing CS
     * between them clears the APP_CMD flag on this card.                    */
    /* Try HCS=1 first (needed for SDHC, and some SDSC v2 cards also require
     * it to complete ACMD41).  With HCS=0 this card returns 0x01 forever. */
    uint32_t acmd41_arg = is_v2 ? 0x40000000 : 0x00000000;

    uint32_t t0 = _cfg.get_ms();
    int log_count = 0;
    do {
        /* Assert CS LOW and keep it low through CMD55 + ACMD41 in one
         * uninterrupted transaction.  Releasing CS between the two commands
         * clears the APP_CMD flag on this card, causing ACMD41 to be treated
         * as illegal CMD41 (R1 = 0x05).                                   */
        _cs_low();
        uint8_t r55 = _send_cmd(55, 0);
        /* CS is still LOW here — do NOT call _cs_high() yet              */

        if (r55 == 0xFF) {
            /* Card stopped responding.  Wait 500 ms to see if it recovers
             * on its own — it may be temporarily busy during internal init.
             * Only send CMD0 if it still won't respond after waiting.     */
            _cs_high();
            _cfg.delay_ms(500);

            _cs_low();
            uint8_t r55_check = _send_cmd(55, 0);
            if (r55_check <= 0x01) {
                /* Card recovered — immediately try ACMD41 */
                _cs_high();
                _cs_low();
                r = _send_cmd(41, acmd41_arg);
                _cs_high();
                _cfg.delay_ms(10);
            } else {
                /* Still silent — re-enter SPI mode with CMD0 */
                _cs_high();
                _spi_clock_cycles(16);
                _cs_low();
                _send_cmd(0, 0);
                _cs_high();
                _cfg.delay_ms(100);
                r = 0xFF;
            }
        } else if (r55 <= 0x01) {
            /* Release CS between CMD55 and ACMD41 (separate transactions) */
            _cs_high();
            _cs_low();
            r = _send_cmd(41, acmd41_arg);
            _cs_high();
            /* Give the card 2 s between ACMD41 retries — it goes silent if
             * retried too quickly (likely needs time for internal NAND init) */
            if (r == 0x01) _cfg.delay_ms(2000);
        } else {
            _cs_high();
            r = r55;
            _cfg.delay_ms(10);
        }

        /* Log first 12 attempts so we can see what the card returns */
        if (_cfg.log && log_count < 12) {
            char buf[40];
            snprintf(buf, sizeof(buf), "55=%02X A41=%02X t=%lu\r\n",
                     r55, r, (unsigned long)(_cfg.get_ms() - t0));
            _cfg.log(buf);
            log_count++;
        }
    } while (r != 0x00 && (_cfg.get_ms() - t0) < SD_TIMEOUT_INIT_MS);

    /* ── Fallback: CMD1 for MMC / very old SD cards that reject ACMD41 ── */
    if (r != 0x00) {
        if (_cfg.log) _cfg.log("ACMD41 timed out, trying CMD1...\r\n");
        t0 = _cfg.get_ms();
        int cmd1_count = 0;
        do {
            _cs_low();
            r = _send_cmd(1, 0);
            _cs_high();
            if (_cfg.log && cmd1_count < 4) {
                char buf[24];
                snprintf(buf, sizeof(buf), "CMD1=%02X\r\n", r);
                _cfg.log(buf);
                cmd1_count++;
            }
            if (r == 0x01) _cfg.delay_ms(10);
        } while (r != 0x00 && (_cfg.get_ms() - t0) < SD_TIMEOUT_INIT_MS);
    }

    if (r != 0x00) return SD_ERR_TIMEOUT;

    /* ── CMD58: Read OCR register — confirm SDHC bit (CCS, bit 30) ──────
     * Only valid after ACMD41 returns 0x00 (power-up complete).            */
    _cs_low();
    r = _send_cmd(58, 0);
    for (int i = 0; i < 4; i++) ocr[i] = _spi(0xFF);
    _cs_high();

    if (r == 0x00 && (ocr[0] & 0x40))
        _info.type = SD_TYPE_SDHC;   /* CCS=1: SDHC or SDXC                */
    else if (is_v2)
        _info.type = SD_TYPE_SD2;    /* V2 SDSC (CCS=0)                    */
    else
        _info.type = SD_TYPE_SD1;    /* V1 SDSC                            */

    /* ── CMD16: Set block length to 512 bytes (SDSC only) ───────────────
     * SDHC/SDXC always use 512-byte blocks; CMD16 is not needed for them. */
    if (_info.type != SD_TYPE_SDHC) {
        _cs_low();
        r = _send_cmd(16, SD_BLOCK_SIZE);
        _cs_high();
        if (r != 0x00) return SD_ERR_CMD;
    }

    /* ── CMD9: Read CSD register to determine card capacity ─────────────  */
    _cs_low();
    r = _send_cmd(9, 0);
    uint8_t csd[16] = {0};
    if (r == 0x00 && _wait_data_token()) {
        for (int i = 0; i < 16; i++) csd[i] = _spi(0xFF);
        _spi(0xFF); _spi(0xFF);  /* CRC (ignored) */
    }
    _cs_high();

    _info.block_count = _parse_csd_block_count(csd);
    _info.capacity_mb = ((uint64_t)_info.block_count * SD_BLOCK_SIZE) >> 20;

    /* ── Switch to high-speed clock for data transfers ─────────────────── */
    _cfg.spi_set_fast();
    _initialized = true;

    if (info_out) *info_out = _info;
    return SD_OK;
}

/* ── Block read ──────────────────────────────────────────────────────────── */

SD_Error sd_read_block(uint32_t block_addr, uint8_t *buf)
{
    if (!_initialized) return SD_ERR_INIT;
    if (!buf)          return SD_ERR_PARAM;

    /* SDSC: byte addressing.  SDHC/SDXC: block addressing. */
    uint32_t addr = (_info.type == SD_TYPE_SDHC) ? block_addr
                                                  : block_addr * SD_BLOCK_SIZE;
    _cs_low();
    uint8_t r    = _send_cmd(17, addr);
    SD_Error err = (r == 0x00) ? _read_data(buf) : SD_ERR_CMD;
    _cs_high();
    return err;
}

SD_Error sd_read_blocks(uint32_t block_addr, uint8_t *buf, uint32_t count)
{
    if (!_initialized || !buf || count == 0) return SD_ERR_PARAM;

    uint32_t addr = (_info.type == SD_TYPE_SDHC) ? block_addr
                                                  : block_addr * SD_BLOCK_SIZE;
    _cs_low();
    SD_Error err = SD_OK;
    uint8_t r = _send_cmd(18, addr);  /* CMD18: READ_MULTIPLE_BLOCK */
    if (r != 0x00) { _cs_high(); return SD_ERR_CMD; }

    for (uint32_t i = 0; i < count && err == SD_OK; i++)
        err = _read_data(buf + i * SD_BLOCK_SIZE);

    /* CMD12: STOP_TRANSMISSION */
    _send_cmd(12, 0);
    _cs_high();
    return err;
}

/* ── Block write ─────────────────────────────────────────────────────────── */

SD_Error sd_write_block(uint32_t block_addr, const uint8_t *buf)
{
    if (!_initialized) return SD_ERR_INIT;
    if (!buf)          return SD_ERR_PARAM;

    uint32_t addr = (_info.type == SD_TYPE_SDHC) ? block_addr
                                                  : block_addr * SD_BLOCK_SIZE;
    _cs_low();
    uint8_t r    = _send_cmd(24, addr);
    SD_Error err = (r == 0x00) ? _write_data(buf, 0xFE) : SD_ERR_CMD;
    _cs_high();
    return err;
}

SD_Error sd_write_blocks(uint32_t block_addr, const uint8_t *buf, uint32_t count)
{
    if (!_initialized || !buf || count == 0) return SD_ERR_PARAM;

    uint32_t addr = (_info.type == SD_TYPE_SDHC) ? block_addr
                                                  : block_addr * SD_BLOCK_SIZE;
    _cs_low();

    /* ACMD23: pre-erase hint — speeds up multi-block write on most cards */
    _send_acmd(23, count);

    uint8_t r = _send_cmd(25, addr);  /* CMD25: WRITE_MULTIPLE_BLOCK */
    if (r != 0x00) { _cs_high(); return SD_ERR_CMD; }

    SD_Error err = SD_OK;
    for (uint32_t i = 0; i < count && err == SD_OK; i++)
        err = _write_data(buf + i * SD_BLOCK_SIZE, 0xFC);  /* 0xFC = multi-write token */

    /* Stop token + wait for card to finish last block */
    _wait_ready(SD_TIMEOUT_DATA);
    _spi(0xFD);
    _wait_ready(SD_TIMEOUT_DATA);
    _cs_high();
    return err;
}

/* ── Accessors ───────────────────────────────────────────────────────────── */

const SD_Info *sd_get_info(void) { return &_info; }

const char *sd_error_str(SD_Error err)
{
    switch (err) {
        case SD_OK:          return "OK";
        case SD_ERR_TIMEOUT: return "Timeout";
        case SD_ERR_INIT:    return "Init failed";
        case SD_ERR_CMD:     return "Command error";
        case SD_ERR_CRC:     return "CRC error";
        case SD_ERR_PARAM:   return "Bad parameter";
        case SD_ERR_WRITE:   return "Write failed";
        case SD_ERR_READ:    return "Read failed";
        default:             return "Unknown";
    }
}
