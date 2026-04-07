/**
 * example_main.c – Porting Guide & Demo
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * This file shows how to wire the three drivers to real hardware.
 *
 * HARDWARE CONNECTIONS (2.8" ILI9341 SPI shield, common pin-out)
 * ───────────────────────────────────────────────────────────────
 *  Shield Pin  │ Function           │ MCU (example)
 *  ────────────┼────────────────────┼────────────────
 *  VCC         │ 3.3 V              │ 3.3V rail
 *  GND         │ Ground             │ GND
 *  CS  (T_CS)  │ LCD chip-select    │ PA4  / D10
 *  RST         │ LCD hardware reset │ PA3  / D8
 *  DC          │ Data/Command       │ PA2  / D9
 *  MOSI (SDA)  │ SPI MOSI           │ PA7  / D11
 *  SCK  (SCL)  │ SPI clock          │ PA5  / D13
 *  LED         │ Backlight (PWM ok) │ 3.3V or PWM pin
 *  MISO (SDO)  │ SPI MISO           │ PA6  / D12
 *  T_CS        │ Touch chip-select  │ PB0  / D7
 *  T_IRQ       │ Touch IRQ          │ PB1  / D6  (optional)
 *  SD_CS       │ SD chip-select     │ PB2  / D5
 */

/* ═══════════════════════════════════════════════════════════════════════════
 * SECTION A – STM32 HAL EXAMPLE
 * ═══════════════════════════════════════════════════════════════════════════ */
#if defined(USE_STM32_HAL)

#include "stm32f4xx_hal.h"   /* Adjust for your MCU family */
#include "ili9341.h"
#include "xpt2046.h"
#include "sd_spi.h"

extern SPI_HandleTypeDef hspi1;

/* --- GPIO helpers -------------------------------------------------------- */
static void lcd_set_cs(int v)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void lcd_set_dc(int v)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void lcd_set_rst(int v) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void tch_set_cs(int v)  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static void sd_set_cs(int v)   { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  v ? GPIO_PIN_SET : GPIO_PIN_RESET); }
static int  tch_irq(void)      { return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1); }

/* Shared SPI transceive (all three devices use the same bus) */
static uint8_t spi_txrx(uint8_t byte)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &byte, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void spi_set_slow(void) { /* Set prescaler for ~400 kHz */ }
static void spi_set_fast(void) { /* Set prescaler for ~25 MHz  */ }

/* --- Initialise all subsystems ------------------------------------------ */
void board_init(void)
{
    /* LCD */
    ILI9341_Config lcd_cfg = {
        .spi_txrx = spi_txrx,
        .set_cs   = lcd_set_cs,
        .set_dc   = lcd_set_dc,
        .set_rst  = lcd_set_rst,
        .delay_ms = HAL_Delay,
        .rotation = ROTATION_0,
    };
    ili9341_init(&lcd_cfg);

    /* Touch */
    XPT2046_Config tch_cfg = {
        .spi_txrx = spi_txrx,
        .set_cs   = tch_set_cs,
        .read_irq = tch_irq,
    };
    xpt2046_init(&tch_cfg);
    xpt2046_default_cal(ILI9341_WIDTH, ILI9341_HEIGHT);

    /* SD card */
    SD_Config sd_cfg = {
        .spi_txrx    = spi_txrx,
        .spi_set_slow = spi_set_slow,
        .spi_set_fast = spi_set_fast,
        .set_cs      = sd_set_cs,
        .delay_ms    = HAL_Delay,
    };
    SD_Info info;
    if (sd_init(&sd_cfg, &info) == SD_OK) {
        /* SD ready – info.capacity_mb holds size in megabytes */
    }
}

#endif /* USE_STM32_HAL */


