/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
   * ╔══════════════════════════════════════════════════════════════════════╗
   * ║       2.8" ILI9341 + XPT2046 + SD Card  ──  Nucleo-F446RE            ║
   * ╠══════════════════════════════════════════════════════════════════════╣
   * ║                                                                      ║
   * ║   Module                        STM32          Nucleo Header         ║
   * ║  ┌─────────┐                                                         ║
   * ║  │  T_IRQ  ├──────────────────  PB2            CN10 morpho           ║
   * ║  │  T_OUT  ├────────────────┐   PA6   ┐        CN7  morpho           ║
   * ║  │  T_DIN  ├──────────────┐ │   PA7   ┤        A4   (CN8)            ║
   * ║  │  T_CS   ├──────────────│─│─  PB1            CN10 morpho           ║
   * ║  │  T_CLK  ├────────────┐ │ │   PA5  ┐        D6   (CN5)             ║
   * ║  │  SDO    ├────────────│─│─┘   PA6   ┘ shared with T_OUT            ║
   * ║  │  LED    ├────────────│─│───  PB0            A3   (CN8)            ║
   * ║  │  SCK    ├────────────┘ │     PA5    shared with T_CLK             ║
   * ║  │  SDI    ├──────────────┘     PA7     shared with T_DIN            ║
   * ║  │  DC/RS  ├──────────────────  PA0            D11  (CN9)            ║
   * ║  │  RESET  ├──────────────────  PC4            CN10 morpho           ║
   * ║  │  CS     ├──────────────────  PA1            D12  (CN9)            ║
   * ║  │  GND    ├──────────────────  GND            any GND               ║
   * ║  │  VCC    ├──────────────────  3.3V           CN6  pin 4            ║
   * ║  └─────────┘                                                         ║
   * ║  │  SD_SCK   ├──────────────────  PA5         shared with T_CLK      ║ 
   * ║  │  SD_MOSI  ├──────────────────  PA7          shared with T_DIN     ║
   * ║  │  SD_MISO  ├──────────────────  PA6          shared with T_OUT     ║
   * ║  │  SD_CS    ├──────────────────  PC5          CN10 morpho           ║
   * ║  SPI2 shared bus:  SCK = PA5  │  MOSI = PA7  │  MISO = PA6           ║
   * ║  Connected These Pins Together Externally:                           ║
   * ║  SDO/T_OUT/SD_MISO  → same wire → PA6                                ║
   * ║  SDI/T_DIN/SD_MOSI  → same wire → PA7                                ║
   * ║  SCK/T_CLK/SD_SCK  → same wire → PA5                                 ║
   * ╚══════════════════════════════════════════════════════════════════════╝
   
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "xpt2046.h"
#include "sd_spi.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static ILI9341_Config lcd_cfg;
static SD_Config      sd_cfg;
static SD_Info        sd_info;
static XPT2046_Config tch_cfg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART debug print
static void uprint(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

// SPI speed control for SD card init (APB1 = 45 MHz)
static void spi_set_slow(void) {
    // ~350 kHz for SD card init (required <= 400 kHz)
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_128);
    __HAL_SPI_ENABLE(&hspi2);
}
static void spi_set_fast(void) {
    // ~2.8 MHz for SD card data (restore default prescaler 16)
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR, SPI_BAUDRATEPRESCALER_16);
    __HAL_SPI_ENABLE(&hspi2);
}

// SPI2 Shared for LCD, Touchscreen, SD Card
static uint8_t spi2_txrx(uint8_t byte)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2, &byte, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

// LCD GPIO
static void lcd_set_cs(int v)
{
    if (v) HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
    else   HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
}

static void lcd_set_dc(int v)
{
    if (v) HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
    else   HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
}

static void lcd_set_rst(int v)
{
    if (v) HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);
    else   HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
}

// SD Card GPIO
static void sd_set_cs(int v)
{
    if (v) HAL_GPIO_WritePin(SD_CS_GPIO_Port,  SD_CS_Pin, GPIO_PIN_SET);
    else   HAL_GPIO_WritePin(SD_CS_GPIO_Port,  SD_CS_Pin, GPIO_PIN_RESET);
}

// Touch Screen GPIO
static void tch_set_cs(int v)
{
    if (v) HAL_GPIO_WritePin(TCH_CS_GPIO_Port, TCH_CS_Pin, GPIO_PIN_SET);
    else   HAL_GPIO_WritePin(TCH_CS_GPIO_Port, TCH_CS_Pin, GPIO_PIN_RESET);
}

static int tch_read_irq(void)
{
    return HAL_GPIO_ReadPin(TCH_IRQ_GPIO_Port, TCH_IRQ_Pin);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // Deassert all SPI CS pins; hold LCD in hardware reset (draws ~0.1 mA
  // vs ~40 mA active) so the SD card gets the full current budget during init.
  lcd_set_cs(1);
  sd_set_cs(1);
  tch_set_cs(1);
  lcd_set_rst(0);   // ILI9341 in reset — minimal current draw
  HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET); // BL off

  // init SD card config (LCD not yet initialised)
  SD_Config sd_cfg = {
    .spi_txrx     = spi2_txrx,
    .spi_set_slow = spi_set_slow,
    .spi_set_fast = spi_set_fast,
    .set_cs       = sd_set_cs,
    .delay_ms     = HAL_Delay,
    .get_ms       = HAL_GetTick,
    .log          = uprint,
  };

  /* ── SD card init via driver ─────────────────────────────────────────── */
  /* Wait 4 s from power-on before first SPI touch — give the SD card's
   * internal controller time to complete its own startup.               */
  uprint("Waiting 1s for SD power-up...\r\n");
  HAL_Delay(1000);
  uprint("Running sd_init...\r\n");
  SD_Info sd_info;
  SD_Error sd_err = sd_init(&sd_cfg, &sd_info);
  {
    char tmp[40];
    snprintf(tmp, sizeof(tmp), "sd_init result: %s\r\n", sd_error_str(sd_err));
    uprint(tmp);
  }

  ILI9341_Config lcd_cfg = {
    .spi_txrx = spi2_txrx,
    .set_cs   = lcd_set_cs,
    .set_dc   = lcd_set_dc,
    .set_rst  = lcd_set_rst,
    .delay_ms = HAL_Delay,
    .rotation = ROTATION_0,
  };
  ili9341_init(&lcd_cfg);
  HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET);

  // init touchscreen
  XPT2046_Config tch_cfg = {
    .spi_txrx = spi2_txrx,
    .set_cs   = tch_set_cs,
    .read_irq = tch_read_irq,
  };
  xpt2046_init(&tch_cfg);
  xpt2046_default_cal(ILI9341_WIDTH, ILI9341_HEIGHT);

  if (sd_err == SD_OK) {
      ili9341_fill_rect(10, 60, 220, 20, COLOR_GREEN);
      ili9341_set_text_color(COLOR_BLACK, COLOR_GREEN);
      char sd_buf[40];
      snprintf(sd_buf, sizeof(sd_buf), "SD OK  %lluMB", sd_info.capacity_mb);
      ili9341_draw_string(20, 64, sd_buf);

      /* ── Read/Write Test ──────────────────────────────────────────────────
       * Write a known pattern to block 8 (safely past the MBR/partition
       * table), read it back and verify byte-for-byte.                     */
      static uint8_t wr_buf[512];
      static uint8_t rd_buf[512];
      for (int i = 0; i < 512; i++) wr_buf[i] = (uint8_t)(i & 0xFF);

      SD_Error we = sd_write_block(8, wr_buf);
      SD_Error re = sd_read_block(8, rd_buf);

      int mismatch = 0;
      if (we == SD_OK && re == SD_OK) {
          for (int i = 0; i < 512; i++) {
              if (rd_buf[i] != wr_buf[i]) { mismatch = i + 1; break; }
          }
      }

      if (we == SD_OK && re == SD_OK && mismatch == 0) {
          ili9341_fill_rect(10, 82, 220, 20, COLOR_GREEN);
          ili9341_set_text_color(COLOR_BLACK, COLOR_GREEN);
          ili9341_draw_string(20, 86, "RW Test PASSED");
      } else {
          ili9341_fill_rect(10, 82, 220, 20, COLOR_RED);
          ili9341_set_text_color(COLOR_WHITE, COLOR_RED);
          if (we != SD_OK)
              snprintf(sd_buf, sizeof(sd_buf), "Write ERR: %s", sd_error_str(we));
          else if (re != SD_OK)
              snprintf(sd_buf, sizeof(sd_buf), "Read ERR: %s", sd_error_str(re));
          else
              snprintf(sd_buf, sizeof(sd_buf), "Mismatch @byte %d", mismatch - 1);
          ili9341_draw_string(20, 86, sd_buf);
      }
  } else {
      char sd_buf[32];
      snprintf(sd_buf, sizeof(sd_buf), "SD: %s", sd_error_str(sd_err));
      ili9341_fill_rect(10, 60, 220, 40, COLOR_RED);
      ili9341_set_text_color(COLOR_WHITE, COLOR_RED);
      ili9341_draw_string(20, 74, sd_buf);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    TouchPoint tp;
    uint16_t rx, ry, rz;
    xpt2046_raw(&rx, &ry, &rz);
    if (xpt2046_read(&tp)) {
      ili9341_fill_circle(tp.x, tp.y, 4, COLOR_YELLOW);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TFT_CS_Pin|TFT_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TFT_RST_Pin|SD_CS_Pin|SLND4_Pin|SLND3_Pin
                          |SLND2_Pin|SLND1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_BL_Pin|TCH_CS_Pin|SLND5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_H_Pin */
  GPIO_InitStruct.Pin = BTN_H_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_H_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin TFT_CS_Pin TFT_DC_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|TFT_CS_Pin|TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RST_Pin SD_CS_Pin SLND4_Pin SLND3_Pin
                           SLND2_Pin SLND1_Pin */
  GPIO_InitStruct.Pin = TFT_RST_Pin|SD_CS_Pin|SLND4_Pin|SLND3_Pin
                          |SLND2_Pin|SLND1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_BL_Pin TCH_CS_Pin SLND5_Pin */
  GPIO_InitStruct.Pin = TFT_BL_Pin|TCH_CS_Pin|SLND5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TCH_IRQ_Pin */
  GPIO_InitStruct.Pin = TCH_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TCH_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN3_Pin BTN2_Pin BTN1_Pin */
  GPIO_InitStruct.Pin = BTN3_Pin|BTN2_Pin|BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
