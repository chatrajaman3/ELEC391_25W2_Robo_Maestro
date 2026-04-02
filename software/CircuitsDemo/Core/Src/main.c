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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "actuator.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  MODE_MENU,
  MODE_CW,
  MODE_CCW,
  MODE_ACTUATE,
  MODE_STOP,
  MODE_FINGER_ONOFF  // waiting for "on" or "off" after "fingers" command
} UARTMode;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;

    uint8_t last_reading;
    uint8_t stable_state;

    uint32_t last_debounce_time;
    uint32_t debounce_delay;
} Button_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 32
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

  // Finger actuator pins
  // Finger: 1 (SLND1)   2 (SLND2)   3 (SLND3)   4 (SLND4)   5 (SLND5)
  GPIO_Pin_t fingers[5] = {
    {GPIOC, SLND1_Pin},
    {GPIOC, SLND2_Pin},
    {GPIOC, SLND3_Pin},
    {GPIOC, SLND4_Pin},
    {GPIOB, SLND5_Pin}
  };
  uint8_t selected_fingers = 0; // bitmask of fingers chosen by "fingers" command

  // other variables
  uint8_t rx_char;
  char rx_buffer[RX_BUFFER_SIZE];
  uint8_t rx_index = 0;
  volatile uint8_t command_ready = 0;
  UARTMode current_mode = MODE_MENU;
  uint8_t actuation_state = 0;
  Button_t blue_button;

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

int _write(int file, char *ptr, int len);
void MotorStop(void);
void MotorCW(uint16_t duty_cycle);
void MotorCCW(uint16_t duty_cycle);
void ProcessCommand(char *cmd);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Button_Init(Button_t *btn, GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_delay)
{
    btn->port = port;
    btn->pin = pin;
    btn->debounce_delay = debounce_delay;

    btn->last_reading = HAL_GPIO_ReadPin(port, pin);
    btn->stable_state = btn->last_reading;
    btn->last_debounce_time = 0;
}

// this function redirects printf output to UART2
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void ProcessCommand(char *cmd)
{
    // 'm' returns to menu from any mode
    if (current_mode != MODE_MENU && strncmp(cmd, "m", 1) == 0)
    {
        current_mode = MODE_MENU;
        printf("Switched to MENU mode\r\n");
        return;
    }

    // Waiting for "on" or "off" after a "fingers" command
    if (current_mode == MODE_FINGER_ONOFF)
    {
        if (strncmp(cmd, "on", 2) == 0)
        {
            for (int i = 0; i < 5; i++) {
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_SET);
            }
            printf("Fingers ON\r\n");
        }
        else if (strncmp(cmd, "off", 3) == 0)
        {
            for (int i = 0; i < 5; i++) {
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
            }
            printf("Fingers OFF\r\n");
        }
        else
        {
            printf("Type 'on', 'off', or 'm' for menu\r\n");
            return;
        }
        current_mode = MODE_MENU;
        return;
    }

    // Menu-mode commands
    if (strncmp(cmd, "fingers", 7) == 0)
    {
        char *p = cmd + 7;
        char *endptr;
        selected_fingers = 0;
        int8_t state = -1; // -1 = not set, 0 = off, 1 = on

        while (*p)
        {
            // skip spaces
            while (*p == ' ') p++;
            if (*p == '\0') break;

            // try to parse as a number first
            long n = strtol(p, &endptr, 10);
            if (endptr != p)
            {
                if (n >= 1 && n <= 5) selected_fingers |= (1 << (n - 1));
                p = endptr;
            }
            else
            {
                // not a number — check for "on" or "off"
                if (strncmp(p, "on", 2) == 0 && (p[2] == ' ' || p[2] == '\0'))
                {
                    state = 1;
                    p += 2;
                }
                else if (strncmp(p, "off", 3) == 0 && (p[3] == ' ' || p[3] == '\0'))
                {
                    state = 0;
                    p += 3;
                }
                else
                {
                    // skip unknown token
                    while (*p && *p != ' ') p++;
                }
            }
        }

        if (selected_fingers == 0)
        {
            printf("No valid fingers (1-5). Usage: fingers 1 2 3 on\r\n");
        }
        else if (state == -1)
        {
            // no on/off provided — store selection and prompt
            printf("Selected fingers:");
            for (int i = 0; i < 5; i++) {
                if (selected_fingers & (1 << i)) printf(" %d", i + 1);
            }
            printf("\r\nTurn on or off? (on/off): ");
            current_mode = MODE_FINGER_ONOFF;
        }
        else
        {
            GPIO_PinState pin_state = (state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            for (int i = 0; i < 5; i++) {
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, pin_state);
            }
            printf("Fingers");
            for (int i = 0; i < 5; i++) {
                if (selected_fingers & (1 << i)) printf(" %d", i + 1);
            }
            printf(" %s\r\n", (state == 1) ? "ON" : "OFF");
        }
    }
    else if (strncmp(cmd, "cw", 2) == 0)
    {
        current_mode = MODE_CW;
        printf("Switched to CW mode\r\n");
    }
    else if (strncmp(cmd, "ccw", 3) == 0)
    {
        current_mode = MODE_CCW;
        printf("Switched to CCW mode\r\n");
    }
    else if (strncmp(cmd, "s", 1) == 0)
    {
        current_mode = MODE_STOP;
        printf("Switched to STOP mode\r\n");
    }
    else
    {
        printf("Unknown command\r\n");
    }
}

void MotorStop(){
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // 0% duty cycle
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // 0% duty cycle
}

void MotorCW(uint16_t duty_cycle){
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle); // set duty cycle
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // 0% duty cycle
}
void MotorCCW(uint16_t duty_cycle){
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // 0% duty cycle
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_cycle); // set duty cycle
}
uint8_t Button_Update(Button_t *btn)
{
	uint8_t reading = HAL_GPIO_ReadPin(btn->port, btn->pin);

	if(reading != btn->last_reading)
	{
		btn->last_debounce_time = HAL_GetTick();
	}

	if((HAL_GetTick() - btn->last_debounce_time) > btn->debounce_delay)
	{
		if(reading != btn->stable_state)
		{
			btn->stable_state = reading;

			if(btn->stable_state == GPIO_PIN_RESET)
			{
				btn->last_reading = reading;
				return 1;
			}
		}
	}

	btn->last_reading = reading;
	return 0;
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
  // start tim1 PWM channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  MotorStop();

  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // set PB12 low

  Actuator_Init(5, fingers[0], fingers[1], fingers[2], fingers[3], fingers[4]);

  Button_Init(&blue_button, GPIOC, GPIO_PIN_13, 20); // initialize blue button with 20ms debounce delay

  while(1){
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_Delay(1000);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(command_ready){
      ProcessCommand(rx_buffer);
      command_ready = 0;
    }
    if(current_mode == MODE_CW){
      MotorCW(65535); // CW Max Speed
      current_mode = MODE_MENU;
    }
    if(current_mode == MODE_CCW){
      MotorCCW(65535); // CCW Max Speed
      current_mode = MODE_MENU;
    }
    if(current_mode == MODE_ACTUATE){
      if(Button_Update(&blue_button)){ // if blue button is pressed
        actuation_state++;
        if(actuation_state == 4){
          actuation_state = 0;
        }
        printf("Actuation state: %d\r\n", actuation_state);
        if(actuation_state == 0){
          HAL_GPIO_WritePin(SLND1_GPIO_Port, SLND1_Pin, 0);
          HAL_GPIO_WritePin(SLND2_GPIO_Port, SLND2_Pin, 0);
        }
        if(actuation_state == 1){
          HAL_GPIO_WritePin(SLND1_GPIO_Port, SLND1_Pin, 1);
          HAL_GPIO_WritePin(SLND2_GPIO_Port, SLND2_Pin, 0);
        }
        if(actuation_state == 2){
          HAL_GPIO_WritePin(SLND1_GPIO_Port, SLND1_Pin, 0);
          HAL_GPIO_WritePin(SLND2_GPIO_Port, SLND2_Pin, 1);
        }
        if(actuation_state == 3){
          HAL_GPIO_WritePin(SLND1_GPIO_Port, SLND1_Pin, 1);
          HAL_GPIO_WritePin(SLND2_GPIO_Port, SLND2_Pin, 1);
        }
        
      }
    }
    if(current_mode == MODE_STOP){
      MotorStop();
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_GPIO_WritePin(GPIOB, TFT_BL_Pin|GPIO_PIN_14|SLND5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : TFT_BL_Pin PB14 SLND5_Pin */
  GPIO_InitStruct.Pin = TFT_BL_Pin|GPIO_PIN_14|SLND5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UART_Transmit(&huart2, &rx_char, 1, HAL_MAX_DELAY);
        // Detect end of command
        if (rx_char == '\r' || rx_char == '\n')
        {
            if (rx_index > 0)   // Ignore empty ENTER presses
            {
                rx_buffer[rx_index] = '\0';   // Null terminate string
                command_ready = 1;
                rx_index = 0;
            }
        }
        else
        {
            // Store received character if buffer not full
            if (rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = rx_char;
            }
            else
            {
                // Buffer overflow protection
                rx_index = 0;
            }
        }

        // Re-arm UART interrupt (VERY IMPORTANT)
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    }
}
    

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
