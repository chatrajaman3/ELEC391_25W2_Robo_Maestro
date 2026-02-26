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
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
  ZONE_LOW, // when encoder count is 0-500
  ZONE_MID, // when encoder count is 500-2250
  ZONE_HIGH // when encoder count is 2250-2750
} CounterZone; // for rotation tracking

typedef enum{
  MODE_MENU,
  MODE_RECEIVE,
  MODE_READ_VELOCITY,
  MODE_CW,
  MODE_CCW,
  MODE_ACTUATE,
  MODE_STOP
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
#define DT 0.1

// other defines
#define RX_BUFFER_SIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

  // Logic and Reading variables
  int32_t rotation_count = 0;
  CounterZone zone_curr = ZONE_MID;
  CounterZone zone_prev = ZONE_MID;
  float ang_curr = 0; 
  float ang_prev = 0;
  float ang_vel = 0; 
  uint16_t tick_curr = 0;
  uint16_t tick_prev = 0;
  uint16_t count_curr = 0;
  uint16_t count_prev = 0;

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
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len);
void PrintValues(void);
float GetMotorAngle(void);
float GetMotorAngVel(void);
void IRRFilderD(float* sigD);
void IRRFilderF(float* sigF);
float PID(void);
void MotorStop(void);
void MotorCW(uint16_t duty_cycle);
void MotorCCW(uint16_t duty_cycle);
void MotorControl(void);
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

// this function redirects printf output to UART3
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void ProcessCommand(char *cmd)
{
    if(current_mode != MODE_MENU){ // enter m to return to menu
      if(strncmp(cmd, "m", 1) ==0){
        current_mode = MODE_MENU;
        printf("Switched to MENU mode\r\n");
      }
    }
    else if(strncmp(cmd, "cw", 2) == 0) // enter cw to rotate clockwise
    {
        current_mode = MODE_CW;
        printf("Switched to CW mode\r\n");
    }
    else if(strncmp(cmd, "ccw", 3) == 0) // enter ccw to rotate counter-clockwise
    {
        current_mode = MODE_CCW;
        printf("Switched to CCW mode\r\n");
    }
    else if(strncmp(cmd, "act", 3) == 0) // enter act to actuate motor
    {
        current_mode = MODE_ACTUATE;
        printf("Switched to ACTUATE mode\r\n");
    }
    else if(strncmp(cmd, "v", 1) == 0) // enter v to read angular velocity
    {
        current_mode = MODE_READ_VELOCITY;
        printf("Switched to READ VELOCITY mode\r\n");
    }
    else if(strncmp(cmd, "s", 1) == 0) // enter s to stop motor
    {
        current_mode = MODE_STOP;
        printf("Switched to STOP mode\r\n");
    }
    else // if command not recognized, print error message
    {
        printf("Unknown command\r\n");
    }
}

float GetMotorAngle(){ // tim2 counter period is 2750
  count_curr = __HAL_TIM_GET_COUNTER(&htim2);

  zone_prev = zone_curr;

  // get current zone
  if(count_curr <= 500){
    zone_curr = ZONE_LOW;
  }
  else if(count_curr >= 2250){
    zone_curr = ZONE_HIGH;
  }
  else{
    zone_curr = ZONE_MID;
  }
  count_prev = count_curr;

  // if going from high to low add rotation, if low to high subtract rotation
  if(zone_prev == ZONE_HIGH && zone_curr == ZONE_LOW){
    rotation_count++;
  }
  else if(zone_prev == ZONE_LOW && zone_curr == ZONE_HIGH){
    rotation_count--;
  }

  return ((float)rotation_count + ((float)__HAL_TIM_GET_COUNTER(&htim2) / 2750.0)) * 2.0 * M_PI; // rotations + angle
}

float GetMotorAngVel(){
    ang_vel = (ang_curr - ang_prev) / DT;
    return ang_vel;
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // start the timer in encoder mode
 
  // start tim1 PWM channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  MotorStop();

  // start tim2 in encoder mode
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // set PB12 low

  Button_Init(&blue_button, GPIOC, GPIO_PIN_13, 20); // initialize blue button with 20ms debounce delay
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(command_ready){
      ProcessCommand(rx_buffer);
      command_ready = 0;
    }
    if(current_mode == MODE_READ_VELOCITY){
      printf("ang_vel: %.2f\r\n", ang_vel);
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
        actuation_state = !actuation_state; // toggle state
        printf("Actuation state: %d\r\n", actuation_state);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, actuation_state); // toggle PB10
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, actuation_state); // toggle PB4
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2750;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)  
    {
        /* USER CODE BEGIN TIM3_ISR */
        ang_curr = GetMotorAngle();
        ang_vel = GetMotorAngVel();
        ang_prev = ang_curr;
        /* USER CODE END TIM3_ISR */
    }
}
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
