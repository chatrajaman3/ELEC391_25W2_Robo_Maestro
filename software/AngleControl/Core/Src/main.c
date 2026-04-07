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
  MODE_READ_VELOCITY
} UARTMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// PID control limits
#define PID_ABS_MAX_OUTPUT 65535.0
#define PID_ABS_MIN_OUTPUT 5000.0
// integral windup limits
#define INTEGRAL_MAX 100.0
#define INTEGRAL_MIN -100.0
// error deadzone
#define ERROR_THRESHOLD 1.0

/* control loop timing */
/*
Hz ftimer = fclk/(PSC+1)
dt = (ARR+1)/ftimer
CF = 1/dt 
tau = 5*dt (for now)
beta = exp(-dt/tau)
*/
#define DT 0.01 
#define CONTROL_FREQUENCY 100.0 
#define TAU 0.05
#define BETA 0.818730753078 

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
  float ang_set = 0;
  float ang_curr = 0; 
  float ang_prev = 0;
  float ang_vel = 0; 
  uint16_t tick_curr = 0;
  uint16_t tick_prev = 0;
  uint16_t count_curr = 0;
  uint16_t count_prev = 0;

  // PID parameters
  float Kp = 500.0;
  float Ki = 0.0;
  float Kd = 0.0;

  // PID variables
  float err = 0;
  float err_prev = 0;
  float integral = 0;
  float derivative = 0;
  float control_signal = 0;

  // other variables
  uint8_t rx_char;
  char rx_buffer[RX_BUFFER_SIZE];
  uint8_t rx_index = 0;
  volatile uint8_t command_ready = 0;
  UARTMode current_mode = MODE_MENU;

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
void IRRFilter(float* sig);
float PID(void);
void MotorStop(void);
void MotorCW(uint16_t duty_cycle);
void MotorCCW(uint16_t duty_cycle);
void MotorControl(void);
void ProcessCommand(char *cmd);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    else if(strncmp(cmd, "set ", 4) == 0) // enter set (angle) to change the setpoint angle
    {
        float val = atof(&cmd[4]);
        ang_set = val;
        printf("New set angle: %.2f\r\n", ang_set);
    }
    else if(strncmp(cmd, "r", 1) == 0) // enter r to receive info in the from PrintValues()
    {
        current_mode = MODE_RECEIVE;
        printf("Switched to RECEIVE mode\r\n");
    }
    else if(strncmp(cmd, "v", 1) == 0) // enter v to read angular velocity
    {
        current_mode = MODE_READ_VELOCITY;
        printf("Switched to READ VELOCITY mode\r\n");
    }
    else // if command not recognized, print error message
    {
        printf("Unknown command\r\n");
    }
}

void PrintValues(){
    printf("ang_curr: %.2f, err: %.2f, integral %.2f, derivative %.2f, Control Output: %.2f \r\n", ang_curr, err, integral, derivative, control_signal);
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

  return ((float)rotation_count + ((float)__HAL_TIM_GET_COUNTER(&htim2) / 2750.0)) * 360.0; // rotations + angle
}
void IRRFilter(float* sig){
  static float sigfilt_prev;
  *sig = BETA * sigfilt_prev + (1 - BETA) * (*sig);
  sigfilt_prev = *sig;
}

float PID(){
  err = ang_set - ang_curr;
  // if within treshold, zero error and integral
  if(fabsf(err) < ERROR_THRESHOLD){
    err = 0;
    integral = 0;
  }
  integral += err * DT;
  derivative = (err - err_prev) / DT;
  IRRFilter(&derivative);
  err_prev = err;




  // clamp integral
  if(integral > INTEGRAL_MAX){
    integral = INTEGRAL_MAX;
  }
  if(integral < INTEGRAL_MIN){
    integral = INTEGRAL_MIN;
  }

  float P = Kp * err;
  float I = Ki * integral;
  float D = Kd * derivative;
  control_signal = P + I + D;

  // clamp control signal
  if(control_signal > PID_ABS_MAX_OUTPUT){
    control_signal = PID_ABS_MAX_OUTPUT;
  }
  if(control_signal < -PID_ABS_MAX_OUTPUT){
    control_signal = -PID_ABS_MAX_OUTPUT;
  }
  if(control_signal > 0 && control_signal < PID_ABS_MIN_OUTPUT){
    control_signal = PID_ABS_MIN_OUTPUT;
  }
  if(control_signal < 0 && control_signal > -PID_ABS_MIN_OUTPUT){
    control_signal = -PID_ABS_MIN_OUTPUT;
  }
  return control_signal;
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

void MotorControl(){
  control_signal = PID();
  if(fabsf(control_signal) < 100.0){
    MotorStop();
  }
  else if (control_signal > 0){
    MotorCW((uint16_t)control_signal);
  }
  else{
    MotorCCW((uint16_t)(-control_signal));
  }
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
  // initialize both PWM channels to 0% duty cycle
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

  // start tim2 in encoder mode
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 0);  // start counter at 0

  HAL_TIM_Base_Start_IT(&htim3); // enable ISR for tim3
  HAL_UART_Receive_IT(&huart2, &rx_char, 1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(command_ready){
      ProcessCommand(rx_buffer);
      command_ready = 0;
    }
    if(current_mode == MODE_RECEIVE){
      PrintValues();
    }
    if(current_mode == MODE_READ_VELOCITY){
      printf("ang_vel: %.2f\r\n", ang_vel);
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
  htim1.Init.Period = 8999;
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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)  // Make sure it’s TIM3
    {
        /* USER CODE BEGIN TIM3_ISR */
        ang_curr = GetMotorAngle();
        MotorControl();
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
