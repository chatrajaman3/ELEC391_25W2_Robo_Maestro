# Function Headers, Code, and Includes to copy and paste

## Includes 
```c
#include <stdio.h>  
#include <math.h>  
#include <string.h>  
#include <stdlib.h>  
```
## Function Prototypes

### Serial Communication and Testing UI:
```c
int _write(int file, char *ptr, int len);  
void ProcessCommand(char *cmd);  
void PrintValues(void);  
```
### Reading Encoder:
```c
float GetMotorAngle(void);  
float GetMotorAngVel(void);  
void IRRFilderD(float* sigD);  
void IRRFilderF(float* sigF);  
```
### PID:
```c
float PID(void);  
```
### Motor Control: 
```c
void MotorStop(void);  
void MotorCW(uint16_t duty_cycle);  
void MotorCCW(uint16_t duty_cycle);  
void MotorControl(void);  
```
### Detecting Button Pushes:
```c
void Button_Init(Button_t *btn, GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_delay)  
uint8_t Button_Update(Button_t *btn)  
```
## Functions

### Serial Communication and Testing UI:
```c
// redirects printf output to UART2
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


// you need this to have this struct to use ProcessCommand
typedef enum{
  MODE_MENU,
  MODE_RECEIVE,
  MODE_READ_VELOCITY,
  MODE_ACTUATE,
  MODE_CCW,
  MODE_CW
  MODE_STOP
} UARTMode;

// processes the string that gets typed through UART 2
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

// interrupt that gets called every time a character is typed in COM port, it stores the string into rx_buffer, it sets command_ready=1 when you hit enter, it re-arms the interrupt after
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

// used to spin motor slowly and then stop and set 0-point 
void AngInit(void)
{
  MotorCW(33000);   // Spin slowly

  while (1)
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
      HAL_Delay(20);  // debounce

      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
        MotorStop();
        __HAL_TIM_SET_COUNTER(&htim2, 0);  // reset encoder
        ang_curr = 0;

        HAL_TIM_Base_Start_IT(&htim3);     // start PID loop

        break;  // exit homing loop
      }
    }
  }
}
```
### Reading Encoder:
```c
```
### PID:
```c
// output is a counter number that sets the duty cycle for the PWM
float PID(){
  err = ang_set - ang_curr;
  // if within treshold, zero error and integral
  if(fabsf(err) < ERROR_THRESHOLD){
    err = 0;
    integral = 0;
  }
  integral += err * DT;
  derivative = (err - err_prev) / DT;
  IRRFilderD(&derivative);
  err_prev = err;
  ang_prev = ang_curr;

  // clamp integral
  if(integral > INTEGRAL_MAX){
    integral = INTEGRAL_MAX;
  }
  if(integral < INTEGRAL_MIN){
    integral = INTEGRAL_MIN;
  }

  P = Kp * err;
  I = Ki * integral;
  D = -Kd * derivative;
  control_signal = P + I + D;

  // clamp control signal
  if(control_signal > PID_ABS_MAX_OUTPUT){
    control_signal = PID_ABS_MAX_OUTPUT;
  }
  if(control_signal < -PID_ABS_MAX_OUTPUT){
    control_signal = -PID_ABS_MAX_OUTPUT;
  }
  return control_signal;
}
```
### Motor Control:
```c
// we are using channel 1 & 2 on timer 1 (16-bit) to set the PWM output to our motor driver

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

// calls the PID function, sets motor either in CW or CCW direction based on sign of the control signal, and sets the PWM duty cycle according to the magnitude eof the control signal
void MotorControl(){
  control_signal = PID(); // scale control signal to timer period
  if(fabsf(control_signal) < PID_ABS_MIN_OUTPUT){
    MotorStop();
  }
  else if (control_signal > 0){
    MotorCW((uint16_t)control_signal);
  }
  else{
    MotorCCW((uint16_t)(-control_signal));
  }
}
```
### Detecting Button Pushes:
```c
// initializes button variable for a pin, sets the debounce delay
void Button_Init(Button_t *btn, GPIO_TypeDef *port, uint16_t pin, uint32_t debounce_delay)
{
    btn->port = port;
    btn->pin = pin;
    btn->debounce_delay = debounce_delay;

    btn->last_reading = HAL_GPIO_ReadPin(port, pin);
    btn->stable_state = btn->last_reading;
    btn->last_debounce_time = 0;
}

// uses the system clk and button states to check if the button was pressed
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
```
### Filtering:
```c
// IRR filter for derivative branch of PID
void IRRFilderD(float* sigD){
  static float sigfilt_prev;
  *sigD = BETAD * sigfilt_prev + (1 - BETAD) * (*sigD);
  sigfilt_prev = *sigD;
}

// IRR filter for feedback path
void IRRFilderF(float* sigF){
  static float sigfilt_prev;
  *sigF = BETAF * sigfilt_prev + (1 - BETAF) * (*sigF);
  sigfilt_prev = *sigF;
}
```