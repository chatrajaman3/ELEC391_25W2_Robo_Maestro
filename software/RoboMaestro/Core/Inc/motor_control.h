/**
 * @file    motor_control.h
 * @brief   Motor PID control, angle reading, and UART command processing
 *          for STM32 + AS5600 magnetic encoder.
 *
 * Usage:
 *   1. Call MotorControl_Init() once after all HAL peripherals are ready.
 *   2. Call MotorControl_Process() from your main loop.
 *   3. Call MotorControl_TimerISR() from HAL_TIM_PeriodElapsedCallback (TIM3).
 *   4. Call MotorControl_UartISR() from HAL_UART_RxCpltCallback (USART2).
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include <stdint.h>

/* ── External HAL handles (defined in main.c / generated code) ───────────── */
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart2;

/* ── Configuration macros ────────────────────────────────────────────────── */

/* PID output limits (maps directly to TIM1 ARR = 65535) */
#define PID_ABS_MIN_OUTPUT  30000.0f
#define PID_ABS_MAX_OUTPUT  65535.0f

/* Integral windup clamp */
#define INTEGRAL_MAX        1.0f
#define INTEGRAL_MIN       -1.0f

/* Error dead-zone (rad) — inside this band the motor holds position */
#define ERROR_THRESHOLD     0.02f   /* rad */

/* Control-loop timing
 *   TIM3:  PSC=8999, ARR=99, APB1×2 = 90 MHz  →  f_timer = 10 kHz, dt = 10 ms
 *   DT     = (ARR+1) / f_timer  = 100/10000 = 0.01 s
 *   CF     = 1/DT = 100 Hz
 *   tau    = 5×DT = 0.05 s
 *   BETAD  = exp(-DT/tau) ≈ 0.6065   (derivative low-pass)
 *   BETAF  = 0.2                      (feedforward / extra filter)
 */
#define DT                  0.01f
#define CONTROL_FREQUENCY   100.0f
#define TAU                 0.05f
#define BETAD               0.6065f
#define BETAF               0.2f

/* UART receive buffer size */
#define RX_BUFFER_SIZE      32

/* ── Types ───────────────────────────────────────────────────────────────── */

typedef enum {
    MODE_MENU,
    MODE_RECEIVE,
    MODE_READ_VELOCITY,
    MODE_STOP
} UARTMode;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the motor-control module.
 *         Call once after MX_*_Init() functions and AS5600_Init().
 *         Starts TIM1 PWM channels and arms the UART receive interrupt.
 */
void MotorControl_Init(void);

/**
 * @brief  Main-loop task.
 *         Handles pending UART commands and prints telemetry in RECEIVE /
 *         READ_VELOCITY modes.  Call continuously from while(1).
 */
void MotorControl_Process(void);

/**
 * @brief  Call this from HAL_TIM_PeriodElapsedCallback when htim == TIM3.
 *         Reads the encoder, runs the PID, and drives the motor.
 */
void MotorControl_TimerISR(void);

/**
 * @brief  Call this from HAL_UART_RxCpltCallback when huart == USART2.
 *         Echoes the character, assembles the command buffer, and sets the
 *         command_ready flag.
 */
void MotorControl_UartISR(void);

/**
 * @brief  Blocking homing routine.
 *         Spins the motor slowly until the user-button (PC13) is pressed,
 *         then zeros the encoder and starts the TIM3 control-loop interrupt.
 *         Call this instead of (or after) MotorControl_Init() when homing
 *         is required.
 */
void MotorControl_Home(void);

/* ── Optional getters (useful for external logging / debug) ──────────────── */
float        MotorControl_GetAngle(void);       /* current angle, rad      */
float        MotorControl_GetAngVel(void);      /* current ang-vel, rad/s  */
float        MotorControl_GetSetpoint(void);    /* current setpoint, rad   */
float        MotorControl_GetControlSignal(void);
UARTMode     MotorControl_GetMode(void);

/* ── Optional setters ────────────────────────────────────────────────────── */
void MotorControl_SetSetpoint(float rad);       /* set angle target, rad   */
void MotorControl_SetGains(float kp, float ki, float kd);

#endif /* MOTOR_CONTROL_H */
