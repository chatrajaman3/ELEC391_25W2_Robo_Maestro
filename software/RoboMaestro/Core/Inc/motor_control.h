/**
 * @file    motor_control.h
 * @brief   Motor PID control and angle reading for STM32 + AS5600 magnetic encoder.
 *
 * Usage:
 *   1. Call MotorControl_Init() once after all HAL peripherals are ready.
 *   2. Call MotorControl_Process() from your main loop.
 *   3. Call MotorControl_TimerISR() from HAL_TIM_PeriodElapsedCallback (TIM3).
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

/* PID output limits (maps directly to TIM1 ARR = 65535, f_PWM = 2.75 kHz) */
#define PID_ABS_MIN_OUTPUT  7000.0f  /* minimum duty when actively driving  */
#define PID_ABS_MAX_OUTPUT  8998.0f  /* hard ceiling = ARR                  */
/* Dead time inserted between direction changes to prevent H-bridge shoot-through.
 * Both channels are forced to 0 for this many ms before the new direction is applied. */
#define MOTOR_DEADTIME_MS   1

/* Integral windup clamp */
#define INTEGRAL_MAX        1.0f
#define INTEGRAL_MIN       -1.0f

/* Error dead-zone with hysteresis to prevent limit-cycle vibration.
 * Motor stops when error < THRESHOLD_OFF, stays stopped until error > THRESHOLD_ON. */
#define ERROR_THRESHOLD_OFF  0.25f  /* rad — stop motor when inside this band  */
#define ERROR_THRESHOLD_ON   0.40f  /* rad — restart motor when outside this   */

/* Static-friction kickstart: apply max PWM for this many control ticks when
 * transitioning from stopped to moving, to overcome stiction.
 * At 100 Hz control rate, 5 ticks = 50 ms. */
#define KICKSTART_TICKS      10

/* Gear-and-rack geometry */
#define PITCH_RADIUS        0.019f  /* metres — 1.9 cm pitch radius */

/* Control-loop timing
 *   TIM3:  PSC=8999, ARR=99, APB1×2 = 90 MHz  →  f_timer = 10 kHz, dt = 10 ms
 *   DT     = (ARR+1) / f_timer  = 100/10000 = 0.01 s
 *   BETAD  = exp(-DT/tau) ≈ 0.6065   (derivative low-pass, tau = 50 ms)
 */
#define DT                  0.01f
#define CONTROL_FREQUENCY   100.0f
#define TAU                 0.05f
#define BETAD               0.6065f

/* ── Mode enum ───────────────────────────────────────────────────────────── */

typedef enum {
    /* Top-level main menu */
    MODE_MAIN_MENU,

    /* ── Test sub-menu ── */
    MODE_TEST_IDLE,         /* test menu, motor stopped                    */
    MODE_TEST_CW,           /* open-loop CW at max speed                   */
    MODE_TEST_CCW,          /* open-loop CCW at max speed                  */
    MODE_TEST_SWITCH,       /* alternates CW/CCW to exercise dead time      */
    MODE_TEST_FINGER_ONOFF, /* waiting for "on"/"off" reply                */
    MODE_TEST_ACTUATE,      /* button-driven solenoid cycling              */

    /* ── PID sub-menu ── */
    MODE_PID_IDLE,          /* PID menu, motor stopped                     */
    MODE_PID_ACTIVE,        /* PID running, tracking setpoint silently     */
    MODE_PID_RECEIVE,       /* PID running + stream telemetry at 10 Hz    */
    MODE_PID_STOP,          /* motor frozen, setpoint locked to current    */
} UARTMode;

/* Convenience checks */
#define IS_TEST_MODE(m) ((m) >= MODE_TEST_IDLE  && (m) <= MODE_TEST_ACTUATE)
#define IS_PID_MODE(m)  ((m) >= MODE_PID_IDLE   && (m) <= MODE_PID_STOP)

/* Interval between direction changes in switching test mode (ms) */
#define MOTOR_SWITCH_INTERVAL_MS  300

/* UART receive buffer size */
#define RX_BUFFER_SIZE  64

/* ── Public API ──────────────────────────────────────────────────────────── */

void MotorControl_Init(void);
void MotorControl_Process(void);
void MotorControl_TimerISR(void);
void MotorControl_UartISR(void);
void MotorControl_Home(void);

/* Getters */
float    MotorControl_GetAngle(void);
float    MotorControl_GetLinPos(void);        /* cached from last PID tick  */
float    MotorControl_ReadLinPos(void);       /* fresh AS5600 read          */
float    MotorControl_GetSetpoint(void);
float    MotorControl_GetControlSignal(void);
UARTMode MotorControl_GetMode(void);

/* Setters */
void MotorControl_SetSetpoint(float rad);
void MotorControl_SetLinSetpoint(float metres);
void MotorControl_SetGains(float kp, float ki, float kd);
void MotorControl_PlaySong(void);

/* Open-loop test controls */
void MotorControl_SetTestStop(void);
void MotorControl_SetTestCW(void);
void MotorControl_SetTestCCW(void);
void MotorControl_SetTestSwitch(void);

/**
 * @brief  Weak hook for commands not handled by motor_control.
 *         Only called from test mode.  Define a strong version in main.c.
 */
void MotorControl_ExtCommand(const char *cmd);

#endif /* MOTOR_CONTROL_H */
