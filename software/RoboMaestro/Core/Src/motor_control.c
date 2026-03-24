/**
 * @file    motor_control.c
 * @brief   Motor PID control, angle reading, and UART command processing.
 *
 * See motor_control.h for the public API and configuration macros.
 */

#include "motor_control.h"
#include "as5600_position.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ── Private state ───────────────────────────────────────────────────────── */

/* Angle / velocity */
static float ang_set  = 0.0f;
static float ang_curr = 0.0f;
static float ang_prev = 0.0f;
static float ang_vel  = 0.0f;

/* PID gains — tunable at runtime via MotorControl_SetGains() */
static float Kp = 350000.0f;
static float Ki =  20000.0f;
static float Kd =  15000.0f;

/* PID working variables */
static float err          = 0.0f;
static float integral     = 0.0f;
static float derivative   = 0.0f;
static float control_signal = 0.0f;
static float P = 0.0f, I = 0.0f, D = 0.0f;

/* UART */
static uint8_t  rx_char;
static char     rx_buffer[RX_BUFFER_SIZE];
static uint8_t  rx_index = 0;
static volatile uint8_t command_ready = 0;
static UARTMode current_mode = MODE_MENU;

/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Redirect printf → UART2 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* -------------------------------------------------------------------------- */
static void PrintValues(void)
{
    printf(" ang_curr: %.2f, err: %.2f, P %.2f, I %.2f, D %.2f, Output: %.2f\r\n",
           ang_curr, err, P, I, D, control_signal);
}

/* -------------------------------------------------------------------------- */
static float GetMotorAngle(void)
{
    AS5600_Update();
    AS5600_Position_t pos;
    AS5600_GetPosition(&pos);
    return (pos.absolute_ticks / 4096.0f) * 2.0f * (float)M_PI;
}

/* -------------------------------------------------------------------------- */
/** IIR low-pass filter for the derivative term (coefficient BETAD). */
static void IRRFilterD(float *sigD)
{
    static float prev = 0.0f;
    *sigD   = BETAD * prev + (1.0f - BETAD) * (*sigD);
    prev    = *sigD;
}

/** IIR low-pass filter with coefficient BETAF (reserved / feedforward). */
static void IRRFilterF(float *sigF)
{
    static float prev = 0.0f;
    *sigF   = BETAF * prev + (1.0f - BETAF) * (*sigF);
    prev    = *sigF;
}

/* -------------------------------------------------------------------------- */
static float PID_Compute(void)
{
    err = ang_set - ang_curr;

    /* Dead-zone: freeze error and integral when sufficiently close */
    if (fabsf(err) < ERROR_THRESHOLD) {
        err      = 0.0f;
        integral = 0.0f;
    }

    integral   += err * DT;
    derivative  = (ang_curr - ang_prev) / DT;
    IRRFilterD(&derivative);

    ang_prev = ang_curr;     /* advance history */

    /* Clamp integral */
    if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
    if (integral <  INTEGRAL_MIN) integral =  INTEGRAL_MIN;

    P = Kp * err;
    I = Ki * integral;
    D = -Kd * derivative;
    control_signal = P + I + D;

    /* Clamp output */
    if (control_signal >  PID_ABS_MAX_OUTPUT) control_signal =  PID_ABS_MAX_OUTPUT;
    if (control_signal < -PID_ABS_MAX_OUTPUT) control_signal = -PID_ABS_MAX_OUTPUT;

    return control_signal;
}

/* -------------------------------------------------------------------------- */
static void MotorStop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

static void MotorCW(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

static void MotorCCW(uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
}

static void MotorDrive(void)
{
    float sig = PID_Compute();

    if (fabsf(sig) < PID_ABS_MIN_OUTPUT) {
        MotorStop();
    } else if (sig > 0.0f) {
        MotorCW((uint16_t)sig);
    } else {
        MotorCCW((uint16_t)(-sig));
    }
}

/* -------------------------------------------------------------------------- */
static void ProcessCommand(char *cmd)
{
    /* 'm' returns to menu from any mode */
    if (current_mode != MODE_MENU) {
        if (strncmp(cmd, "m", 1) == 0) {
            current_mode = MODE_MENU;
            printf("Switched to MENU mode\r\n");
            return;
        }
    }

    if (strncmp(cmd, "set ", 4) == 0) {
        /* "set <degrees>"  — change the position setpoint */
        float deg = atof(&cmd[4]);
        ang_set = deg * (float)M_PI / 180.0f;
        printf("New set angle: %.2f rad (%.1f deg)\r\n", ang_set, deg);
    }
    else if (strncmp(cmd, "r", 1) == 0) {
        current_mode = MODE_RECEIVE;
        printf("Switched to RECEIVE mode\r\n");
    }
    else if (strncmp(cmd, "v", 1) == 0) {
        current_mode = MODE_READ_VELOCITY;
        printf("Switched to READ VELOCITY mode\r\n");
    }
    else if (strncmp(cmd, "s", 1) == 0) {
        current_mode = MODE_STOP;
        printf("Switched to STOP mode\r\n");
    }
    else {
        printf("Unknown command\r\n");
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void MotorControl_Init(void)
{
    /* Start PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    MotorStop();

    /* Arm first UART receive interrupt */
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);

    /* NOTE: TIM3 (control loop) is NOT started here so the caller can choose
     * whether to start it directly or via MotorControl_Home(). */
}

/* -------------------------------------------------------------------------- */
void MotorControl_Process(void)
{
    /* Handle any completed UART command */
    if (command_ready) {
        ProcessCommand(rx_buffer);
        command_ready = 0;
    }

    /* Mode-specific main-loop tasks */
    switch (current_mode) {
        case MODE_RECEIVE:
            PrintValues();
            break;

        case MODE_READ_VELOCITY:
            printf("ang_vel: %.2f rad/s\r\n", ang_vel);
            break;

        case MODE_STOP:
            MotorStop();
            ang_set = ang_curr;   /* freeze setpoint so PID stays quiet */
            break;

        default:
            break;
    }
}

/* -------------------------------------------------------------------------- */
void MotorControl_TimerISR(void)
{
    /* 1. Read current angle from encoder */
    ang_curr = GetMotorAngle();

    /* 2. Compute angular velocity (simple backward difference) */
    ang_vel = (ang_curr - ang_prev) / DT;

    /* 3. Drive the motor (PID is inside MotorDrive) */
    if (current_mode != MODE_STOP) {
        MotorDrive();
    } else {
        MotorStop();
    }
}

/* -------------------------------------------------------------------------- */
void MotorControl_UartISR(void)
{
    /* Echo character back so the terminal shows what was typed */
    HAL_UART_Transmit(&huart2, &rx_char, 1, HAL_MAX_DELAY);

    if (rx_char == '\r' || rx_char == '\n') {
        if (rx_index > 0) {
            rx_buffer[rx_index] = '\0';
            command_ready = 1;
            rx_index = 0;
        }
    } else {
        if (rx_index < RX_BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = rx_char;
        } else {
            rx_index = 0;   /* overflow: discard */
        }
    }

    /* Re-arm interrupt */
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
}

/* -------------------------------------------------------------------------- */
void MotorControl_Home(void)
{
    /* Spin slowly CW until the blue user-button (PC13, active-low) is pressed.
     * Then zero the encoder and start the TIM3 control-loop interrupt. */
    MotorCW(33000);

    while (1) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            HAL_Delay(20);   /* debounce */
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
                MotorStop();
                AS5600_ResetPosition();
                ang_curr = 0.0f;
                ang_prev = 0.0f;
                HAL_TIM_Base_Start_IT(&htim3);   /* start PID loop */
                break;
            }
        }
    }
}

/* ── Getters / setters ───────────────────────────────────────────────────── */

float    MotorControl_GetAngle(void)         { return ang_curr; }
float    MotorControl_GetAngVel(void)        { return ang_vel;  }
float    MotorControl_GetSetpoint(void)      { return ang_set;  }
float    MotorControl_GetControlSignal(void) { return control_signal; }
UARTMode MotorControl_GetMode(void)          { return current_mode; }

void MotorControl_SetSetpoint(float rad)
{
    ang_set = rad;
}

void MotorControl_SetGains(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}
