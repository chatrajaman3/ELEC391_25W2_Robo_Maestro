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

/* Angle / linear position */
static float   ang_set    = 0.0f;
static float   ang_curr   = 0.0f;
static float   ang_prev   = 0.0f;
static float   lin_pos    = 0.0f;  /* metres */
static int64_t home_offset = 0;

/* PID gains — tunable at runtime via MotorControl_SetGains() */
static float Kp = 50000.0f;
static float Ki =  10000.0f;
static float Kd =  15000.0f;

/* PID working variables */
static float err          = 0.0f;
static float integral     = 0.0f;
static float derivative   = 0.0f;
static float control_signal = 0.0f;
static float actual_pwm     = 0.0f;  /* clamped value actually sent to timer */
static float P = 0.0f, I = 0.0f, D = 0.0f;

/* UART */
static uint8_t  rx_char;
static char     rx_buffer[RX_BUFFER_SIZE];
static uint8_t  rx_index = 0;
static volatile uint8_t command_ready = 0;
static UARTMode current_mode = MODE_MENU;

/* Control-loop tick flag — set by ISR, cleared in Process() */
static volatile uint8_t pid_tick = 0;

/* Print rate divider: print every 10 ticks = 10 Hz */
static uint8_t print_div = 0;

/* ── Private helpers ─────────────────────────────────────────────────────── */

/* Redirect printf → UART2 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* -------------------------------------------------------------------------- */
static void PrintMenu(void)
{
    printf("\r\n=== Motor Control Commands ===\r\n");
    printf("  set <deg>            Set position setpoint (degrees)\r\n");
    printf("  setl <cm>            Set linear position setpoint (cm)\r\n");
    printf("  a                    Print current position & setpoint\r\n");
    printf("  r                    RECEIVE mode: stream PID telemetry at 10 Hz\r\n");
    printf("  s                    STOP motor\r\n");
    printf("  gains <kp> <ki> <kd> Update PID gains\r\n");
    printf("  m                    Return to MENU (stop streaming)\r\n");
    printf("  ?                    Show this menu\r\n");
    printf("==============================\r\n");
}

/* -------------------------------------------------------------------------- */
static void PrintValues(void)
{
    printf("lin: %.4f m  err: %.4f rad  pid: %.0f  pwm: %.0f (%.1f%%)\r\n",
           lin_pos, err, control_signal, actual_pwm,
           (actual_pwm < 0.0f ? -actual_pwm : actual_pwm) / PID_ABS_MAX_OUTPUT * 100.0f);
}

/* -------------------------------------------------------------------------- */
static float GetMotorAngle(void)
{
    AS5600_Update();
    AS5600_Position_t pos;
    AS5600_GetPosition(&pos);
    return -((pos.absolute_ticks - home_offset) / 4096.0f) * 2.0f * (float)M_PI;
}

/* -------------------------------------------------------------------------- */
/** IIR low-pass filter for the derivative term (coefficient BETAD). */
static void IRRFilterD(float *sigD)
{
    static float prev = 0.0f;
    *sigD   = BETAD * prev + (1.0f - BETAD) * (*sigD);
    prev    = *sigD;
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

    /* Stop only when error is within dead-zone */
    if (fabsf(err) < ERROR_THRESHOLD) {
        actual_pwm = 0.0f;
        MotorStop();
        return;
    }

    /* Clamp up to minimum duty when actively driving */
    if (sig > 0.0f) {
        if (sig < PID_ABS_MIN_OUTPUT) sig = PID_ABS_MIN_OUTPUT;
        actual_pwm = sig;
        MotorCW((uint16_t)sig);
    } else {
        if (-sig < PID_ABS_MIN_OUTPUT) sig = -PID_ABS_MIN_OUTPUT;
        actual_pwm = sig;   /* negative = CCW */
        MotorCCW((uint16_t)(-sig));
    }
}

/* -------------------------------------------------------------------------- */
static void ProcessCommand(char *cmd)
{
    if (strncmp(cmd, "setl ", 5) == 0) {
        /* "setl <cm>"  — set linear position setpoint */
        float cm = atof(&cmd[5]);
        ang_set = (cm / 100.0f) / PITCH_RADIUS;
        printf("Setpoint: %.2f cm  (%.2f rad)\r\n", cm, ang_set);
    }
    else if (strncmp(cmd, "set ", 4) == 0) {
        /* "set <degrees>"  — change the position setpoint */
        float deg = atof(&cmd[4]);
        ang_set = deg * (float)M_PI / 180.0f;
        printf("Setpoint: %.1f deg  (%.4f m)\r\n", deg, ang_set * PITCH_RADIUS);
    }
    else if (strncmp(cmd, "gains ", 6) == 0) {
        /* "gains <kp> <ki> <kd>"  — update PID gains at runtime */
        float kp, ki, kd;
        if (sscanf(&cmd[6], "%f %f %f", &kp, &ki, &kd) == 3) {
            Kp = kp; Ki = ki; Kd = kd;
            printf("Gains: Kp=%.1f Ki=%.1f Kd=%.1f\r\n", Kp, Ki, Kd);
        } else {
            printf("Usage: gains <kp> <ki> <kd>\r\n");
        }
    }
    else if (strncmp(cmd, "a", 1) == 0) {
        AS5600_PrintMagnetStatus();
        printf("linear: %.4f m (%.2f cm)  setpoint: %.4f m (%.2f cm)\r\n",
               lin_pos,           lin_pos           * 100.0f,
               ang_set * PITCH_RADIUS, ang_set * PITCH_RADIUS * 100.0f);
        printf("angle:  %.2f rad (%.1f deg)\r\n",
               ang_curr, ang_curr * 180.0f / (float)M_PI);
    }
    else if (strncmp(cmd, "r", 1) == 0) {
        current_mode = MODE_RECEIVE;
        printf("RECEIVE mode (type m to stop)\r\n");
    }
    else if (strncmp(cmd, "s", 1) == 0) {
        current_mode = MODE_STOP;
        printf("Motor STOPPED\r\n");
    }
    else if (strncmp(cmd, "m", 1) == 0) {
        current_mode = MODE_MENU;
        printf("MENU mode\r\n");
    }
    else if (strncmp(cmd, "?", 1) == 0) {
        PrintMenu();
    }
    else {
        printf("Unknown command — type ? for help\r\n");
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

    PrintMenu();
}

/* -------------------------------------------------------------------------- */
void MotorControl_Process(void)
{
    /* Handle any completed UART command */
    if (command_ready) {
        ProcessCommand(rx_buffer);
        command_ready = 0;
    }

    /* PID tick: encoder read + control update moved here to avoid blocking I2C in ISR */
    if (pid_tick) {
        pid_tick = 0;

        ang_curr = GetMotorAngle();
        lin_pos  = ang_curr * PITCH_RADIUS;

        if (current_mode == MODE_STOP) {
            ang_set  = ang_curr;
            ang_prev = ang_curr;
            MotorStop();
        } else {
            MotorDrive();   /* updates ang_prev internally via PID_Compute */
        }

        /* Print telemetry at 10 Hz (every 10th tick at 100 Hz control rate) */
        if (++print_div >= 10) {
            print_div = 0;
            switch (current_mode) {
                case MODE_RECEIVE:
                    PrintValues();
                    break;
                default:
                    break;
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
void MotorControl_TimerISR(void)
{
    /* Set flag only — encoder read and PID run in MotorControl_Process()
     * to avoid blocking I2C calls inside interrupt context. */
    pid_tick = 1;
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
    MotorCCW(20000);

    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET) {}
    HAL_Delay(20);   /* debounce */
    MotorStop();
    AS5600_ResetPosition();
    AS5600_Position_t home_pos;
    AS5600_GetPosition(&home_pos);
    home_offset = home_pos.absolute_ticks;
    ang_curr = 0.0f;
    ang_prev = 0.0f;
    ang_set  = 0.0f;
    HAL_TIM_Base_Start_IT(&htim3);   /* start PID loop */
}

/* ── Getters / setters ───────────────────────────────────────────────────── */

float    MotorControl_GetAngle(void)         { return ang_curr; }
float    MotorControl_GetSetpoint(void)      { return ang_set;  }
float    MotorControl_GetLinPos(void)        { return lin_pos;  }
float    MotorControl_GetControlSignal(void) { return control_signal; }
UARTMode MotorControl_GetMode(void)          { return current_mode; }

void MotorControl_SetSetpoint(float rad)
{
    ang_set = rad;
}

void MotorControl_SetLinSetpoint(float metres)
{
    ang_set = metres / PITCH_RADIUS;
}

void MotorControl_SetGains(float kp, float ki, float kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
}
