/**
 * @file    motor_control.c
 * @brief   Motor PID control, angle reading, and UART command processing.
 *
 * Two top-level modes selectable from the main menu:
 *   test  — open-loop motor (cw/ccw), actuators, sensor, LCD
 *   pid   — position control: home, setl, gains, r (stream), s (stop)
 */

#include "motor_control.h"
#include "as5600_position.h"
#include "actuator.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ── Private state ───────────────────────────────────────────────────────── */

static float   ang_set     = 0.0f;
static float   ang_curr    = 0.0f;
static float   ang_prev    = 0.0f;
static float   lin_pos     = 0.0f;    /* metres */
static int64_t home_offset = 0;       /* encoder ticks at home */
static uint8_t  motor_active = 0;           /* hysteresis state: 1=driving, 0=stopped */
static uint8_t  kickstart_remaining = 0;    /* ticks left in static-friction burst    */

static float Kp = 2000.0f;
static float Ki = 1000.0f;
static float Kd = 500.0f;

static float err            = 0.0f;
static float integral       = 0.0f;
static float derivative     = 0.0f;
static float control_signal = 0.0f;
static float actual_pwm     = 0.0f;
static float P = 0.0f, I = 0.0f, D = 0.0f;

/* Motor direction state — used to insert dead time only on actual reversals */
typedef enum { DIR_STOP, DIR_CW, DIR_CCW } MotorDir;
static MotorDir current_dir = DIR_STOP;

/* Song sequencer ----------------------------------------------------------- */
typedef struct { float pos_cm; uint32_t dur_ms; } SongNote;

/* Twinkle Twinkle Little Star — white-key positions at 2.3 cm per key,
 * C4 = 0 cm, D=2.3, E=4.6, F=6.9, G=9.2, A=11.5
 * Quarter note = 500 ms (120 BPM), half note = 1000 ms                    */
static const SongNote song_notes[] = {
    /* C  C  G  G  A  A  G(half)  */
    { 0.0f,  500}, { 0.0f,  500}, { 9.2f,  500}, { 9.2f,  500},
    {11.5f,  500}, {11.5f,  500}, { 9.2f, 1000},
    /* F  F  E  E  D  D  C(half)  */
    { 6.9f,  500}, { 6.9f,  500}, { 4.6f,  500}, { 4.6f,  500},
    { 2.3f,  500}, { 2.3f,  500}, { 0.0f, 1000},
    /* G  G  F  F  E  E  D(half)  */
    { 9.2f,  500}, { 9.2f,  500}, { 6.9f,  500}, { 6.9f,  500},
    { 4.6f,  500}, { 4.6f,  500}, { 2.3f, 1000},
    /* G  G  F  F  E  E  D(half)  */
    { 9.2f,  500}, { 9.2f,  500}, { 6.9f,  500}, { 6.9f,  500},
    { 4.6f,  500}, { 4.6f,  500}, { 2.3f, 1000},
    /* C  C  G  G  A  A  G(half)  */
    { 0.0f,  500}, { 0.0f,  500}, { 9.2f,  500}, { 9.2f,  500},
    {11.5f,  500}, {11.5f,  500}, { 9.2f, 1000},
    /* F  F  E  E  D  D  C(half)  */
    { 6.9f,  500}, { 6.9f,  500}, { 4.6f,  500}, { 4.6f,  500},
    { 2.3f,  500}, { 2.3f,  500}, { 0.0f, 1000},
};
#define SONG_NUM_NOTES  (sizeof(song_notes) / sizeof(song_notes[0]))

static uint8_t  song_playing = 0;
static uint32_t song_step    = 0;
static uint32_t song_next_ms = 0;

/* Switching test state */
static MotorDir switch_dir     = DIR_CW;
static uint32_t switch_next_ms = 0;

static uint8_t  rx_char;
static char     rx_buffer[RX_BUFFER_SIZE];
static uint8_t  rx_index = 0;
static volatile uint8_t command_ready = 0;
static UARTMode current_mode = MODE_MAIN_MENU;

static volatile uint8_t pid_tick  = 0;
static uint8_t           print_div = 0;

static const GPIO_Pin_t fingers[5] = {
    {GPIOC, SLND1_Pin},
    {GPIOC, SLND2_Pin},
    {GPIOC, SLND3_Pin},
    {GPIOC, SLND4_Pin},
    {GPIOB, SLND5_Pin},
};
static uint8_t selected_fingers = 0;
static uint8_t actuate_state    = 0; /* 0=all off, 1-5=each finger, 6=all on */

/* ── Printf redirect ─────────────────────────────────────────────────────── */

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* ── Menu printers ───────────────────────────────────────────────────────── */

static void PrintMainMenu(void)
{
    printf("\r\n=== RoboMaestro ===\r\n");
    printf("  test  - Open-loop motor, actuators, sensor, LCD\r\n");
    printf("  pid   - Position control (set, stream, stop)\r\n");
    printf("  help  - Show this menu\r\n");
    printf("===================\r\n\r\n");
}

static void PrintTestMenu(void)
{
    printf("\r\n=== Test Mode ===\r\n");
    printf("  cw                   - Open-loop CW at max speed\r\n");
    printf("  ccw                  - Open-loop CCW at max speed\r\n");
    printf("  s                    - Stop motor\r\n");
    printf("  fingers <n> [on|off] - e.g. 'fingers 1 2 3 on'\r\n");
    printf("  actuate              - Cycle solenoids with blue button (s to stop)\r\n");
    printf("  sensor               - One-shot AS5600 read\r\n");
    printf("  help                 - Show this menu\r\n");
    printf("  back                 - Return to main menu\r\n");
    printf("=================\r\n\r\n");
}

static void PrintPIDMenu(void)
{
    printf("\r\n=== PID Mode ===\r\n");
    printf("  home                  - Spin CCW until button, record home\r\n");
    printf("  setl <cm>             - Set linear position target (cm)\r\n");
    printf("  gains <kp> <ki> <kd>  - Update PID gains at runtime\r\n");
    printf("  a                     - Print current position and setpoint\r\n");
    printf("  pid                   - Enable PID tracking\r\n");
    printf("  r                     - Enable PID + stream telemetry at 10 Hz\r\n");
    printf("  s                     - Stop motor (freeze setpoint)\r\n");
    printf("  m                     - Disable PID, motor stops\r\n");
    printf("  help                  - Show this menu\r\n");
    printf("  back                  - Return to main menu\r\n");
    printf("================\r\n\r\n");
}

/* ── Motor primitives ────────────────────────────────────────────────────── */

static void MotorStop(void)
{
    /* Kill both PWMs first, then deassert both enables (active-low → SET = OFF) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(ENBL1_GPIO_Port, ENBL1_Pin, GPIO_PIN_SET);  /* N_EN_P1=1, P1 OFF */
    HAL_GPIO_WritePin(ENBL2_GPIO_Port, ENBL2_Pin, GPIO_PIN_SET);  /* N_EN_P2=1, P2 OFF */
    current_dir = DIR_STOP;
}

/* CW: P1 ON (ENBL1 LOW), N2 driven by CH2.  P2 OFF, CH1=0. */
static void MotorCW(uint16_t duty)
{
    if (current_dir == DIR_CCW) {
        /* Safe reversal: kill PWMs, disable both P-sides, wait dead time */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        HAL_GPIO_WritePin(ENBL1_GPIO_Port, ENBL1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ENBL2_GPIO_Port, ENBL2_Pin, GPIO_PIN_SET);
        HAL_Delay(MOTOR_DEADTIME_MS);
    }
    /* Apply CW: CH1=0, P2 OFF, then P1 ON, then CH2=duty */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);              /* PWM_1 = 0 (N1 off) */
    HAL_GPIO_WritePin(ENBL2_GPIO_Port, ENBL2_Pin, GPIO_PIN_SET);  /* N_EN_P2=1, P2 OFF  */
    HAL_GPIO_WritePin(ENBL1_GPIO_Port, ENBL1_Pin, GPIO_PIN_RESET);/* N_EN_P1=0, P1 ON   */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);           /* PWM_2 = duty (N2)  */
    current_dir = DIR_CW;
}

/* CCW: P2 ON (ENBL2 LOW), N1 driven by CH1.  P1 OFF, CH2=0. */
static void MotorCCW(uint16_t duty)
{
    if (current_dir == DIR_CW) {
        /* Safe reversal: kill PWMs, disable both P-sides, wait dead time */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        HAL_GPIO_WritePin(ENBL1_GPIO_Port, ENBL1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ENBL2_GPIO_Port, ENBL2_Pin, GPIO_PIN_SET);
        HAL_Delay(MOTOR_DEADTIME_MS);
    }
    /* Apply CCW: CH2=0, P1 OFF, then P2 ON, then CH1=duty */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);              /* PWM_2 = 0 (N2 off) */
    HAL_GPIO_WritePin(ENBL1_GPIO_Port, ENBL1_Pin, GPIO_PIN_SET);  /* N_EN_P1=1, P1 OFF  */
    HAL_GPIO_WritePin(ENBL2_GPIO_Port, ENBL2_Pin, GPIO_PIN_RESET);/* N_EN_P2=0, P2 ON   */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);           /* PWM_1 = duty (N1)  */
    current_dir = DIR_CCW;
}

/* ── Encoder ─────────────────────────────────────────────────────────────── */

static float GetMotorAngle(void)
{
    AS5600_Update();
    AS5600_Position_t pos;
    AS5600_GetPosition(&pos);
    return -((pos.absolute_ticks - home_offset) / 4096.0f) * 2.0f * (float)M_PI;
}

/* ── PID ─────────────────────────────────────────────────────────────────── */

static void IRRFilterD(float *sigD)
{
    static float prev = 0.0f;
    *sigD = BETAD * prev + (1.0f - BETAD) * (*sigD);
    prev  = *sigD;
}

static float PID_Compute(void)
{
    err = ang_set - ang_curr;

    if (!motor_active && fabsf(err) < ERROR_THRESHOLD_ON) {
        err      = 0.0f;
        integral = 0.0f;
    }

    integral  += err * DT;
    derivative = (ang_curr - ang_prev) / DT;
    IRRFilterD(&derivative);
    ang_prev = ang_curr;

    if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
    if (integral <  INTEGRAL_MIN) integral =  INTEGRAL_MIN;

    P = Kp * err;
    I = Ki * integral;
    D = -Kd * derivative;
    control_signal = P + I + D;

    if (control_signal >  PID_ABS_MAX_OUTPUT) control_signal =  PID_ABS_MAX_OUTPUT;
    if (control_signal < -PID_ABS_MAX_OUTPUT) control_signal = -PID_ABS_MAX_OUTPUT;

    return control_signal;
}

static void MotorDrive(void)
{
    float sig = PID_Compute();

    /* Hysteresis: stop when inside OFF band, only restart when outside ON band */
    if (motor_active && fabsf(err) < ERROR_THRESHOLD_OFF) {
        motor_active = 0;
        kickstart_remaining = 0;
        actual_pwm = 0.0f;
        MotorStop();
        return;
    }
    if (!motor_active && fabsf(err) < ERROR_THRESHOLD_ON) {
        actual_pwm = 0.0f;
        return;
    }
    if (!motor_active)
        kickstart_remaining = KICKSTART_TICKS;
    motor_active = 1;

    /* Kickstart: override PID with max output for the first N ticks to break stiction */
    if (kickstart_remaining > 0) {
        kickstart_remaining--;
        sig = (sig >= 0.0f) ? PID_ABS_MAX_OUTPUT : -PID_ABS_MAX_OUTPUT;
    }

    if (sig > 0.0f) {
        if (sig < PID_ABS_MIN_OUTPUT) sig = PID_ABS_MIN_OUTPUT;
        actual_pwm = sig;
        MotorCW((uint16_t)sig);
    } else {
        if (-sig < PID_ABS_MIN_OUTPUT) sig = -PID_ABS_MIN_OUTPUT;
        actual_pwm = sig;
        MotorCCW((uint16_t)(-sig));
    }
}

static void PrintPIDValues(void)
{
    printf("lin: %.4f m  err: %.4f rad  pid: %.0f  pwm: %.0f (%.1f%%)\r\n",
           lin_pos, err, control_signal, actual_pwm,
           (actual_pwm < 0.0f ? -actual_pwm : actual_pwm) / PID_ABS_MAX_OUTPUT * 100.0f);
}

/* ── Command processing ──────────────────────────────────────────────────── */

static void ProcessMainMenu(char *cmd)
{
    if (strncmp(cmd, "test", 4) == 0) {
        current_mode = MODE_TEST_IDLE;
        PrintTestMenu();
    } else if (strncmp(cmd, "pid", 3) == 0) {
        current_mode = MODE_PID_IDLE;
        PrintPIDMenu();
    } else if (strncmp(cmd, "help", 4) == 0 || cmd[0] == '?') {
        PrintMainMenu();
    } else {
        printf("Type 'test', 'pid', or 'help'\r\n");
    }
}

/* -------------------------------------------------------------------------- */
static void ProcessTestMode(char *cmd)
{
    /* Waiting for on/off after "fingers" */
    if (current_mode == MODE_TEST_FINGER_ONOFF) {
        if (strncmp(cmd, "on", 2) == 0) {
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_SET);
            printf("Fingers ON\r\n");
        } else if (strncmp(cmd, "off", 3) == 0) {
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
            printf("Fingers OFF\r\n");
        } else {
            printf("Type 'on' or 'off'\r\n");
            return;
        }
        current_mode = MODE_TEST_IDLE;
        return;
    }

    if (strncmp(cmd, "cw", 2) == 0 && (cmd[2] == '\0' || cmd[2] == '\r')) {
        current_mode = MODE_TEST_CW;
        printf("Motor CW — type 's' to stop\r\n");
    }
    else if (strncmp(cmd, "ccw", 3) == 0 && (cmd[3] == '\0' || cmd[3] == '\r')) {
        current_mode = MODE_TEST_CCW;
        printf("Motor CCW — type 's' to stop\r\n");
    }
    else if (cmd[0] == 's' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        MotorStop();
        if (current_mode == MODE_TEST_ACTUATE) {
            for (int i = 0; i < 5; i++)
                HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
            printf("Actuate stopped — all fingers OFF\r\n");
        } else {
            printf("Motor stopped\r\n");
        }
        current_mode = MODE_TEST_IDLE;
    }
    else if (strncmp(cmd, "sensor", 6) == 0) {
        AS5600_Update();
        AS5600_Position_t pos;
        AS5600_GetPosition(&pos);
        float raw_deg = (pos.absolute_ticks / 4096.0f) * 360.0f;
        printf("AS5600: raw=%u  turns=%ld  abs_ticks=%lld  raw_angle=%.1f deg\r\n",
               (unsigned)pos.raw_angle,
               (long)pos.turns,
               (long long)pos.absolute_ticks,
               raw_deg);
        printf("  homed: angle=%.4f rad  linear=%.4f m (%.2f cm)\r\n",
               ang_curr, lin_pos, lin_pos * 100.0f);
        AS5600_PrintMagnetStatus();
    }
    else if (strncmp(cmd, "fingers", 7) == 0) {
        char *p = cmd + 7;
        char *endptr;
        selected_fingers = 0;
        int8_t state = -1;

        while (*p) {
            while (*p == ' ') p++;
            if (*p == '\0') break;
            long n = strtol(p, &endptr, 10);
            if (endptr != p) {
                if (n >= 1 && n <= 5) selected_fingers |= (1 << (n - 1));
                p = endptr;
            } else {
                if (strncmp(p, "on",  2) == 0 && (p[2] == ' ' || p[2] == '\0')) { state = 1; p += 2; }
                else if (strncmp(p, "off", 3) == 0 && (p[3] == ' ' || p[3] == '\0')) { state = 0; p += 3; }
                else { while (*p && *p != ' ') p++; }
            }
        }

        if (selected_fingers == 0) {
            printf("No valid fingers (1-5). Usage: fingers 1 2 3 on\r\n");
        } else if (state == -1) {
            printf("Selected fingers:");
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i)) printf(" %d", i + 1);
            printf("\r\nTurn on or off? (on/off): ");
            current_mode = MODE_TEST_FINGER_ONOFF;
        } else {
            GPIO_PinState pin_state = (state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, pin_state);
            printf("Fingers");
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i)) printf(" %d", i + 1);
            printf(" %s\r\n", (state == 1) ? "ON" : "OFF");
        }
    }
    else if (strncmp(cmd, "actuate", 7) == 0 && (cmd[7] == '\0' || cmd[7] == '\r')) {
        actuate_state = 0;
        for (int i = 0; i < 5; i++)
            HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
        current_mode = MODE_TEST_ACTUATE;
        printf("Actuate mode: press blue button to cycle (s to stop)\r\n");
        printf("  0=all OFF  1-5=each finger  6=all ON\r\n");
        printf("State 0: all OFF\r\n");
    }
    else if (strncmp(cmd, "help", 4) == 0 || cmd[0] == '?') {
        PrintTestMenu();
    }
    else {
        /* Delegate to main.c for lcd or other commands */
        MotorControl_ExtCommand(cmd);
    }
}

/* -------------------------------------------------------------------------- */
static void ProcessPIDMode(char *cmd)
{
    /* 'm' → disable PID, motor stops, stay in PID idle */
    if (cmd[0] == 'm' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        MotorStop();
        integral = 0.0f;
        current_mode = MODE_PID_IDLE;
        printf("PID disabled\r\n");
        return;
    }

    if (strncmp(cmd, "home", 4) == 0) {
        printf("Homing: spinning CCW, press blue button...\r\n");
        MotorControl_Home();
        printf("Homed at 0. Type 'pid' or 'r' to start tracking.\r\n");
    }
    else if (strncmp(cmd, "setl ", 5) == 0) {
        float cm = atof(&cmd[5]);
        ang_set = (cm / 100.0f) / PITCH_RADIUS;
        printf("Setpoint: %.2f cm  (%.4f rad)\r\n", cm, ang_set);
    }
    else if (strncmp(cmd, "gains ", 6) == 0) {
        float kp, ki, kd;
        if (sscanf(&cmd[6], "%f %f %f", &kp, &ki, &kd) == 3) {
            Kp = kp; Ki = ki; Kd = kd;
            printf("Gains: Kp=%.1f  Ki=%.1f  Kd=%.1f\r\n", Kp, Ki, Kd);
        } else {
            printf("Usage: gains <kp> <ki> <kd>\r\n");
        }
    }
    else if (cmd[0] == 'a' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        AS5600_PrintMagnetStatus();
        printf("linear: %.4f m (%.2f cm)  setpoint: %.4f m (%.2f cm)\r\n",
               lin_pos,                lin_pos                * 100.0f,
               ang_set * PITCH_RADIUS, ang_set * PITCH_RADIUS * 100.0f);
        printf("angle:  %.4f rad (%.1f deg)\r\n",
               ang_curr, ang_curr * 180.0f / (float)M_PI);
    }
    else if (strncmp(cmd, "pid", 3) == 0 && (cmd[3] == '\0' || cmd[3] == '\r')) {
        integral = 0.0f;
        current_mode = MODE_PID_ACTIVE;
        printf("PID active — tracking setpoint (type 's' to stop, 'm' to disable)\r\n");
    }
    else if (cmd[0] == 'r' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        integral = 0.0f;
        current_mode = MODE_PID_RECEIVE;
        printf("PID active + streaming at 10 Hz (type 'm' to stop)\r\n");
    }
    else if (cmd[0] == 's' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        current_mode = MODE_PID_STOP;
        printf("Motor stopped (setpoint frozen)\r\n");
    }
    else if (strncmp(cmd, "help", 4) == 0 || cmd[0] == '?') {
        PrintPIDMenu();
    }
    else {
        printf("Unknown PID command (type 'help')\r\n");
    }
}

/* -------------------------------------------------------------------------- */
static void ProcessCommand(char *cmd)
{
    /* 'back' returns to the main menu from anywhere */
    if (strncmp(cmd, "back", 4) == 0) {
        MotorStop();
        current_mode = MODE_MAIN_MENU;
        PrintMainMenu();
        return;
    }

    if (current_mode == MODE_MAIN_MENU) {
        ProcessMainMenu(cmd);
    } else if (IS_TEST_MODE(current_mode)) {
        ProcessTestMode(cmd);
    } else if (IS_PID_MODE(current_mode)) {
        ProcessPIDMode(cmd);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void MotorControl_Init(void)
{
    /* Start both PWM channels before asserting any enables */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /* Ensure enables start HIGH (active-low → HIGH = P-side OFF) */
    HAL_GPIO_WritePin(ENBL1_GPIO_Port, ENBL1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ENBL2_GPIO_Port, ENBL2_Pin, GPIO_PIN_SET);

    MotorStop();

    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    PrintMainMenu();
}

/* -------------------------------------------------------------------------- */
void MotorControl_Process(void)
{
    if (command_ready) {
        ProcessCommand(rx_buffer);
        command_ready = 0;
    }

    /* Actuate mode: debounced blue-button (PC13, active-low) cycles solenoids */
    if (current_mode == MODE_TEST_ACTUATE) {
        static uint8_t  btn_prev   = 1;
        static uint8_t  btn_stable = 1;
        static uint32_t btn_time   = 0;

        uint8_t btn_now = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
        if (btn_now != btn_prev) {
            btn_time = HAL_GetTick();
            btn_prev = btn_now;
        }
        if ((HAL_GetTick() - btn_time) > 20 && btn_now != btn_stable) {
            btn_stable = btn_now;
            if (btn_stable == GPIO_PIN_RESET) {
                /* Falling edge — button pressed */
                actuate_state++;
                if (actuate_state > 6) actuate_state = 0;

                for (int i = 0; i < 5; i++)
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);

                if (actuate_state == 0) {
                    printf("State 0: all OFF\r\n");
                } else if (actuate_state <= 5) {
                    HAL_GPIO_WritePin(fingers[actuate_state - 1].port,
                                     fingers[actuate_state - 1].pin, GPIO_PIN_SET);
                    printf("State %d: finger %d ON\r\n", actuate_state, actuate_state);
                } else {
                    for (int i = 0; i < 5; i++)
                        HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_SET);
                    printf("State 6: all ON\r\n");
                }
            }
        }
    }

    if (pid_tick) {
        pid_tick = 0;

        ang_curr = GetMotorAngle();
        lin_pos  = ang_curr * PITCH_RADIUS;

        switch (current_mode) {
            case MODE_PID_ACTIVE:
            case MODE_PID_RECEIVE:
                MotorDrive();
                break;

            case MODE_PID_STOP:
                ang_set    = ang_curr;
                ang_prev   = ang_curr;
                actual_pwm = 0.0f;
                MotorStop();
                break;

            case MODE_TEST_CW:
                MotorCW(65535);
                ang_prev = ang_curr;
                break;

            case MODE_TEST_CCW:
                MotorCCW(65535);
                ang_prev = ang_curr;
                break;

            case MODE_TEST_SWITCH:
                if (switch_dir == DIR_CW)
                    MotorCW(65535);
                else
                    MotorCCW(65535);
                ang_prev = ang_curr;
                break;

            default:
                MotorStop();
                ang_prev = ang_curr;
                break;
        }

        /* Print telemetry at 10 Hz */
        if (++print_div >= 10) {
            print_div = 0;
            if (current_mode == MODE_PID_RECEIVE)
                PrintPIDValues();
        }
    }

    /* Switching test: toggle direction every MOTOR_SWITCH_INTERVAL_MS */
    if (current_mode == MODE_TEST_SWITCH) {
        uint32_t now = HAL_GetTick();
        if (now >= switch_next_ms) {
            switch_dir     = (switch_dir == DIR_CW) ? DIR_CCW : DIR_CW;
            switch_next_ms = now + MOTOR_SWITCH_INTERVAL_MS;
            if (switch_dir == DIR_CW)
                MotorCW(65535);
            else
                MotorCCW(65535);
        }
    }

    /* Song sequencer: advance to the next note when the dwell time has elapsed */
    if (song_playing) {
        uint32_t now = HAL_GetTick();
        if (now >= song_next_ms) {
            if (song_step >= SONG_NUM_NOTES) {
                song_playing = 0;
            } else {
                float cm = song_notes[song_step].pos_cm;
                ang_set      = (cm / 100.0f) / PITCH_RADIUS;
                song_next_ms = now + song_notes[song_step].dur_ms;
                song_step++;
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
void MotorControl_TimerISR(void)
{
    pid_tick = 1;
}

/* -------------------------------------------------------------------------- */
void MotorControl_UartISR(void)
{
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
            rx_index = 0;
        }
    }

    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
}

/* -------------------------------------------------------------------------- */
void MotorControl_Home(void)
{
    MotorCCW(65535);
    HAL_Delay(1000);
    MotorCCW(25535);
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET) {}
    HAL_Delay(20);
    MotorStop();

    AS5600_Update();
    AS5600_Position_t home_pos;
    AS5600_GetPosition(&home_pos);
    home_offset = home_pos.absolute_ticks;

    ang_curr = 0.0f;
    ang_prev = 0.0f;
    ang_set  = 0.0f;
    lin_pos  = 0.0f;
    integral = 0.0f;

    HAL_TIM_Base_Start_IT(&htim3);
}

/* ── Getters / setters ───────────────────────────────────────────────────── */

float    MotorControl_GetAngle(void)         { return ang_curr; }
float    MotorControl_GetLinPos(void)        { return lin_pos;  }
float    MotorControl_ReadLinPos(void)
{
    float angle = GetMotorAngle();
    return angle * PITCH_RADIUS;
}
float    MotorControl_GetSetpoint(void)      { return ang_set;  }
float    MotorControl_GetControlSignal(void) { return control_signal; }
UARTMode MotorControl_GetMode(void)          { return current_mode; }

void MotorControl_SetSetpoint(float rad)     { ang_set = rad; }
void MotorControl_SetLinSetpoint(float m)    { ang_set = m / PITCH_RADIUS; }
void MotorControl_SetGains(float kp, float ki, float kd) { Kp = kp; Ki = ki; Kd = kd; }

void MotorControl_SetTestStop(void)   { MotorStop(); current_mode = MODE_TEST_IDLE; }
void MotorControl_SetTestCW(void)     { current_mode = MODE_TEST_CW;  MotorCW(65535); }
void MotorControl_SetTestCCW(void)    { current_mode = MODE_TEST_CCW; MotorCCW(65535); }
void MotorControl_SetTestSwitch(void)
{
    switch_dir     = DIR_CW;
    switch_next_ms = HAL_GetTick() + MOTOR_SWITCH_INTERVAL_MS;
    current_mode   = MODE_TEST_SWITCH;
    MotorCW(65535);   /* start immediately without waiting for first tick */
}

void MotorControl_PlaySong(void)
{
    song_step    = 0;
    song_next_ms = HAL_GetTick();
    song_playing = 1;
    integral     = 0.0f;
    current_mode = MODE_PID_ACTIVE;
}

/* -------------------------------------------------------------------------- */
__attribute__((weak)) void MotorControl_ExtCommand(const char *cmd)
{
    (void)cmd;
    printf("Unknown command (type 'help')\r\n");
}
