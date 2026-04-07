/**
 * @file    motor_control.c
 * @brief   Motor PID control and angle reading.
 *
 * ENBL1/ENBL2 are active-low N_EN signals for the P-side MOSFETs:
 *   CW:   ENBL1=LOW  (P1 ON), ENBL2=HIGH (P2 OFF), CH1=0,    CH2=duty
 *   CCW:  ENBL1=HIGH (P1 OFF),ENBL2=LOW  (P2 ON),  CH1=duty, CH2=0
 *   Stop: ENBL1=HIGH, ENBL2=HIGH, CH1=0, CH2=0
 *
 * NEVER assert both enables LOW simultaneously (P1+P2 ON = shoot-through).
 * NEVER assert an enable LOW while the same-side PWM is non-zero (half-bridge short).
 *
 */

#include "motor_control.h"
#include "as5600_position.h"
#include "actuator.h"
#include "stm32f4xx_hal.h"

#include <math.h>

/* ── Private state ───────────────────────────────────────────────────────── */

static float   ang_set     = 0.0f;
static float   ang_curr    = 0.0f;
static float   ang_prev    = 0.0f;
static float   lin_pos     = 0.0f;    /* metres */
static int64_t home_offset = 0;       /* encoder ticks at home */
static uint8_t  motor_active = 0;           /* hysteresis state: 1=driving, 0=stopped */
static uint8_t  kickstart_remaining = 0;    /* ticks left in static-friction burst    */

static float Kp = 500.0f;
static float Ki = 100.0f;
static float Kd = 100.0f;

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
typedef struct {
    float    pos_cm;       /* carriage target position (cm)                  */
    uint32_t dur_ms;       /* note duration — time before advancing to next  */
    uint8_t  fingers;      /* bitmask: bit0=finger1 … bit4=finger5           */
    uint32_t finger_dur_ms; /* ms the fingers stay pressed                   */
} SongNote;

/* Twinkle Twinkle Little Star — white-key positions at 2.3 cm per key,
 * C4 = 0.5 cm (offset from end stop), D=2.3, E=4.6, F=6.9, G=9.2, A=11.5
 * Quarter note = 1000 ms (60 BPM), half note = 2000 ms
 *
 * Fields: { pos_cm, dur_ms, fingers, finger_dur_ms }
 *   fingers       — bitmask, e.g. 0x01=finger1, 0x03=fingers1+2, 0=no press
 *   finger_dur_ms — how long the finger stays pressed (fires immediately on note start) */
static const SongNote song_notes[] = {
    /* C  C  G  G  A  A  G(half)  */
    { 0.0f,  1000, 0x01, 500}, { 10.0f,  2000, 0x01, 500}, { 20.0f,  2000, 0x01, 500},
    {30.0f,  2000, 0x01, 500}, {10.0f,  2000, 0x01, 500}, { 7.0f, 2000, 0x01, 500},
    /* F  F  E  E  D  D  C(half)  */
    { 10.0f,  1000, 0x01, 500}, { 12.0f,  1000, 0x01, 500},
    { 14.0f,  1000, 0x01, 500}, { 16.0f,  1000, 0x01, 500},
    { 18.0f,  1000, 0x01, 500}, { 20.0f,  1000, 0x01, 500}, { 22.0f, 2000, 0x01, 500},
    /* G  G  F  F  E  E  D(half)  */
    { 9.0f,  1000, 0x01, 500}, { 9.0f,  1000, 0x01, 500},
    { 6.0f,  1000, 0x01, 500}, { 6.0f,  1000, 0x01, 500},
    { 2.0f,  1000, 0x01, 500}, { 2.0f,  1000, 0x01, 500}, { 2.0f, 2000, 0x01, 1000},
};
#define SONG_NUM_NOTES  (sizeof(song_notes) / sizeof(song_notes[0]))

static uint8_t  song_playing = 0;
static uint32_t song_step    = 0;
static uint32_t song_next_ms = 0;

/* Per-note finger timing */
static uint8_t  song_fingers_waiting  = 0;  /* waiting for motor to settle before pressing */
static uint32_t song_finger_dur_saved = 0;  /* finger_dur_ms saved until motor settles     */
static uint8_t  song_fingers_pressed  = 0;  /* fingers currently held down                 */
static uint32_t song_finger_off_ms    = 0;  /* absolute tick to release fingers            */

/* Switching test state */
static MotorDir switch_dir     = DIR_CW;
static uint32_t switch_next_ms = 0;

static UARTMode current_mode = MODE_MAIN_MENU;

static volatile uint8_t pid_tick = 0;

static const GPIO_Pin_t fingers[5] = {
    {GPIOC, SLND1_Pin},
    {GPIOC, SLND2_Pin},
    {GPIOC, SLND3_Pin},
    {GPIOC, SLND4_Pin},
    {GPIOB, SLND5_Pin},
};
static uint8_t selected_fingers = 0;
static uint8_t actuate_state    = 0; /* 0=all off, 1-5=each finger, 6=all on */


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
}

/* -------------------------------------------------------------------------- */
void MotorControl_Process(void)
{
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
                actuate_state++;
                if (actuate_state > 6) actuate_state = 0;

                for (int i = 0; i < 5; i++)
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);

                if (actuate_state == 0) {
                    /* all off */
                } else if (actuate_state <= 5) {
                    HAL_GPIO_WritePin(fingers[actuate_state - 1].port,
                                     fingers[actuate_state - 1].pin, GPIO_PIN_SET);
                } else {
                    for (int i = 0; i < 5; i++)
                        HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_SET);
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
                MotorCW(8999);
                ang_prev = ang_curr;
                break;

            case MODE_TEST_CCW:
                MotorCCW(8999);
                ang_prev = ang_curr;
                break;

            case MODE_TEST_SWITCH:
                if (switch_dir == DIR_CW)
                    MotorCW(4500);
                else
                    MotorCCW(4500);
                ang_prev = ang_curr;
                break;

            default:
                MotorStop();
                ang_prev = ang_curr;
                break;
        }
    }

    /* Switching test: toggle direction every MOTOR_SWITCH_INTERVAL_MS */
    if (current_mode == MODE_TEST_SWITCH) {
        uint32_t now = HAL_GetTick();
        if (now >= switch_next_ms) {
            switch_dir     = (switch_dir == DIR_CW) ? DIR_CCW : DIR_CW;
            switch_next_ms = now + MOTOR_SWITCH_INTERVAL_MS;
            if (switch_dir == DIR_CW)
                MotorCW(4500);
            else
                MotorCCW(4500);
        }
    }

    /* Song sequencer: advance to the next note when the dwell time has elapsed */
    if (song_playing) {
        uint32_t now = HAL_GetTick();
        if (now >= song_next_ms) {
            if (song_step >= SONG_NUM_NOTES) {
                song_playing = 0;
                song_fingers_waiting = 0;
                /* Release any fingers still held at song end */
                if (song_fingers_pressed) {
                    for (int i = 0; i < 5; i++)
                        HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
                    song_fingers_pressed = 0;
                }
                MotorStop();
                current_mode = MODE_PID_IDLE;
            } else {
                const SongNote *n = &song_notes[song_step];
                ang_set      = (n->pos_cm / 100.0f) / PITCH_RADIUS;
                song_next_ms = now + n->dur_ms;

                /* Release any fingers still held from the previous note */
                if (song_fingers_pressed) {
                    for (int i = 0; i < 5; i++)
                        if (song_fingers_pressed & (1 << i))
                            HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
                    song_fingers_pressed = 0;
                }
                /* Arm fingers — will press once motor settles at new position */
                song_fingers_waiting  = n->fingers;
                song_finger_dur_saved = n->finger_dur_ms;

                song_step++;
            }
        } 

        /* Press fingers once motor has settled at the target position.
         * Use ang_set - ang_curr directly so a freshly-armed note never fires
         * immediately due to stale err from the previous note's settled state. */
        if (song_fingers_waiting && !song_fingers_pressed &&
            fabsf(ang_set - ang_curr) < ERROR_THRESHOLD_OFF) {
            for (int i = 0; i < 5; i++)
                if (song_fingers_waiting & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_SET);
            song_fingers_pressed  = song_fingers_waiting;
            song_fingers_waiting  = 0;
            song_finger_off_ms    = now + song_finger_dur_saved;
        }

        /* Release fingers when finger_dur_ms has elapsed */
        if (song_fingers_pressed && now >= song_finger_off_ms) {
            for (int i = 0; i < 5; i++)
                if (song_fingers_pressed & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
            song_fingers_pressed = 0;
        }
    }
}

/* -------------------------------------------------------------------------- */
void MotorControl_TimerISR(void)
{
    pid_tick = 1;
}

/* -------------------------------------------------------------------------- */
void MotorControl_Home(void)
{
    // MotorCCW(8999);
    // HAL_Delay(1000);
    // MotorCCW(4500);
    // while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_RESET) {}
    // HAL_Delay(20);
    // MotorStop();

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
    MotorCW(65535);
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
}
