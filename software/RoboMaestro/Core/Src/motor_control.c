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
#include "stm32f4xx_hal_gpio.h"

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

static float Kp = 200.0f;
static float Ki = 10.0f;
static float Kd = 20.0f;

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
    MusicalNote note;           /* note name — looked up in note_positions_cm */
    uint32_t    dur_ms;         /* time before advancing to the next note     */
    uint8_t     fingers;        /* bitmask: bit0=finger1 … bit4=finger5       */
    uint32_t    finger_dur_ms;  /* ms the fingers stay pressed                */
} SongNote;

/* ── NOTE POSITION TABLE ────────────────────────────────────────────────────
 * One value per white key, in enum order (C1, D1, E1, F1, G1, A1, B1, C2 …).
 * Edit these cm values to match your instrument's physical fret positions.
 * ────────────────────────────────────────────────────────────────────────── */
static const float note_positions_cm[NOTE_COUNT] = {
    /* Octave 1 (base positions from F) */
     0.0f,   /* F1 */
     2.6f,   /* G1 */
     4.9f,   /* A1 */
     7.2f,   /* B1 */
     9.6f,   /* C2 */
    11.6f,   /* D2 */
    14.2f,   /* E2 */
    /* Octave 2 (+16 cm) */
    16.5f,   /* F2 */   
    18.7f,   /* G2 */
    22.7f,   /* A2 */
    25.0f,   /* B2 */
    26.4f,   /* C3 */
    28.2f,   /* D3 */
    28.5f,   /* E3 */
    /* Octave 3 (+32 cm) */
    30.8f,   /* F3 */
    33.1f,   /* G3 */
    35.4f,   /* A3 */
    37.7f,   /* B3 */
    40.0f,   /* C4 */
    42.3f,   /* D4 */
    44.6f,   /* E4 */
    /* Octave 4 (+48 cm, up to B) */
    46.9f,   /* F4 */
    49.2f,   /* G4 */
    51.5f,   /* A4 */
    53.8f,   /* B4 */
};

/* ── SONG 0 (test song — played by the SONG button on the Test screen) ─────
 * Format: { NOTE_XX, dur_ms, finger_mask, finger_dur_ms }
 * ────────────────────────────────────────────────────────────────────────── */
static const SongNote song0_notes[] = {
    /* ascending scale — every note, finger 1 */
    { NOTE_F1, 800, 0x01, 400 },
    { NOTE_G1, 800, 0x01, 400 },
    { NOTE_A1, 800, 0x01, 400 },
    { NOTE_B1, 800, 0x01, 400 },
    { NOTE_C2, 800, 0x01, 400 },
    { NOTE_D2, 800, 0x01, 400 },
    { NOTE_E2, 800, 0x01, 400 },
    { NOTE_F2, 800, 0x01, 400 },
    { NOTE_G2, 800, 0x01, 400 },
    { NOTE_A2, 800, 0x01, 400 },
    { NOTE_B2, 800, 0x01, 400 },
    { NOTE_C3, 800, 0x01, 400 },
    { NOTE_D3, 800, 0x01, 400 },
    { NOTE_E3, 1600, 0x01, 400 },
    { NOTE_F3, 800, 0x01, 400 },
    { NOTE_G3, 800, 0x01, 400 },
    { NOTE_A3, 800, 0x01, 400 },
    { NOTE_B3, 800, 0x01, 400 },
    { NOTE_C4, 800, 0x01, 400 },
    { NOTE_D4, 800, 0x01, 400 },
    { NOTE_E4, 800, 0x01, 400 },
    { NOTE_F4, 800, 0x01, 400 },
    {NOTE_A2, 3000, 0x00, 1000},
};

/* ── SONG 1 ──────────────────────────────────────────────────────────────────
 * Format: { NOTE_XX, dur_ms, finger_mask, finger_dur_ms }
 * ────────────────────────────────────────────────────────────────────────── */
static const SongNote song1_notes[] = {
    {NOTE_E2, 3000, 0x00, 1000},
    {NOTE_E2, 1000, 0x01, 500},
    {NOTE_D2, 1000, 0x01, 500},
    {NOTE_C2, 2000, 0x01, 1000},
    {NOTE_E2, 1000, 0x01, 500},
    {NOTE_D2, 1000, 0x01, 500},
    {NOTE_C2, 2000, 0x01, 1000},
    {NOTE_C2, 500, 0x01, 100},
    {NOTE_C2, 500, 0x01, 100},
    {NOTE_C2, 500, 0x01, 100},
    {NOTE_C2, 500, 0x01, 100},
    {NOTE_D2, 500, 0x01, 100},
    {NOTE_D2, 500, 0x01, 100},
    {NOTE_D2, 500, 0x01, 100},
    {NOTE_D2, 500, 0x01, 100},
    {NOTE_E2, 1000, 0x01, 500},
    {NOTE_D2, 1000, 0x01, 500},
    {NOTE_C2, 2000, 0x01, 1000},
    {NOTE_A1, 3000, 0x00, 1000},
};

/* ── SONG 2 ──────────────────────────────────────────────────────────────────
 * Format: { NOTE_XX, dur_ms, finger_mask, finger_dur_ms }
 * ────────────────────────────────────────────────────────────────────────── */
static const SongNote song2_notes[] = {
    {NOTE_F2, 2000, 0x00, 750},

    {NOTE_F2, 1500, 0x10, 750},
    {NOTE_E2, 500, 0x10, 150},
    {NOTE_F2, 1000, 0x04, 500},
    {NOTE_E2, 1000, 0x04, 500},
    {NOTE_F2, 1500, 0x01, 750},
    {NOTE_G2, 500, 0x01, 150},
    {NOTE_F2, 1000, 0x04, 500},
    {NOTE_F2, 1000, 0x10, 500},
    {NOTE_E2, 1500, 0x10, 750},
    {NOTE_F2, 500, 0x04, 150},
    {NOTE_E2, 1000, 0x04, 500},
    {NOTE_F2, 1000, 0x01, 500},
    {NOTE_E2, 4000, 0x01, 3000},
    {NOTE_D2, 1500, 0x10, 750},
    {NOTE_E2, 500, 0x04, 150},
    {NOTE_D2, 1000, 0x04, 500},
    {NOTE_E2, 1000, 0x01, 500},
    {NOTE_D2, 1500, 0x01,750},
    {NOTE_E2, 500, 0x04, 150},
    {NOTE_D2, 1000, 0x04, 500},
    {NOTE_D2, 1000, 0x10, 500},
    {NOTE_D2, 1500, 0x08, 750},
    {NOTE_D2, 1000, 0x04, 500},
    {NOTE_E2, 1000, 0x01, 500},
    {NOTE_D2, 1000, 0x01,500},
    {NOTE_C2, 2000, 0x01, 1000},
    {NOTE_A1, 3000, 0x00, 1000},
};

/* ── SONG 3 ──────────────────────────────────────────────────────────────────
 * Format: { NOTE_XX, dur_ms, finger_mask, finger_dur_ms }
 * ────────────────────────────────────────────────────────────────────────── */
static const SongNote song3_notes[] = {
// 1st Line
    // 1st Line
    { NOTE_C2,  700,  0x00, 150 },
    { NOTE_C2,  700,  0x01, 150 },      // C
    { NOTE_C2,  700,  0x02, 150 },      // C#
    { NOTE_D2,  700,  0x01, 150 },      // D
    { NOTE_D2,  700,  0x02, 150 },      // D#
    { NOTE_E2,  1200,  0x01, 300 },     // E
    { NOTE_E2,  700,  0x01, 150 },      // E
    { NOTE_F2,  700,  0x01, 150 },      // F
    { NOTE_F2,  700,  0x02, 150 },      // F#
    { NOTE_G2,  700,  0x01, 150 },      // G
    { NOTE_G2,  1200,  0x02, 300 },     // G#
    { NOTE_G2,  700,  0x01, 150 },      // G
    { NOTE_G2,  1200,  0x02, 300 },     // G#
    { NOTE_G2,  700,  0x01, 150 },      // G
    { NOTE_G2,  700,  0x02, 150 },      // G#
    { NOTE_G2,  700,  0x02, 150 },      // G#
    { NOTE_G2,  700,  0x01, 150 },      // G
    { NOTE_B1,  1200,  0x15, 300 },     // BDF

    { NOTE_A1,  700,  0x10, 150 },      // E
    { NOTE_A1,  700,  0x08, 150 },      // D#
    { NOTE_A1,  700,  0x10, 150 },      // E
    { NOTE_C2,  1200,  0x10, 300 },     // G
    { NOTE_D2,  700,  0x04, 150 },      // F
    { NOTE_E2,  700,  0x01, 150 },      // E
    { NOTE_D2,  700,  0x04, 150 },      // F
    { NOTE_E2,  1200,  0x04, 300 },     // G
    { NOTE_D2,  700,  0x04, 150 },      // F
    { NOTE_E2,  700,  0x01, 150 },      // E
    { NOTE_D2,  700,  0x04, 150 },      // F
    { NOTE_E2,  1200,  0x04, 300 },     // G
    { NOTE_E2,  700,  0x01, 150 },      // E
    { NOTE_E2,  700,  0x04, 150 },      // G
    { NOTE_F2,  700,  0x04, 150 },      // A
    { NOTE_C2,  1200,  0x15, 300 },     // G

    { NOTE_A1,  700,  0x10, 150 },      // E
    { NOTE_A1,  700,  0x08, 150 },      // D#
    { NOTE_A1,  700,  0x10, 150 },      // E
    { NOTE_C2,  1200,  0x10, 300 },     // G
    { NOTE_D2,  700,  0x04, 150 },      // F
    { NOTE_E2,  700,  0x01, 150 },      // E
    { NOTE_D2,  700,  0x04, 150 },      // F
    { NOTE_E2,  1200,  0x04, 300 },     // G

    { NOTE_C2,  700,  0x10, 150 },     // G
    { NOTE_C2,  700,  0x08, 150 },     // F#
    { NOTE_C2,  700,  0x10, 150 },     // G
    { NOTE_D2,  700,  0x08, 150 },     // G#
    { NOTE_D2,  700,  0x10, 150 },     // A
    { NOTE_E2,  700,  0x08, 150 },     // A#
    { NOTE_E2,  700,  0x10, 150 },     // B
    { NOTE_F2,  1200,  0x10, 300 },    // C

    { NOTE_C2,  1500,  0x15, 400 },    // CEG
    { NOTE_F2,  700,  0x15, 150 },     // FAC
    { NOTE_G2,  700,  0x15, 150 },     // GBD
    { NOTE_C2,  2000,  0x15, 600 },    // CEG
    {NOTE_A1, 3000, 0x00, 1000},
};

static const SongNote * const songs[4] = { song0_notes, song1_notes, song2_notes, song3_notes };
static const uint32_t song_lengths[4]  = {
    sizeof(song0_notes) / sizeof(song0_notes[0]),
    sizeof(song1_notes) / sizeof(song1_notes[0]),
    sizeof(song2_notes) / sizeof(song2_notes[0]),
    sizeof(song3_notes) / sizeof(song3_notes[0]),
};

static const SongNote *active_song     = song0_notes;
static uint32_t        active_song_len = sizeof(song0_notes) / sizeof(song0_notes[0]);

static uint8_t  song_playing  = 0;
static uint8_t  song_finished = 0;   /* set to 1 when playback ends, cleared by getter */
static uint32_t song_step     = 0;
static uint32_t song_next_ms  = 0;

/* Per-note finger timing */
static uint8_t  song_fingers_waiting  = 0;  /* waiting for motor to settle before pressing */
static uint32_t song_finger_dur_saved = 0;  /* finger_dur_ms saved until motor settles     */
static uint8_t  song_fingers_pressed  = 0;  /* fingers currently held down                 */
static uint32_t song_finger_off_ms    = 0;  /* absolute tick to release fingers            */

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

/* ── Command processing ──────────────────────────────────────────────────── */

static void ProcessMainMenu(char *cmd)
{
    if (strncmp(cmd, "test", 4) == 0) {
        current_mode = MODE_TEST_IDLE;
    } else if (strncmp(cmd, "pid", 3) == 0) {
        current_mode = MODE_PID_IDLE;
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
        } else if (strncmp(cmd, "off", 3) == 0) {
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
        } else {
            return;
        }
        current_mode = MODE_TEST_IDLE;
        return;
    }

    if (strncmp(cmd, "cw", 2) == 0 && (cmd[2] == '\0' || cmd[2] == '\r')) {
        current_mode = MODE_TEST_CW;
    }
    else if (strncmp(cmd, "ccw", 3) == 0 && (cmd[3] == '\0' || cmd[3] == '\r')) {
        current_mode = MODE_TEST_CCW;
    }
    else if (cmd[0] == 's' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        MotorStop();
        if (current_mode == MODE_TEST_ACTUATE) {
            for (int i = 0; i < 5; i++)
                HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
        }
        current_mode = MODE_TEST_IDLE;
    }
    else if (strncmp(cmd, "sensor", 6) == 0) {
        AS5600_Update();
        AS5600_Position_t pos;
        AS5600_GetPosition(&pos);
        (void)pos;
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
            /* no valid fingers */
        } else if (state == -1) {
            current_mode = MODE_TEST_FINGER_ONOFF;
        } else {
            GPIO_PinState pin_state = (state == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            for (int i = 0; i < 5; i++)
                if (selected_fingers & (1 << i))
                    HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, pin_state);
        }
    }
    else if (strncmp(cmd, "actuate", 7) == 0 && (cmd[7] == '\0' || cmd[7] == '\r')) {
        actuate_state = 0;
        for (int i = 0; i < 5; i++)
            HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
        current_mode = MODE_TEST_ACTUATE;
    }
    else {
        /* Delegate to main.c for lcd commands */
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
        return;
    }

    if (strncmp(cmd, "home", 4) == 0) {
        MotorControl_Home();
    }
    else if (strncmp(cmd, "setl ", 5) == 0) {
        float cm = atof(&cmd[5]);
        ang_set = (cm / 100.0f) / PITCH_RADIUS;
    }
    else if (strncmp(cmd, "gains ", 6) == 0) {
        float kp, ki, kd;
        if (sscanf(&cmd[6], "%f %f %f", &kp, &ki, &kd) == 3) {
            Kp = kp; Ki = ki; Kd = kd;
        }
    }
    else if (cmd[0] == 'a' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        /* position query — no serial output */
    }
    else if (strncmp(cmd, "pid", 3) == 0 && (cmd[3] == '\0' || cmd[3] == '\r')) {
        integral = 0.0f;
        current_mode = MODE_PID_ACTIVE;
    }
    else if (cmd[0] == 'r' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        integral = 0.0f;
        current_mode = MODE_PID_RECEIVE;
    }
    else if (cmd[0] == 's' && (cmd[1] == '\0' || cmd[1] == '\r')) {
        current_mode = MODE_PID_STOP;
    }
}

/* -------------------------------------------------------------------------- */
static void ProcessCommand(char *cmd)
{
    /* 'back' returns to the main menu from anywhere */
    if (strncmp(cmd, "back", 4) == 0) {
        MotorStop();
        current_mode = MODE_MAIN_MENU;
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
                    /* all OFF */
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
                    MotorCW(8999);
                else
                    MotorCCW(8999);
                ang_prev = ang_curr;
                break;

            default:
                MotorStop();
                ang_prev = ang_curr;
                break;
        }

        /* Advance print_div even though we no longer stream telemetry */
        if (++print_div >= 10)
            print_div = 0;
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
        uint32_t now     = HAL_GetTick();
        /* Set only when the step advance forcibly released held fingers this
         * iteration — prevents the settling check from immediately re-pressing
         * on the same call (double-press on repeated notes). */
        uint8_t  just_released = 0;
        if (now >= song_next_ms) {
            if (song_step >= active_song_len) {
                song_playing  = 0;
                song_finished = 1;
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
                const SongNote *n = &active_song[song_step];
                ang_set      = (note_positions_cm[n->note] / 100.0f) / PITCH_RADIUS;
                song_next_ms = now + n->dur_ms;

                /* Release any fingers still held from the previous note */
                if (song_fingers_pressed) {
                    for (int i = 0; i < 5; i++)
                        if (song_fingers_pressed & (1 << i))
                            HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
                    song_fingers_pressed = 0;
                    just_released = 1;  /* guard against same-tick re-press */
                }
                /* Arm fingers — will press once motor settles at new position */
                song_fingers_waiting  = n->fingers;
                song_finger_dur_saved = n->finger_dur_ms;

                song_step++;
            }
        }

        /* Press fingers once motor has settled at the target position.
         * Skip if we just released fingers this iteration so the solenoid has
         * at least one loop iteration to physically move before being re-pressed. */
        if (!just_released && song_fingers_waiting && !song_fingers_pressed &&
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
void MotorControl_UartISR(void)
{
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
    MotorCCW(8995);
    HAL_Delay(500);
    MotorCCW(1200);
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
    /* Live read — works even before homing (position is relative to power-on) */
    float angle = GetMotorAngle();
    return -angle * PITCH_RADIUS;
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

void MotorControl_PlaySong(int song_idx)
{
    if (song_idx < 0 || song_idx > 3) song_idx = 0;
    active_song     = songs[song_idx];
    active_song_len = song_lengths[song_idx];
    song_step       = 0;
    song_next_ms    = HAL_GetTick();
    song_playing    = 1;
    song_finished   = 0;
    integral        = 0.0f;
    current_mode    = MODE_PID_ACTIVE;
}

void MotorControl_StopSong(void)
{
    song_playing         = 0;
    song_fingers_waiting = 0;
    if (song_fingers_pressed) {
        for (int i = 0; i < 5; i++)
            HAL_GPIO_WritePin(fingers[i].port, fingers[i].pin, GPIO_PIN_RESET);
        song_fingers_pressed = 0;
    }
    MotorStop();
    current_mode = MODE_PID_IDLE;
}

uint8_t MotorControl_IsSongFinished(void)
{
    if (song_finished) {
        song_finished = 0;
        return 1;
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
__attribute__((weak)) void MotorControl_ExtCommand(const char *cmd)
{
    (void)cmd;
}
