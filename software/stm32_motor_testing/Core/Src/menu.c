#include "menu.h"
#include "main.h"
#include "actuator.h"
#include "motor_control.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * FORWARD DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */
static void _handle_confirm(int index);
static void _process_buttons(void);
static void _redraw_screen(ScreenState s);

/* ═══════════════════════════════════════════════════════════════════════════
 * CALLBACKS – forward declarations
 * ═══════════════════════════════════════════════════════════════════════════ */
/* Sensor */
static void cb_update_pos(void);

/* Actuators */
static void cb_finger_1(void);
static void cb_finger_2(void);
static void cb_finger_3(void);
static void cb_finger_4(void);
static void cb_finger_5(void);

/* Motor test */
static void cb_home(void);
static void cb_song(void);
static void cb_cw(void);
static void cb_ccw(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * MENU ITEM ARRAYS
 *
 * Screen area: 240x320 portrait.
 * Title bar:   y = 0..40  (40 px)
 * Usable area: y = 48..319 (271 px)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Main menu ───────────────────────────────────────────────────────────── */
/*  Three full-width buttons, maximised to fill the usable area.
 *  Each h=78, gap=10 → 3×78 + 2×10 = 254 px, starts at y=48.           */
static MenuItem main_items[] = {
    /*  x    y    w    h   label        sc  normal       pressed            target                  callback */
    {  10,  48, 220,  78, "SENSOR",      3, COL_BLUE,    COL_BLUE_PRESS,   SCREEN_SENSOR,          NULL },
    {  10, 136, 220,  78, "ACTUATORS",   3, COL_PURPLE,  COL_PURPLE_PRESS, SCREEN_ACTUATORS,       NULL },
    {  10, 224, 220,  78, "MOTOR TEST",  3, COL_GREEN,   COL_GREEN_PRESS,  SCREEN_MOTOR_TEST,      NULL },
};
#define MAIN_ITEM_COUNT  3

/* ── Sensor screen ───────────────────────────────────────────────────────── */
/*  One huge UPDATE POS button + a BACK button.
 *  UPDATE: y=50, h=155  – takes most of the screen.
 *  BACK:   y=260, h=52                                                      */
static MenuItem sensor_items[] = {
    /*  x    y    w    h   label         sc  normal       pressed           target        callback     */
    {  10,  50, 220, 155, "UPDATE POS",   3, COL_BLUE,    COL_BLUE_PRESS,  SCREEN_NONE,  cb_update_pos },
    {  10, 260, 220,  52, "BACK",         2, COL_RED,     COL_RED_PRESS,   SCREEN_NONE,  menu_navigate_back },
};
#define SENSOR_ITEM_COUNT  2

/* ── Actuators screen ────────────────────────────────────────────────────── */
/*  5 finger toggle buttons in a 2-2-1+BACK layout.
 *  Two columns: x=10, w=105  /  x=125, w=105
 *  Row 1: F1, F2  y=50, h=82
 *  Row 2: F3, F4  y=142, h=82
 *  Row 3: F5, BACK  y=234, h=66                                             */
static MenuItem actuator_items[] = {
    /*  x    y    w    h   label  sc  normal       pressed           target       callback    */
    {  10,  50, 105,  82, "F1",   3, COL_MIDGREY, COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_1 },
    { 125,  50, 105,  82, "F2",   3, COL_MIDGREY, COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_2 },
    {  10, 142, 105,  82, "F3",   3, COL_MIDGREY, COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_3 },
    { 125, 142, 105,  82, "F4",   3, COL_MIDGREY, COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_4 },
    {  10, 234, 105,  66, "F5",   3, COL_MIDGREY, COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_5 },
    { 125, 234, 105,  66, "BACK", 2, COL_RED,     COL_RED_PRESS,  SCREEN_NONE,  menu_navigate_back },
};
#define ACTUATOR_ITEM_COUNT  6

/* ── Motor test screen ───────────────────────────────────────────────────── */
/*  HOME (full-width), SONG (full-width), CW + CCW (side by side), BACK.
 *  HOME: y=50, h=60
 *  SONG: y=120, h=60
 *  CW / CCW: y=190, h=60
 *  BACK: y=260, h=52                                                        */
static MenuItem motor_test_items[] = {
    /*  x    y    w    h   label  sc  normal       pressed            target       callback  */
    {  10,  50, 220,  60, "HOME", 3, COL_ORANGE,  COL_ORANGE_PRESS, SCREEN_NONE,  cb_home },
    {  10, 120, 220,  60, "SONG", 3, COL_GREEN,   COL_GREEN_PRESS,  SCREEN_NONE,  cb_song },
    {  10, 190, 105,  60, "CW",   3, COL_MIDGREY, COL_BLUE_PRESS,   SCREEN_NONE,  cb_cw   },
    { 125, 190, 105,  60, "CCW",  3, COL_MIDGREY, COL_BLUE_PRESS,   SCREEN_NONE,  cb_ccw  },
    {  10, 260, 220,  52, "BACK", 2, COL_RED,     COL_RED_PRESS,    SCREEN_NONE,  menu_navigate_back },
};
#define MOTOR_TEST_ITEM_COUNT  5

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERNAL STATE
 * ═══════════════════════════════════════════════════════════════════════════ */

static MenuItem    *_items      = NULL;
static int          _item_count = 0;

static ScreenState  _cur_screen  = SCREEN_MAIN_MENU;
static ScreenState  _prev_screen = SCREEN_NONE;

static ScreenState  _stack[4];
static int          _stack_top = 0;

/* Button-navigation state */
static int  _cursor    = 0;   /* index of highlighted menu item */

#define BTN_STABLE  5         /* consecutive samples to register a press */
static int  _nav_cnt   = 0;
static bool _nav_armed = true;
static int  _sel_cnt   = 0;
static bool _sel_armed = true;

/* Sensor screen state */
static char _pos_text[32] = "";

/* Actuators state */
static bool _finger_on[5] = {false, false, false, false, false};

/* Motor test state */
typedef enum { MTEST_STOP = 0, MTEST_CW, MTEST_CCW } MotorTestState;
static MotorTestState _motor_state = MTEST_STOP;

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void _handle_confirm(int index)
{
    const MenuItem *item = &_items[index];

    if (item->target_screen != SCREEN_NONE)
        menu_navigate_to(item->target_screen);

    if (item->on_select != NULL)
        item->on_select();
}

/* Poll BTN_NAV and BTN_SEL (active-low, pull-up).
 * BTN_NAV cycles the cursor through items.
 * BTN_SEL confirms the highlighted item. */
static void _process_buttons(void)
{
    bool nav_raw = (HAL_GPIO_ReadPin(BTN_NAV_GPIO_Port, BTN_NAV_Pin) == GPIO_PIN_RESET);
    bool sel_raw = (HAL_GPIO_ReadPin(BTN_SEL_GPIO_Port, BTN_SEL_Pin) == GPIO_PIN_RESET);

    /* NAV: advance cursor on stable press */
    if (nav_raw) {
        if (_nav_cnt < BTN_STABLE) _nav_cnt++;
    } else {
        _nav_cnt = 0;
        _nav_armed = true;
    }
    if (_nav_cnt >= BTN_STABLE && _nav_armed) {
        _nav_armed = false;
        int old = _cursor;
        _cursor = (_cursor + 1) % _item_count;
        menu_draw_item(&_items[old], false);
        menu_draw_item(&_items[_cursor], true);
    }

    /* SEL: confirm on stable press */
    if (sel_raw) {
        if (_sel_cnt < BTN_STABLE) _sel_cnt++;
    } else {
        _sel_cnt = 0;
        _sel_armed = true;
    }
    if (_sel_cnt >= BTN_STABLE && _sel_armed) {
        _sel_armed = false;
        ScreenState before = _cur_screen;
        _handle_confirm(_cursor);
        /* If we didn't navigate away, re-highlight the cursor item */
        if (_cur_screen == before) {
            menu_draw_item(&_items[_cursor], true);
        }
    }
}

static void _redraw_screen(ScreenState s)
{
    ili9341_fill_screen(COL_BLACK);

    switch (s) {
        case SCREEN_MAIN_MENU:
            _items      = main_items;
            _item_count = MAIN_ITEM_COUNT;
            menu_draw_main();
            break;
        case SCREEN_SENSOR:
            _items      = sensor_items;
            _item_count = SENSOR_ITEM_COUNT;
            menu_draw_sensor();
            break;
        case SCREEN_ACTUATORS:
            _items      = actuator_items;
            _item_count = ACTUATOR_ITEM_COUNT;
            menu_draw_actuators();
            break;
        case SCREEN_MOTOR_TEST:
            _items      = motor_test_items;
            _item_count = MOTOR_TEST_ITEM_COUNT;
            menu_draw_motor_test();
            break;
        default:
            break;
    }

    _cursor    = 0;
    _nav_cnt   = 0; _nav_armed = true;
    _sel_cnt   = 0; _sel_armed = true;
    menu_draw_item(&_items[0], true);  /* highlight first item */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC – NAVIGATION
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_navigate_to(ScreenState s)
{
    if (_stack_top < 4)
        _stack[_stack_top++] = _cur_screen;
    _cur_screen = s;
}

void menu_navigate_back(void)
{
    if (_stack_top > 0)
        _cur_screen = _stack[--_stack_top];
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC – DRAWING
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_draw_item(const MenuItem *item, bool pressed)
{
    uint16_t fill = pressed ? item->col_pressed : item->col_normal;

    ili9341_fill_rounded_rect(item->x, item->y, item->w, item->h, 8, fill);
    ili9341_draw_rounded_rect(item->x, item->y, item->w, item->h, 8, COL_WHITE);

    uint8_t sc = item->label_scale;
    ili9341_set_font_scale(sc);
    uint16_t tw = ili9341_string_width(item->label);
    uint16_t tx = item->x + (item->w - tw) / 2;
    uint16_t ty = item->y + (item->h - TEXT_H(sc)) / 2;

    ili9341_set_text_color(COL_WHITE, fill);
    ili9341_draw_string(tx, ty, item->label);
}

void menu_draw_title(const char *title)
{
    ili9341_fill_rect(0, 0, SCREEN_W, 40, COL_DARKGREY);

    ili9341_set_font_scale(3);
    uint16_t tw = ili9341_string_width(title);
    uint16_t tx = (SCREEN_W - tw) / 2;
    uint16_t ty = (40 - TEXT_H(3)) / 2;

    ili9341_set_text_color(COL_WHITE, COL_DARKGREY);
    ili9341_draw_string(tx, ty, title);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC – PER-SCREEN DRAW FUNCTIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_draw_main(void)
{
    menu_draw_title("MTR TESTER");

    for (int i = 0; i < MAIN_ITEM_COUNT; i++)
        menu_draw_item(&main_items[i], false);
}

void menu_draw_sensor(void)
{
    menu_draw_title("SENSOR");

    for (int i = 0; i < SENSOR_ITEM_COUNT; i++)
        menu_draw_item(&sensor_items[i], false);

    /* Redraw last known position reading */
    if (_pos_text[0] != '\0') {
        ili9341_set_font_scale(2);
        ili9341_set_text_color(COL_LIGHTGREY, COL_BLACK);
        ili9341_draw_string(10, 215, _pos_text);
    }
}

void menu_draw_actuators(void)
{
    menu_draw_title("ACTUATORS");

    /* Sync button colours to current finger state */
    for (int i = 0; i < 5; i++)
        actuator_items[i].col_normal = _finger_on[i] ? COL_GREEN : COL_MIDGREY;

    for (int i = 0; i < ACTUATOR_ITEM_COUNT; i++)
        menu_draw_item(&actuator_items[i], false);
}

void menu_draw_motor_test(void)
{
    /* Stop any running test when entering this screen */
    _motor_state = MTEST_STOP;
    MotorControl_SetTestStop();
    motor_test_items[2].col_normal = COL_MIDGREY;
    motor_test_items[3].col_normal = COL_MIDGREY;

    menu_draw_title("MOTOR TEST");

    for (int i = 0; i < MOTOR_TEST_ITEM_COUNT; i++)
        menu_draw_item(&motor_test_items[i], false);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC – INIT & UPDATE
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_init(void)
{
    _cur_screen  = SCREEN_MAIN_MENU;
    _prev_screen = SCREEN_NONE;
    _stack_top   = 0;
    _cursor      = 0;
    _nav_cnt     = 0; _nav_armed = true;
    _sel_cnt     = 0; _sel_armed = true;
}

void menu_update(void)
{
    _process_buttons();

    if (_cur_screen != _prev_screen) {
        _redraw_screen(_cur_screen);
        _prev_screen = _cur_screen;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SENSOR SCREEN CALLBACKS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void cb_update_pos(void)
{
    float lin = MotorControl_ReadLinPos();
    float cm  = lin * 100.0f;

    /* Format floats manually — newlib-nano strips %f by default */
    int c_int  = (int)cm;
    int c_frac = (int)((cm - (float)c_int) * 100.0f);
    if (c_frac < 0) c_frac = -c_frac;

    snprintf(_pos_text, sizeof(_pos_text), "Pos: %d.%02d cm", c_int, c_frac);

    ili9341_fill_rect(10, 210, 220, 20, COL_BLACK);
    ili9341_set_font_scale(2);
    ili9341_set_text_color(COL_LIGHTGREY, COL_BLACK);
    ili9341_draw_string(10, 215, _pos_text);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ACTUATOR SCREEN CALLBACKS
 * ═══════════════════════════════════════════════════════════════════════════ */

__weak void menu_test_finger(int idx, bool on) { (void)idx; (void)on; }

static void _toggle_finger(int idx)
{
    _finger_on[idx] = !_finger_on[idx];
    menu_test_finger(idx, _finger_on[idx]);
    actuator_items[idx].col_normal = _finger_on[idx] ? COL_GREEN : COL_MIDGREY;
    menu_draw_item(&actuator_items[idx], (idx == _cursor));
}

static void cb_finger_1(void) { _toggle_finger(0); }
static void cb_finger_2(void) { _toggle_finger(1); }
static void cb_finger_3(void) { _toggle_finger(2); }
static void cb_finger_4(void) { _toggle_finger(3); }
static void cb_finger_5(void) { _toggle_finger(4); }

/* ═══════════════════════════════════════════════════════════════════════════
 * MOTOR TEST SCREEN CALLBACKS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void cb_home(void)
{
    /* Stop any open-loop test before homing */
    _motor_state = MTEST_STOP;
    motor_test_items[2].col_normal = COL_MIDGREY;
    motor_test_items[3].col_normal = COL_MIDGREY;
    menu_draw_item(&motor_test_items[2], false);
    menu_draw_item(&motor_test_items[3], false);

    MotorControl_Home();
}

static void cb_song(void)
{
    /* Stop any open-loop test, home, then play */
    _motor_state = MTEST_STOP;
    motor_test_items[2].col_normal = COL_MIDGREY;
    motor_test_items[3].col_normal = COL_MIDGREY;
    menu_draw_item(&motor_test_items[2], false);
    menu_draw_item(&motor_test_items[3], false);

    MotorControl_Home();
    MotorControl_PlaySong();
}

static void cb_cw(void)
{
    if (_motor_state == MTEST_CW) {
        /* Already spinning CW – stop */
        _motor_state = MTEST_STOP;
        MotorControl_SetTestStop();
        motor_test_items[2].col_normal = COL_MIDGREY;
    } else {
        /* Start CW, cancel CCW if active */
        _motor_state = MTEST_CW;
        MotorControl_SetTestCW();
        motor_test_items[2].col_normal = COL_BLUE;
        motor_test_items[3].col_normal = COL_MIDGREY;
        menu_draw_item(&motor_test_items[3], false);
    }
    menu_draw_item(&motor_test_items[2], false);
}

static void cb_ccw(void)
{
    if (_motor_state == MTEST_CCW) {
        /* Already spinning CCW – stop */
        _motor_state = MTEST_STOP;
        MotorControl_SetTestStop();
        motor_test_items[3].col_normal = COL_MIDGREY;
    } else {
        /* Start CCW, cancel CW if active */
        _motor_state = MTEST_CCW;
        MotorControl_SetTestCCW();
        motor_test_items[3].col_normal = COL_BLUE;
        motor_test_items[2].col_normal = COL_MIDGREY;
        menu_draw_item(&motor_test_items[2], false);
    }
    menu_draw_item(&motor_test_items[3], false);
}
