#include "menu.h"
#include "main.h"
#include "actuator.h"
#include "motor_control.h"
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * FORWARD DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */
static bool _hit_test(uint16_t tx, uint16_t ty, const MenuItem *item);
static void _handle_confirm(int index);
static void _process_touch(const TouchPoint *tp);
static void _redraw_screen(ScreenState s);

/* ═══════════════════════════════════════════════════════════════════════════
 * CALLBACKS
 * ═══════════════════════════════════════════════════════════════════════════ */
static void cb_start_playing(void);
static void cb_stop_playing(void);
static void cb_save_settings(void);
static void cb_select_song_0(void);
static void cb_select_song_1(void);
static void cb_select_song_2(void);

/* Test screen */
static void cb_finger_1(void);
static void cb_finger_2(void);
static void cb_finger_3(void);
static void cb_finger_4(void);
static void cb_finger_5(void);
static void cb_motor_cycle(void);
static void cb_enc_pos(void);
static void cb_motor_home(void);
static void cb_song_test(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * MENU ITEM ARRAYS
 * Screen area: 240x320 portrait.
 * Title bar:   y = 0..39  (40 px)
 * Usable area: y = 48..319
 *
 * Full-width buttons: x=10, w=220
 * Three stacked:  h=60, gap=12  → y=48, 120, 192
 * Side-by-side pair: w=105 each, gap=10 → x=10 / x=125
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Main menu ───────────────────────────────────────────────────────────── */
static MenuItem main_items[] = {
    /*  x    y    w    h   label          sc  normal        pressed           target              callback */
    {  10,  48, 220,  60, "SELECT SONG",   2, COL_BLUE,     COL_BLUE_PRESS,  SCREEN_SONG_SELECT, NULL },
    {  10, 120, 220,  60, "SETTINGS",      2, COL_MIDGREY,  COL_LIGHTGREY,   SCREEN_SETTINGS,    NULL },
    {  10, 192, 220,  60, "TEST",          2, COL_GREEN,    COL_GREEN_PRESS, SCREEN_TEST,        NULL },
};
#define MAIN_ITEM_COUNT  3

/* ── Song select screen ──────────────────────────────────────────────────── */
static MenuItem song_select_items[] = {
    /*  x    y    w    h   label     sc  normal       pressed           target          callback           */
    {  10,  48, 220,  52, "SONG 1",   2, COL_BLUE,    COL_BLUE_PRESS,  SCREEN_NONE,    cb_select_song_0   },
    {  10, 108, 220,  52, "SONG 2",   2, COL_BLUE,    COL_BLUE_PRESS,  SCREEN_NONE,    cb_select_song_1   },
    {  10, 168, 220,  52, "SONG 3",   2, COL_BLUE,    COL_BLUE_PRESS,  SCREEN_NONE,    cb_select_song_2   },
    {  10, 240, 105,  52, "BACK",     2, COL_RED,     COL_RED_PRESS,   SCREEN_NONE,    menu_navigate_back },
    { 125, 240, 105,  52, "PLAY",     2, COL_GREEN,   COL_GREEN_PRESS, SCREEN_PLAYING, cb_start_playing   },
};
#define SONG_SELECT_ITEM_COUNT  5

/* ── Playing screen ──────────────────────────────────────────────────────── */
static MenuItem playing_items[] = {
    /*  x    y    w    h   label   sc  normal       pressed           target      callback           */
    {  10, 248, 105,  52, "STOP",   2, COL_RED,     COL_RED_PRESS,   SCREEN_NONE, cb_stop_playing    },
    { 125, 248, 105,  52, "BACK",   2, COL_ORANGE,  COL_ORANGE_PRESS, SCREEN_NONE, menu_navigate_back },
};
#define PLAYING_ITEM_COUNT  2

/* ── Settings screen ─────────────────────────────────────────────────────── */
static MenuItem settings_items[] = {
    {  10,  48, 220,  60, "OPTION 1",  2, COL_MIDGREY, COL_LIGHTGREY,   SCREEN_NONE, NULL               },
    {  10, 120, 220,  60, "OPTION 2",  2, COL_MIDGREY, COL_LIGHTGREY,   SCREEN_NONE, NULL               },
    {  10, 200, 105,  60, "BACK",      2, COL_RED,     COL_RED_PRESS,   SCREEN_NONE, menu_navigate_back },
    { 125, 200, 105,  60, "SAVE",      2, COL_GREEN,   COL_GREEN_PRESS, SCREEN_NONE, cb_save_settings   },
};
#define SETTINGS_ITEM_COUNT  4

/* ── Test screen ─────────────────────────────────────────────────────────────
 * Layout (portrait 240x320):
 *   Title bar     y=0-40
 *   "FINGERS" lbl y=48
 *   F1/F2/F3      y=58  h=50
 *   F4/F5/ALL     y=116 h=50
 *   "MOTOR" lbl   y=172
 *   CCW/HOME/CW   y=182 h=50
 *   BACK          y=255 h=52
 * Three-across columns: x=5/85/165, w=70
 * ─────────────────────────────────────────────────────────────────────────── */
static MenuItem test_items[] = {
    /*  x    y    w    h   label  sc  normal        pressed           target       callback        */
    {   5,  58,  70,  50, "F1",   2, COL_MIDGREY,  COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_1  },
    {  85,  58,  70,  50, "F2",   2, COL_MIDGREY,  COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_2  },
    { 165,  58,  70,  50, "F3",   2, COL_MIDGREY,  COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_3  },
    {   5, 116,  70,  50, "F4",   2, COL_MIDGREY,  COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_4  },
    {  85, 116,  70,  50, "F5",   2, COL_MIDGREY,  COL_LIGHTGREY,  SCREEN_NONE,  cb_finger_5  },
    { 165, 116,  70,  50, "MOT",  2, COL_MIDGREY,  COL_LIGHTGREY,  SCREEN_NONE,  cb_motor_cycle },
    {   5, 182,  70,  50, "ENC",  2, COL_BLUE,     COL_BLUE_PRESS,  SCREEN_NONE, cb_enc_pos   },
    {  85, 182,  70,  50, "HOME", 2, COL_ORANGE,   COL_ORANGE_PRESS,SCREEN_NONE, cb_motor_home},
    { 165, 182,  70,  50, "SONG", 2, COL_GREEN,    COL_GREEN_PRESS, SCREEN_NONE, cb_song_test },
    {  10, 255, 220,  52, "BACK", 2, COL_RED,      COL_RED_PRESS,  SCREEN_NONE,  menu_navigate_back },
};
#define TEST_ITEM_COUNT  10

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERNAL STATE
 * ═══════════════════════════════════════════════════════════════════════════ */

static MenuItem    *_items      = NULL;
static int          _item_count = 0;

static ScreenState  _cur_screen  = SCREEN_MAIN_MENU;
static ScreenState  _prev_screen = SCREEN_NONE;   /* NONE forces first draw */

static ScreenState  _stack[4];
static int          _stack_top = 0;

typedef enum { TOUCH_IDLE, TOUCH_DOWN } TouchState;
static TouchState   _touch_state      = TOUCH_IDLE;
static int          _touch_item_index = -1;
static uint16_t     _touch_down_x     = 0;
static uint16_t     _touch_down_y     = 0;

#define NO_TOUCH_STABLE  3
static int          _no_touch_count   = 0;

static int  _selected_song  = -1;
static bool _finger_on[5]   = {false, false, false, false, false};
static char _enc_text[32]   = "";

typedef enum { MTEST_OFF = 0, MTEST_CW, MTEST_CCW, MTEST_SWITCH } MTestState;
static MTestState _mtest_state = MTEST_OFF;
static const char *_mtest_labels[] = { "MOT", "CW",  "CCW", "SWT" };
static const uint16_t _mtest_colors[] = { COL_MIDGREY, COL_BLUE, COL_GREEN, COL_ORANGE };

/* ═══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool _hit_test(uint16_t tx, uint16_t ty, const MenuItem *item)
{
    return (tx >= item->x) && (tx < item->x + item->w) &&
           (ty >= item->y) && (ty < item->y + item->h);
}

static void _handle_confirm(int index)
{
    const MenuItem *item = &_items[index];

    if (item->target_screen != SCREEN_NONE)
        menu_navigate_to(item->target_screen);

    if (item->on_select != NULL)
        item->on_select();
}

static void _process_touch(const TouchPoint *tp)
{
    /* Debounce lift-off jitter */
    if (!tp->touched) {
        _no_touch_count++;
        if (_no_touch_count < NO_TOUCH_STABLE)
            return;
    } else {
        _no_touch_count = 0;
    }

    /* Finger just landed */
    if (tp->touched && _touch_state == TOUCH_IDLE) {
        _touch_state      = TOUCH_DOWN;
        _touch_down_x     = tp->x;
        _touch_down_y     = tp->y;
        _touch_item_index = -1;

        for (int i = 0; i < _item_count; i++) {
            if (_hit_test(tp->x, tp->y, &_items[i])) {
                _touch_item_index = i;
                menu_draw_item(&_items[i], true);
                break;
            }
        }
        return;
    }

    /* Finger just lifted */
    if (!tp->touched && _touch_state == TOUCH_DOWN) {
        _touch_state = TOUCH_IDLE;

        if (_touch_item_index >= 0) {
            if (_hit_test(_touch_down_x, _touch_down_y,
                          &_items[_touch_item_index])) {
                menu_draw_item(&_items[_touch_item_index], false);
                _handle_confirm(_touch_item_index);
            } else {
                menu_draw_item(&_items[_touch_item_index], false);
            }
            _touch_item_index = -1;
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
        case SCREEN_SONG_SELECT:
            _items      = song_select_items;
            _item_count = SONG_SELECT_ITEM_COUNT;
            menu_draw_song_select();
            break;
        case SCREEN_PLAYING:
            _items      = playing_items;
            _item_count = PLAYING_ITEM_COUNT;
            menu_draw_playing();
            break;
        case SCREEN_SETTINGS:
            _items      = settings_items;
            _item_count = SETTINGS_ITEM_COUNT;
            menu_draw_settings();
            break;
        case SCREEN_TEST:
            _items      = test_items;
            _item_count = TEST_ITEM_COUNT;
            menu_draw_test();
            break;
        default:
            break;
    }

    _touch_state      = TOUCH_IDLE;
    _touch_item_index = -1;
    _no_touch_count   = 0;
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
    menu_draw_title("ROBOMAESTRO");

    for (int i = 0; i < MAIN_ITEM_COUNT; i++)
        menu_draw_item(&main_items[i], false);
}

void menu_draw_song_select(void)
{
    menu_draw_title("SELECT SONG");

    for (int i = 0; i < SONG_SELECT_ITEM_COUNT; i++)
        menu_draw_item(&song_select_items[i], false);
}

void menu_draw_playing(void)
{
    menu_draw_title("NOW PLAYING");

    ili9341_set_font_scale(2);
    ili9341_set_text_color(COL_LIGHTGREY, COL_BLACK);
    if (_selected_song == -1){
        ili9341_draw_string(20, 55, "No Song Selected");
    }
    else {
        char song_label[16];
        snprintf(song_label, sizeof(song_label), "SONG %d", _selected_song + 1);
        ili9341_draw_string(20, 55, song_label);
    }

    ili9341_set_font_scale(1);
    ili9341_set_text_color(COL_MIDGREY, COL_BLACK);
    ili9341_draw_string(20, 80, "Playing...");

    for (int i = 0; i < PLAYING_ITEM_COUNT; i++)
        menu_draw_item(&playing_items[i], false);
}

void menu_draw_settings(void)
{
    menu_draw_title("SETTINGS");

    for (int i = 0; i < SETTINGS_ITEM_COUNT; i++)
        menu_draw_item(&settings_items[i], false);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC – INIT & UPDATE
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_init(void)
{
    _cur_screen       = SCREEN_MAIN_MENU;
    _prev_screen      = SCREEN_NONE;
    _stack_top        = 0;
    _touch_state      = TOUCH_IDLE;
    _touch_item_index = -1;
    _no_touch_count   = 0;
}

void menu_update(void)
{
    /* Suppress all SPI activity during active PID to avoid blocking the control loop */
    UARTMode mode = MotorControl_GetMode();
    if (mode == MODE_PID_ACTIVE || mode == MODE_PID_RECEIVE)
        return;

    /* 1. Read touch */
    TouchPoint tp = {0};
    xpt2046_read(&tp);

    /* 2. Process touch state machine */
    _process_touch(&tp);

    /* 3. Full redraw only on screen transition */
    if (_cur_screen != _prev_screen) {
        _redraw_screen(_cur_screen);
        _prev_screen = _cur_screen;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CALLBACK STUBS – replace with your actual logic
 * ═══════════════════════════════════════════════════════════════════════════ */

static void cb_start_playing(void)
{
    if (_selected_song == -1) {
        return;
    }
}

static void cb_stop_playing(void)
{
    /* TODO: stop motor / playback sequence */
    menu_navigate_back();
}

static void cb_save_settings(void)
{
    /* TODO: persist settings to SD card or flash */
}

static void cb_select_song_0(void) {
    _selected_song = 0;
    for (int i = 0; i < 3; i++)   /* redraw only the 3 song buttons */
        menu_draw_item(&song_select_items[i], i == _selected_song);
}
static void cb_select_song_1(void) {
    _selected_song = 1;
    for (int i = 0; i < 3; i++)
        menu_draw_item(&song_select_items[i], i == _selected_song);
}
static void cb_select_song_2(void) {
    _selected_song = 2;
    for (int i = 0; i < 3; i++)
        menu_draw_item(&song_select_items[i], i == _selected_song);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TEST SCREEN – DRAW
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_draw_test(void)
{
    _enc_text[0] = '\0';   /* clear stale reading on fresh draw */
    _mtest_state = MTEST_OFF;
    MotorControl_SetTestStop();
    test_items[5].label     = _mtest_labels[MTEST_OFF];
    test_items[5].col_normal = _mtest_colors[MTEST_OFF];
    menu_draw_title("TEST");

    /* Sync button colors to current finger state before drawing */
    for (int i = 0; i < 5; i++)
        test_items[i].col_normal = _finger_on[i] ? COL_GREEN : COL_MIDGREY;

    ili9341_set_font_scale(1);
    ili9341_set_text_color(COL_LIGHTGREY, COL_BLACK);
    ili9341_draw_string(10, 48, "FINGERS");
    ili9341_draw_string(10, 172, "MOTOR / ENC");

    for (int i = 0; i < TEST_ITEM_COUNT; i++)
        menu_draw_item(&test_items[i], false);

    /* Encoder reading display area (between motor buttons and BACK) */
    ili9341_set_font_scale(1);
    ili9341_set_text_color(COL_LIGHTGREY, COL_BLACK);
    ili9341_draw_string(10, 237, _enc_text);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TEST SCREEN – CALLBACKS
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Weak stub — main.c provides the strong version that drives the GPIO */
__weak void menu_test_finger(int idx, bool on) { (void)idx; (void)on; }

static void _toggle_finger(int idx)
{
    _finger_on[idx] = !_finger_on[idx];
    menu_test_finger(idx, _finger_on[idx]);
    test_items[idx].col_normal = _finger_on[idx] ? COL_GREEN : COL_MIDGREY;
    menu_draw_item(&test_items[idx], false);
}

static void cb_finger_1(void) { _toggle_finger(0); }
static void cb_finger_2(void) { _toggle_finger(1); }
static void cb_finger_3(void) { _toggle_finger(2); }
static void cb_finger_4(void) { _toggle_finger(3); }
static void cb_finger_5(void) { _toggle_finger(4); }

static void cb_motor_cycle(void)
{
    _mtest_state = (_mtest_state + 1) % 4;

    switch (_mtest_state) {
        case MTEST_OFF:    MotorControl_SetTestStop();   break;
        case MTEST_CW:     MotorControl_SetTestCW();     break;
        case MTEST_CCW:    MotorControl_SetTestCCW();    break;
        case MTEST_SWITCH: MotorControl_SetTestSwitch(); break;
    }

    test_items[5].label      = _mtest_labels[_mtest_state];
    test_items[5].col_normal = _mtest_colors[_mtest_state];
    menu_draw_item(&test_items[5], false);
}

static void cb_enc_pos(void)
{
    float lin = MotorControl_ReadLinPos();
    float cm  = lin * 100.0f;

    /* Format floats manually — newlib-nano strips %f by default */
    int m_int  = (int)lin;
    int m_frac = (int)((lin  - (float)m_int) * 10000.0f);
    int c_int  = (int)cm;
    int c_frac = (int)((cm   - (float)c_int) * 100.0f);
    if (m_frac < 0) m_frac = -m_frac;
    if (c_frac < 0) c_frac = -c_frac;

    snprintf(_enc_text, sizeof(_enc_text), "Lin:%d.%04d m (%d.%02d cm)",
             m_int, m_frac, c_int, c_frac);

    ili9341_fill_rect(10, 235, 220, 10, COL_BLACK);
    ili9341_set_font_scale(1);
    ili9341_set_text_color(COL_LIGHTGREY, COL_BLACK);
    ili9341_draw_string(10, 237, _enc_text);
}

static void cb_motor_home(void) { MotorControl_Home(); }

static void cb_song_test(void)  { MotorControl_Home(); MotorControl_PlaySong(); }
