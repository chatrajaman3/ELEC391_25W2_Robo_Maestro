#ifndef MENU_H
#define MENU_H

#include <stdint.h>
#include <stdbool.h>
#include "ili9341.h"
#include "xpt2046.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * SCREEN DIMENSIONS  (ROTATION_0 → portrait 240×320)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define SCREEN_W  240
#define SCREEN_H  320

/* ═══════════════════════════════════════════════════════════════════════════
 * COLOUR PALETTE  (RGB565)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define COL_BLACK        RGB565(  0,   0,   0)
#define COL_WHITE        RGB565(255, 255, 255)
#define COL_DARKGREY     RGB565( 30,  30,  30)
#define COL_MIDGREY      RGB565( 80,  80,  80)
#define COL_LIGHTGREY    RGB565(180, 180, 180)

#define COL_BLUE         RGB565( 25,  80, 200)
#define COL_BLUE_PRESS   RGB565( 70, 140, 255)
#define COL_GREEN        RGB565( 20, 150,  50)
#define COL_GREEN_PRESS  RGB565( 60, 210,  90)
#define COL_RED          RGB565(200,  30,  30)
#define COL_RED_PRESS    RGB565(255,  80,  80)
#define COL_ORANGE       RGB565(210, 100,   0)
#define COL_ORANGE_PRESS RGB565(255, 160,  40)

/* ═══════════════════════════════════════════════════════════════════════════
 * FONT METRICS
 * ═══════════════════════════════════════════════════════════════════════════ */
#define FONT_BASE_W   6   /* 5px glyph + 1px gap */
#define FONT_BASE_H   7

#define TEXT_W(chars, scale)  ((chars) * FONT_BASE_W * (scale))
#define TEXT_H(scale)         (FONT_BASE_H * (scale))

/* ═══════════════════════════════════════════════════════════════════════════
 * SCREEN STATE ENUM
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    SCREEN_NONE = -1,
    SCREEN_MAIN_MENU,
    SCREEN_SONG_SELECT,
    SCREEN_PLAYING,
    SCREEN_SETTINGS,
} ScreenState;

/* ═══════════════════════════════════════════════════════════════════════════
 * MENU ITEM
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint16_t     x, y;
    uint16_t     w, h;
    const char  *label;
    uint8_t      label_scale;
    uint16_t     col_normal;
    uint16_t     col_pressed;
    ScreenState  target_screen;
    void       (*on_select)(void);
} MenuItem;

/* ═══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

void menu_init(void);
void menu_update(void);

void menu_navigate_to(ScreenState s);
void menu_navigate_back(void);

void menu_draw_item(const MenuItem *item, bool pressed);
void menu_draw_title(const char *title);

void menu_draw_main(void);
void menu_draw_song_select(void);
void menu_draw_playing(void);
void menu_draw_settings(void);

#endif /* MENU_H */
