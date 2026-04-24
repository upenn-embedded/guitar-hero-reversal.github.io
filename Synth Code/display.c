/*
 * display.c ? ST7796S 480?320 LCD driver (software SPI)
 * ESE3500 Final Project ? Guitar Synthesizer Controller
 * Team 3: Synth Specialist (Guitar Hero Edition)
 * University of Pennsylvania ? Spring 2026
 *
 * Authors: Adam Shalabi, Brandon Parkansky, Panos Dimtsoudis
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

#include "display.h"
#include "notes.h"

#define TFT_MOSI_HI()  (PORTB |=  (1U << PB4))
#define TFT_MOSI_LO()  (PORTB &= ~(1U << PB4))
#define TFT_SCK_HI()   (PORTB |=  (1U << PB5))
#define TFT_SCK_LO()   (PORTB &= ~(1U << PB5))
#define TFT_CS_HI()    (PORTB |=  (1U << PB1))
#define TFT_CS_LO()    (PORTB &= ~(1U << PB1))
#define TFT_DC_HI()    (PORTC |=  (1U << PC3))   
#define TFT_DC_LO()    (PORTC &= ~(1U << PC3))   
#define TFT_RST_HI()   (PORTC |=  (1U << PC4))
#define TFT_RST_LO()   (PORTC &= ~(1U << PC4))

#define ST_SWRESET  0x01U
#define ST_SLPOUT   0x11U
#define ST_NORON    0x13U
#define ST_DISPON   0x29U
#define ST_CASET    0x2AU   
#define ST_RASET    0x2BU   
#define ST_RAMWR    0x2CU   
#define ST_MADCTL   0x36U   
#define ST_COLMOD   0x3AU   

#define MADCTL_VAL  0xE8U

#define BGR565(b, g, r) \
    ((uint16_t)(((uint16_t)(b) << 11) | ((uint16_t)(g) << 5) | (uint16_t)(r)))

#define COLOR_BG         BGR565( 0,  0, 31)   
#define COLOR_WHITE      BGR565(31, 63, 31)   
#define COLOR_DIVIDER    BGR565(31, 63, 31)   
#define COLOR_RED        BGR565(31,  0,  0)   
#define COLOR_ORANGE     BGR565(31, 41,  0)   
#define COLOR_YELLOW     BGR565(31, 63,  0)   
#define COLOR_GREEN      BGR565( 0, 63,  0)   
#define COLOR_BLUE_BOX   BGR565( 0, 20, 28)   
#define COLOR_SCROLL_BG  BGR565( 0,  8, 18)   
#define COLOR_SCROLL_HL  BGR565( 2, 28, 26)   
#define COLOR_UNDERLINE  BGR565(31, 63, 31)   

#define SCREEN_W    480U
#define SCREEN_H    320U

#define BTN_X       5U
#define BTN_W       180U
#define BTN_H       58U
#define BTN_GAP     4U      
#define BTN_Y0      5U      
#define BTN_MARGIN  8U      

#define SCR_X       196U
#define SCR_Y       5U
#define SCR_W       279U
#define SCR_H       310U
#define SCR_SLOTS   5U
#define SCR_SLOT_H  (SCR_H / SCR_SLOTS)   

#define SCR_NOTE_MIN  GUITAR_NOTE_E2   
#define SCR_NOTE_MAX  GUITAR_NOTE_C6   
#define SCR_NOTE_SPAN ((uint8_t)(SCR_NOTE_MAX - SCR_NOTE_MIN + 1U))  
#define SCALE_BTN   2U   
#define SCALE_N     2U   
#define SCALE_C     3U   

#define SCR_ERASE_W ((uint16_t)(4U * 6U * SCALE_C))   
#define SCR_ERASE_H ((uint16_t)(8U * SCALE_C)) 

static const uint8_t font5x8[95][5] PROGMEM = {
    {0x00,0x00,0x00,0x00,0x00}, 
    {0x00,0x00,0x5F,0x00,0x00}, 
    {0x00,0x07,0x00,0x07,0x00}, 
    {0x14,0x7F,0x14,0x7F,0x14}, 
    {0x24,0x2A,0x7F,0x2A,0x12}, 
    {0x23,0x13,0x08,0x64,0x62}, 
    {0x36,0x49,0x55,0x22,0x50}, 
    {0x00,0x05,0x03,0x00,0x00}, 
    {0x00,0x1C,0x22,0x41,0x00}, 
    {0x00,0x41,0x22,0x1C,0x00}, 
    {0x14,0x08,0x3E,0x08,0x14}, 
    {0x08,0x08,0x3E,0x08,0x08}, 
    {0x00,0x50,0x30,0x00,0x00}, 
    {0x08,0x08,0x08,0x08,0x08}, 
    {0x00,0x60,0x60,0x00,0x00}, 
    {0x20,0x10,0x08,0x04,0x02}, 
    {0x3E,0x51,0x49,0x45,0x3E}, 
    {0x00,0x42,0x7F,0x40,0x00}, 
    {0x42,0x61,0x51,0x49,0x46}, 
    {0x21,0x41,0x45,0x4B,0x31}, 
    {0x18,0x14,0x12,0x7F,0x10}, 
    {0x27,0x45,0x45,0x45,0x39}, 
    {0x3C,0x4A,0x49,0x49,0x30}, 
    {0x01,0x71,0x09,0x05,0x03}, 
    {0x36,0x49,0x49,0x49,0x36}, 
    {0x06,0x49,0x49,0x29,0x1E}, 
    {0x00,0x36,0x36,0x00,0x00}, 
    {0x00,0x56,0x36,0x00,0x00}, 
    {0x08,0x14,0x22,0x41,0x00}, 
    {0x14,0x14,0x14,0x14,0x14}, 
    {0x00,0x41,0x22,0x14,0x08}, 
    {0x02,0x01,0x51,0x09,0x06}, 
    {0x32,0x49,0x79,0x41,0x3E}, 
    {0x7E,0x11,0x11,0x11,0x7E}, 
    {0x7F,0x49,0x49,0x49,0x36}, 
    {0x3E,0x41,0x41,0x41,0x22}, 
    {0x7F,0x41,0x41,0x22,0x1C}, 
    {0x7F,0x49,0x49,0x49,0x41}, 
    {0x7F,0x09,0x09,0x09,0x01}, 
    {0x3E,0x41,0x49,0x49,0x7A}, 
    {0x7F,0x08,0x08,0x08,0x7F}, 
    {0x00,0x41,0x7F,0x41,0x00}, 
    {0x20,0x40,0x41,0x3F,0x01}, 
    {0x7F,0x08,0x14,0x22,0x41}, 
    {0x7F,0x40,0x40,0x40,0x40}, 
    {0x7F,0x02,0x04,0x02,0x7F}, 
    {0x7F,0x04,0x08,0x10,0x7F}, 
    {0x3E,0x41,0x41,0x41,0x3E}, 
    {0x7F,0x09,0x09,0x09,0x06}, 
    {0x3E,0x41,0x51,0x21,0x5E}, 
    {0x7F,0x09,0x19,0x29,0x46}, 
    {0x46,0x49,0x49,0x49,0x31}, 
    {0x01,0x01,0x7F,0x01,0x01}, 
    {0x3F,0x40,0x40,0x40,0x3F}, 
    {0x1F,0x20,0x40,0x20,0x1F}, 
    {0x3F,0x40,0x38,0x40,0x3F}, 
    {0x63,0x14,0x08,0x14,0x63}, 
    {0x07,0x08,0x70,0x08,0x07}, 
    {0x61,0x51,0x49,0x45,0x43}, 
    {0x00,0x7F,0x41,0x41,0x00}, 
    {0x02,0x04,0x08,0x10,0x20}, 
    {0x00,0x41,0x41,0x7F,0x00}, 
    {0x04,0x02,0x01,0x02,0x04}, 
    {0x40,0x40,0x40,0x40,0x40}, 
    {0x00,0x01,0x02,0x04,0x00}, 
    {0x20,0x54,0x54,0x54,0x78}, 
    {0x7F,0x48,0x44,0x44,0x38}, 
    {0x38,0x44,0x44,0x44,0x20}, 
    {0x38,0x44,0x44,0x48,0x7F}, 
    {0x38,0x54,0x54,0x54,0x18}, 
    {0x08,0x7E,0x09,0x01,0x02}, 
    {0x0C,0x52,0x52,0x52,0x3E}, 
    {0x7F,0x08,0x04,0x04,0x78}, 
    {0x00,0x44,0x7D,0x40,0x00}, 
    {0x20,0x40,0x44,0x3D,0x00}, 
    {0x7F,0x10,0x28,0x44,0x00}, 
    {0x00,0x41,0x7F,0x40,0x00}, 
    {0x7C,0x04,0x18,0x04,0x78}, 
    {0x7C,0x08,0x04,0x04,0x78}, 
    {0x38,0x44,0x44,0x44,0x38}, 
    {0x7C,0x14,0x14,0x14,0x08}, 
    {0x08,0x14,0x14,0x18,0x7C}, 
    {0x7C,0x08,0x04,0x04,0x08}, 
    {0x48,0x54,0x54,0x54,0x20}, 
    {0x04,0x3F,0x44,0x40,0x20}, 
    {0x3C,0x40,0x40,0x40,0x7C}, 
    {0x1C,0x20,0x40,0x20,0x1C}, 
    {0x3C,0x40,0x30,0x40,0x3C}, 
    {0x44,0x28,0x10,0x28,0x44}, 
    {0x0C,0x50,0x50,0x50,0x3C}, 
    {0x44,0x64,0x54,0x4C,0x44}, 
    {0x00,0x08,0x36,0x41,0x00}, 
    {0x00,0x00,0x7F,0x00,0x00}, 
    {0x00,0x41,0x36,0x08,0x00}, 
    {0x08,0x08,0x2A,0x1C,0x08}, 
};

static uint8_t g_btn_notes[5];   
static uint8_t g_sel_btn;        
static uint8_t g_scroll_note;    

static const char btn_labels[5][7] = {
    "RED", "ORANGE", "YELLOW", "GREEN", "BLUE"
};

static const uint16_t btn_colors[5] = {
    COLOR_RED,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_GREEN,
    COLOR_BLUE_BOX,
};

static const uint8_t btn_default_notes[5] = {
    GUITAR_NOTE_D4,    
    GUITAR_NOTE_F4,    
    GUITAR_NOTE_G4,    
    GUITAR_NOTE_GS4,   
    GUITAR_NOTE_D5,    
};

static void spi_write_byte(uint8_t b)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        if (b & 0x80U) { TFT_MOSI_HI(); } else { TFT_MOSI_LO(); }
        TFT_SCK_HI();
        TFT_SCK_LO();
        b = (uint8_t)(b << 1U);
    }
}


static void tft_cmd(uint8_t cmd)
{
    TFT_DC_LO();
    TFT_CS_LO();
    spi_write_byte(cmd);
    TFT_CS_HI();
}


static void tft_data(uint8_t d)
{
    TFT_DC_HI();
    TFT_CS_LO();
    spi_write_byte(d);
    TFT_CS_HI();
}

static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    
    TFT_DC_LO(); TFT_CS_LO(); spi_write_byte(ST_CASET);
    TFT_DC_HI();
    spi_write_byte((uint8_t)(x0 >> 8U)); spi_write_byte((uint8_t)(x0 & 0xFFU));
    spi_write_byte((uint8_t)(x1 >> 8U)); spi_write_byte((uint8_t)(x1 & 0xFFU));
    TFT_CS_HI();

    
    TFT_DC_LO(); TFT_CS_LO(); spi_write_byte(ST_RASET);
    TFT_DC_HI();
    spi_write_byte((uint8_t)(y0 >> 8U)); spi_write_byte((uint8_t)(y0 & 0xFFU));
    spi_write_byte((uint8_t)(y1 >> 8U)); spi_write_byte((uint8_t)(y1 & 0xFFU));
    TFT_CS_HI();
}

static void fill_rect(uint16_t x, uint16_t y,
                       uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0U || h == 0U) { return; }

    set_window(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
   
    TFT_DC_LO(); TFT_CS_LO(); spi_write_byte(ST_RAMWR); TFT_DC_HI();

    uint8_t  hi = (uint8_t)(color >> 8U);
    uint8_t  lo = (uint8_t)(color & 0xFFU);
    uint32_t n  = (uint32_t)w * (uint32_t)h;

    while (n--) {
        spi_write_byte(hi);
        spi_write_byte(lo);
    }
    TFT_CS_HI();
}

static void draw_char(uint16_t x, uint16_t y, char c,
                       uint16_t fg, uint16_t bg, uint8_t scale)
{
    if ((uint8_t)c < 0x20U || (uint8_t)c > 0x7EU) { c = '?'; }
    uint8_t fidx = (uint8_t)c - 0x20U;

    uint16_t cw = (uint16_t)(6U * (uint16_t)scale);   
    uint16_t ch = (uint16_t)(8U * (uint16_t)scale);   

    set_window(x, y, (uint16_t)(x + cw - 1U), (uint16_t)(y + ch - 1U));
    TFT_DC_LO(); TFT_CS_LO(); spi_write_byte(ST_RAMWR); TFT_DC_HI();

    uint8_t fhi = (uint8_t)(fg >> 8U); uint8_t flo = (uint8_t)(fg & 0xFFU);
    uint8_t bhi = (uint8_t)(bg >> 8U); uint8_t blo = (uint8_t)(bg & 0xFFU);

    for (uint16_t r = 0U; r < ch; r++) {
        uint8_t font_row = (uint8_t)(r / (uint16_t)scale);
        for (uint16_t ci = 0U; ci < cw; ci++) {
            uint8_t font_col = (uint8_t)(ci / (uint16_t)scale);
            uint8_t pixel    = 0U;
            if (font_col < 5U) {
                uint8_t col_data = pgm_read_byte(&font5x8[fidx][font_col]);
                pixel = (col_data >> font_row) & 1U;
            }
            if (pixel) { spi_write_byte(fhi); spi_write_byte(flo); }
            else        { spi_write_byte(bhi); spi_write_byte(blo); }
        }
    }
    TFT_CS_HI();
}


static void draw_string(uint16_t x, uint16_t y, const char *s,
                         uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*s) {
        draw_char(x, y, *s, fg, bg, scale);
        x += (uint16_t)(6U * (uint16_t)scale);
        s++;
    }
}


static uint16_t str_px_w(const char *s, uint8_t scale)
{
    return (uint16_t)(strlen(s) * 6U * (uint16_t)scale);
}

static uint16_t btn_top_y(uint8_t idx)
{
    return (uint16_t)(BTN_Y0 + (uint16_t)idx * (uint16_t)(BTN_H + BTN_GAP));
}

static void draw_button_box(uint8_t idx)
{
    uint16_t    bx    = BTN_X;
    uint16_t    by    = btn_top_y(idx);
    uint16_t    bc    = btn_colors[idx];
    const char *label = btn_labels[idx];
    
    fill_rect(bx, by, BTN_W, BTN_H, bc);
    
    uint16_t lh  = (uint16_t)(8U * (uint16_t)SCALE_BTN);
    uint16_t lx  = bx + BTN_MARGIN;
    uint16_t ly  = by + (uint16_t)((BTN_H - lh) / 2U);
    draw_string(lx, ly, label, COLOR_WHITE, bc, SCALE_BTN);

    if (idx == g_sel_btn) {
        uint16_t lw = str_px_w(label, SCALE_BTN);
        
        fill_rect(lx, (uint16_t)(ly + lh + 2U), lw, 2U, COLOR_UNDERLINE);
    }
    
    char note_buf[4];
    note_name_get(g_btn_notes[idx], note_buf);
    uint16_t nw = str_px_w(note_buf, SCALE_BTN);
    uint16_t nx = (uint16_t)(bx + BTN_W - nw - BTN_MARGIN);
    draw_string(nx, ly, note_buf, COLOR_WHITE, bc, SCALE_BTN);
}

static uint16_t slot_top_y(uint8_t slot)
{
    return (uint16_t)(SCR_Y + (uint16_t)slot * (uint16_t)SCR_SLOT_H);
}

static void draw_scroll_slot(uint8_t slot, uint8_t note_idx, uint8_t is_centre)
{
    uint16_t sy  = slot_top_y(slot);
    uint16_t bg  = is_centre ? COLOR_SCROLL_HL : COLOR_SCROLL_BG;
    uint8_t  sc  = is_centre ? SCALE_C : SCALE_N;

    
    fill_rect(SCR_X, sy, SCR_W, (uint16_t)SCR_SLOT_H, bg);

    
    if (slot > 0U) {
        fill_rect(SCR_X, sy, SCR_W, 1U, COLOR_BG);
    }

    
    char buf[4];
    note_name_get(note_idx, buf);
    uint16_t tw = str_px_w(buf, sc);
    uint16_t th = (uint16_t)(8U * (uint16_t)sc);
    uint16_t tx = SCR_X + (uint16_t)((SCR_W - tw) / 2U);
    uint16_t ty = sy    + (uint16_t)((SCR_SLOT_H - th) / 2U);
    draw_string(tx, ty, buf, COLOR_WHITE, bg, sc);

    
    if (is_centre) {
        fill_rect(SCR_X,                             sy,                           SCR_W, 2U, COLOR_WHITE);
        fill_rect(SCR_X,                             (uint16_t)(sy + SCR_SLOT_H - 2U), SCR_W, 2U, COLOR_WHITE);
        fill_rect(SCR_X,                             sy,                           2U, (uint16_t)SCR_SLOT_H, COLOR_WHITE);
        fill_rect((uint16_t)(SCR_X + SCR_W - 2U),   sy,                           2U, (uint16_t)SCR_SLOT_H, COLOR_WHITE);
    }
}





static void draw_scroll_wheel(void)
{
    for (uint8_t s = 0U; s < SCR_SLOTS; s++) {
        int16_t offset = (int16_t)s - 2;   
        int16_t ni     = (int16_t)g_scroll_note + offset;

        
        int16_t span = (int16_t)SCR_NOTE_SPAN;
        while (ni < (int16_t)SCR_NOTE_MIN) { ni += span; }
        while (ni > (int16_t)SCR_NOTE_MAX) { ni -= span; }

        draw_scroll_slot(s, (uint8_t)ni, (s == 2U) ? 1U : 0U);
    }
}












static void redraw_scroll_text_only(void)
{
    for (uint8_t s = 0U; s < SCR_SLOTS; s++) {
        int16_t offset = (int16_t)s - 2;
        int16_t ni     = (int16_t)g_scroll_note + offset;

        
        int16_t span = (int16_t)SCR_NOTE_SPAN;
        while (ni < (int16_t)SCR_NOTE_MIN) { ni += span; }
        while (ni > (int16_t)SCR_NOTE_MAX) { ni -= span; }

        uint16_t sy        = slot_top_y(s);
        uint8_t  is_centre = (s == 2U) ? 1U : 0U;
        uint16_t bg        = is_centre ? COLOR_SCROLL_HL : COLOR_SCROLL_BG;
        uint8_t  sc        = is_centre ? SCALE_C : SCALE_N;

        


        uint16_t erase_x = SCR_X + (uint16_t)((SCR_W  - SCR_ERASE_W) / 2U);
        uint16_t erase_y = sy    + (uint16_t)((SCR_SLOT_H - SCR_ERASE_H) / 2U);
        fill_rect(erase_x, erase_y, SCR_ERASE_W, SCR_ERASE_H, bg);

        
        char buf[4];
        note_name_get((uint8_t)ni, buf);
        uint16_t tw = str_px_w(buf, sc);
        uint16_t th = (uint16_t)(8U * (uint16_t)sc);
        uint16_t tx = SCR_X + (uint16_t)((SCR_W - tw) / 2U);
        uint16_t ty = sy    + (uint16_t)((SCR_SLOT_H - th) / 2U);
        draw_string(tx, ty, buf, COLOR_WHITE, bg, sc);
    }
}




static void tft_hw_init(void)
{
    
    DDRB |= (1U << PB4) | (1U << PB5) | (1U << PB1);
    DDRC |= (1U << PC3) | (1U << PC4);

    
    TFT_CS_HI();
    TFT_DC_HI();
    TFT_SCK_LO();
    TFT_MOSI_LO();

    
    TFT_RST_HI();
    _delay_ms(5);
    TFT_RST_LO();
    _delay_ms(20);
    TFT_RST_HI();
    _delay_ms(150);   

    
    tft_cmd(ST_SWRESET);
    _delay_ms(150);

    
    tft_cmd(ST_SLPOUT);
    _delay_ms(120);

    tft_cmd(ST_MADCTL);
    tft_data(MADCTL_VAL);

    tft_cmd(ST_COLMOD);
    tft_data(0x55U);

    tft_cmd(ST_NORON);

    tft_cmd(ST_DISPON);
    _delay_ms(25);
}


void display_init(void)
{
    for (uint8_t i = 0U; i < 5U; i++) {
        g_btn_notes[i] = btn_default_notes[i];
    }
    g_sel_btn     = 0U;
    g_scroll_note = GUITAR_NOTE_D4;

    tft_hw_init();

    fill_rect(0U, 0U, SCREEN_W, SCREEN_H, COLOR_BG);

    fill_rect((uint16_t)(BTN_X + BTN_W + 5U), 0U, 2U, SCREEN_H, COLOR_DIVIDER);

    for (uint8_t i = 0U; i < 5U; i++) {
        draw_button_box(i);
    }

    draw_scroll_wheel();
}

void display_force_redraw(void)
{
    fill_rect(0U, 0U, SCREEN_W, SCREEN_H, COLOR_BG);
    fill_rect((uint16_t)(BTN_X + BTN_W + 5U), 0U, 2U, SCREEN_H, COLOR_DIVIDER);

    for (uint8_t i = 0U; i < 5U; i++) {
        draw_button_box(i);
    }
    draw_scroll_wheel();
}

void display_move_button_selection(int8_t dir)
{
    uint8_t old = g_sel_btn;

    if (dir < 0) {
        g_sel_btn = (g_sel_btn == 0U) ? 4U : (uint8_t)(g_sel_btn - 1U);
    } else if (dir > 0) {
        g_sel_btn = (g_sel_btn == 4U) ? 0U : (uint8_t)(g_sel_btn + 1U);
    }

    if (g_sel_btn != old) {
        draw_button_box(old);        
        draw_button_box(g_sel_btn);  
    }
}

void display_move_note_selection(int8_t dir, uint32_t now_ms)
{
    (void)now_ms;   

    if (dir > 0) {
        
        g_scroll_note = (g_scroll_note >= SCR_NOTE_MAX)
                      ? SCR_NOTE_MIN
                      : (uint8_t)(g_scroll_note + 1U);
    } else if (dir < 0) {
        
        g_scroll_note = (g_scroll_note <= SCR_NOTE_MIN)
                      ? SCR_NOTE_MAX
                      : (uint8_t)(g_scroll_note - 1U);
    }

    redraw_scroll_text_only();
}

void display_commit_selected_note(void)
{
    g_btn_notes[g_sel_btn] = g_scroll_note;
    draw_button_box(g_sel_btn);
}

uint8_t display_get_button_note(uint8_t btn)
{
    if (btn >= 5U) { return g_btn_notes[0]; }
    return g_btn_notes[btn];
}

uint8_t display_get_selected_button(void)
{
    return g_sel_btn;
}

uint8_t display_get_selected_note(void)
{
    return g_scroll_note;
}