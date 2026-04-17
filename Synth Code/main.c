#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

#include "uart.h"
#include "spi_dac.h"
#include "synth.h"
#include "inputs.h"
#include "notes.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/*
 * Buzzer-debug note map chosen to be clearly different on a passive piezo.
 * You can swap these back later if you want the original guitar-string map.
 */
static const uint8_t fret_note_map[5] = {
    GUITAR_NOTE_C3,   /* GREEN  */
    GUITAR_NOTE_E3,   /* RED    */
    GUITAR_NOTE_G3,   /* YELLOW */
    GUITAR_NOTE_B3,   /* BLUE   */
    GUITAR_NOTE_E4,   /* ORANGE */
};

static volatile uint8_t g_active_fret           = FRET_NONE;
static volatile uint8_t g_fret_changed          = 0U;
static volatile uint8_t g_strum_pressed         = 0U;
static volatile uint8_t g_strum_released        = 0U;
static volatile uint8_t g_button_press_flags    = 0U;
static volatile uint8_t g_button_release_flags  = 0U;

static uint8_t g_strum_down = 0U;

static const char *fret_name(uint8_t fret)
{
    switch (fret) {
    case 0U: return "GREEN";
    case 1U: return "RED";
    case 2U: return "YELLOW";
    case 3U: return "BLUE";
    case 4U: return "ORANGE";
    default: return "NONE";
    }
}

static const char *note_name_for_fret(uint8_t fret)
{
    switch (fret) {
    case 0U: return "C3";
    case 1U: return "E3";
    case 2U: return "G3";
    case 3U: return "B3";
    case 4U: return "E4";
    default: return "NONE";
    }
}

/* 1 ms system tick for debounce. */
static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    OCR0A  = 249U;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
    inputs_tick();
}

void on_fret_change(uint8_t fret)
{
    g_active_fret  = fret;
    g_fret_changed = 1U;
}

void on_button_press(uint8_t fret)
{
    if (fret < 5U) {
        g_button_press_flags |= (uint8_t)(1U << fret);
    }
}

void on_button_release(uint8_t fret)
{
    if (fret < 5U) {
        g_button_release_flags |= (uint8_t)(1U << fret);
    }
}

void on_strum_press(void)
{
    g_strum_pressed = 1U;
}

void on_strum_release(void)
{
    g_strum_released = 1U;
}

int main(void)
{
    uart_init();
    printf("\r\nGuitar Hero input debug start\r\n");
    printf("Watching GREEN, RED, YELLOW, BLUE, ORANGE, and STRUM\r\n");

    pwm_audio_init();
    synth_init();
    inputs_init();
    timer0_init();

    printf("System initialized\r\n");
    printf("PB2 buzzer mode: silent until strum, then play selected note\r\n");

    sei();

    for (;;) {
        uint8_t press_flags;
        uint8_t release_flags;

        cli();
        press_flags = g_button_press_flags;
        g_button_press_flags = 0U;
        release_flags = g_button_release_flags;
        g_button_release_flags = 0U;
        sei();

        for (uint8_t i = 0U; i < 5U; i++) {
            if (press_flags & (1U << i)) {
                printf("%s pressed -> %s\r\n", fret_name(i), note_name_for_fret(i));
            }
            if (release_flags & (1U << i)) {
                printf("%s released\r\n", fret_name(i));
            }
        }

        if (g_fret_changed) {
            uint8_t fret;
            cli();
            fret = g_active_fret;
            g_fret_changed = 0U;
            sei();

            printf("Active fret: %s\r\n", fret_name(fret));

            /* If the strum bar is being held, change the sounding note immediately. */
            if (g_strum_down) {
                if (fret < 5U) {
                    synth_set_note(fret_note_map[fret]);
                    printf("Now playing %s\r\n", note_name_for_fret(fret));
                } else {
                    synth_mute();
                    printf("No fret held, output muted\r\n");
                }
            }
        }

        if (g_strum_pressed) {
            cli();
            g_strum_pressed = 0U;
            sei();

            g_strum_down = 1U;
            printf("STRUM pressed\r\n");

            if (g_active_fret < 5U) {
                synth_set_note(fret_note_map[g_active_fret]);
                printf("Playing %s\r\n", note_name_for_fret(g_active_fret));
            } else {
                synth_mute();
                printf("No fret held, output muted\r\n");
            }
        }

        if (g_strum_released) {
            cli();
            g_strum_released = 0U;
            sei();

            g_strum_down = 0U;
            printf("STRUM released\r\n");
            synth_mute();
        }
    }
}
