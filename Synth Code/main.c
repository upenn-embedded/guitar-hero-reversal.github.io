#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

#include "uart.h"
#include "spi_dac.h"
#include "synth.h"
#include "inputs.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define WHAMMY_SCAN_MS             10UL
#define VIBRATO_TICK_MS             8UL
#define WHAMMY_ADC_CUTOFF         614U   /* ~3.00 V with AVcc = 5 V */
#define WHAMMY_ADC_MAX            941U   /* ~4.60 V with AVcc = 5 V */
#define WHAMMY_MAX_VIBRATO_PCT      6U

static const uint8_t fret_chord_map[5] = {
    CHORD_D4,
    CHORD_F4,
    CHORD_G4,
    CHORD_AB4,
    CHORD_D5,
};

static volatile uint8_t  g_active_fret          = FRET_NONE;
static volatile uint8_t  g_fret_changed         = 0U;
static volatile uint8_t  g_strum_pressed        = 0U;
static volatile uint8_t  g_strum_released       = 0U;
static volatile uint8_t  g_button_press_flags   = 0U;
static volatile uint8_t  g_button_release_flags = 0U;
static volatile uint32_t g_ms_tick              = 0UL;

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

static const char *chord_name_for_fret(uint8_t fret)
{
    switch (fret) {
    case 0U: return "D4 chord";
    case 1U: return "F4 chord";
    case 2U: return "G4 chord";
    case 3U: return "Ab4 chord";
    case 4U: return "D5 chord";
    default: return "NONE";
    }
}

static uint8_t vibrato_depth_from_adc(uint16_t adc)
{
    uint32_t scaled;

    if (adc < WHAMMY_ADC_CUTOFF) {
        return 0U;
    }
    if (adc >= WHAMMY_ADC_MAX) {
        return WHAMMY_MAX_VIBRATO_PCT;
    }

    scaled = ((uint32_t)(adc - WHAMMY_ADC_CUTOFF) * WHAMMY_MAX_VIBRATO_PCT)
           / (uint32_t)(WHAMMY_ADC_MAX - WHAMMY_ADC_CUTOFF);

    return (uint8_t)scaled;
}

static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    OCR0A  = 249U;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
    g_ms_tick++;
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
    uint32_t last_vibrato_ms = 0UL;
    uint32_t last_whammy_ms  = 0UL;
    uint8_t  last_vibrato_pct = 0xFFU;

    uart_init();
    printf("\r\nGuitar Hero input debug start\r\n");
    printf("Watching GREEN, RED, YELLOW, BLUE, ORANGE, STRUM, and WHAMMY\r\n");

    pwm_audio_init();
    synth_init();
    synth_set_vibrato_depth(0U);
    inputs_init();
    timer0_init();

    printf("System initialized\r\n");
    printf("PB2 speaker mode: true 3-note chords in one PWM stream\r\n");
    printf("Whammy vibrato cutoff: below 3.0 V = no vibrato\r\n");

    sei();

    for (;;) {
        uint8_t press_flags;
        uint8_t release_flags;
        uint32_t now_ms;

        cli();
        press_flags = g_button_press_flags;
        g_button_press_flags = 0U;
        release_flags = g_button_release_flags;
        g_button_release_flags = 0U;
        now_ms = g_ms_tick;
        sei();

        for (uint8_t i = 0U; i < 5U; i++) {
            if (press_flags & (1U << i)) {
                printf("%s pressed -> %s\r\n", fret_name(i), chord_name_for_fret(i));
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

            if (g_strum_down) {
                if (fret < 5U) {
                    synth_set_chord(fret_chord_map[fret]);
                    printf("Now playing %s\r\n", chord_name_for_fret(fret));
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
                synth_set_chord(fret_chord_map[g_active_fret]);
                printf("Playing %s\r\n", chord_name_for_fret(g_active_fret));
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

        if ((uint32_t)(now_ms - last_whammy_ms) >= WHAMMY_SCAN_MS) {
            uint16_t whammy_adc;
            uint8_t vibrato_pct;

            last_whammy_ms = now_ms;
            inputs_adc_scan();
            whammy_adc = inputs_whammy;
            vibrato_pct = vibrato_depth_from_adc(whammy_adc);

            if (vibrato_pct == 0U) {
                synth_set_vibrato_depth(0U);
                synth_reset_vibrato();
            } else {
                synth_set_vibrato_depth(vibrato_pct);
            }

            if (vibrato_pct != last_vibrato_pct) {
                last_vibrato_pct = vibrato_pct;
                //printf("Whammy ADC=%u -> vibrato depth=%u%%\r\n",
                       //(unsigned)whammy_adc,
                       //(unsigned)vibrato_pct);
            }
        }

        if (g_strum_down && last_vibrato_pct > 0U &&
            (uint32_t)(now_ms - last_vibrato_ms) >= VIBRATO_TICK_MS) {
            last_vibrato_ms = now_ms;
            synth_vibrato_tick();
        }
    }
}
