#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#include "synth.h"
#include "spi_dac.h"
#include "notes.h"

#define VIBRATO_MAX_STEP  8

/* 256-entry sine wavetable, unsigned 0..65535. */
static const uint16_t sine_table[256] PROGMEM = {
    32768, 33572, 34375, 35179, 35981, 36779, 37576, 38370,
    39161, 39948, 40731, 41508, 42280, 43047, 43808, 44562,
    45307, 46046, 46778, 47501, 48215, 48919, 49614, 50299,
    50973, 51637, 52288, 52929, 53557, 54172, 54774, 55363,
    55938, 56500, 57048, 57581, 58099, 58601, 59088, 59559,
    60014, 60453, 60876, 61280, 61668, 62038, 62391, 62726,
    63043, 63342, 63622, 63884, 64127, 64351, 64556, 64742,
    64909, 65056, 65184, 65292, 65381, 65453, 65502, 65525,
    65535, 65525, 65502, 65453, 65381, 65292, 65184, 65056,
    64909, 64742, 64556, 64351, 64127, 63884, 63622, 63342,
    63043, 62726, 62391, 62038, 61668, 61280, 60876, 60453,
    60014, 59559, 59088, 58601, 58099, 57581, 57048, 56500,
    55938, 55363, 54774, 54172, 53557, 52929, 52288, 51637,
    50973, 50299, 49614, 48919, 48215, 47501, 46778, 46046,
    45307, 44562, 43808, 43047, 42280, 41508, 40731, 39948,
    39161, 38370, 37576, 36779, 35981, 35179, 34375, 33572,
    32767, 31963, 31160, 30356, 29554, 28756, 27959, 27165,
    26374, 25587, 24804, 24027, 23255, 22488, 21727, 20973,
    20228, 19489, 18757, 18034, 17320, 16616, 15921, 15236,
    14562, 13898, 13247, 12606, 11978, 11363, 10761, 10172,
     9597,  9035,  8487,  7954,  7436,  6934,  6447,  5976,
     5521,  5082,  4659,  4255,  3867,  3497,  3144,  2809,
     2492,  2193,  1913,  1651,  1408,  1184,   979,   793,
      626,   479,   351,   243,   154,    82,    33,    10,
        0,    10,    33,    82,   154,   243,   351,   479,
      626,   793,   979,  1184,  1408,  1651,  1913,  2193,
     2492,  2809,  3144,  3497,  3867,  4255,  4659,  5082,
     5521,  5976,  6447,  6934,  7436,  7954,  8487,  9035,
     9597, 10172, 10761, 11363, 11978, 12606, 13247, 13898,
    14562, 15236, 15921, 16616, 17320, 18034, 18757, 19489,
    20228, 20973, 21727, 22488, 23255, 24027, 24804, 25587,
    26374, 27165, 27959, 28756, 29554, 30356, 31160, 31963
};

/*
 * Chord map:
 *   0 = D4 chord  = D4, F#4, A4
 *   1 = F4 chord  = F4, A4, C5
 *   2 = G4 chord  = G4, B4, D5
 *   3 = Ab4 chord = G#4, C5, D#5
 *   4 = D5 chord  = D5, F#5, A5
 */
static const uint8_t chord_notes[NUM_CHORDS][CHORD_TONES] PROGMEM = {
    {GUITAR_NOTE_D4,  GUITAR_NOTE_A4,  GUITAR_NOTE_D5},
    {GUITAR_NOTE_F4,  GUITAR_NOTE_C5,  GUITAR_NOTE_F5},
    {GUITAR_NOTE_G4,  GUITAR_NOTE_D5,  GUITAR_NOTE_G5},
    {GUITAR_NOTE_GS4, GUITAR_NOTE_DS5, GUITAR_NOTE_GS5},
    {GUITAR_NOTE_D5,  GUITAR_NOTE_A5,  GUITAR_NOTE_D6}
};

static volatile uint32_t g_phase_acc[CHORD_TONES];
static volatile uint32_t g_base_phase_inc[CHORD_TONES];
static volatile uint32_t g_cur_phase_inc[CHORD_TONES];
static volatile uint8_t  g_muted = 1U;

/* Vibrato only. No whammy bend. */
static volatile uint8_t  g_vibrato_depth_percent = 0U;
static volatile int8_t   g_vibrato_pos = 0;
static volatile int8_t   g_vibrato_dir = 1;

static void synth_apply_pitch(void)
{
    int32_t delta_q16;
    uint32_t scalar_q16;

    delta_q16 = ((int32_t)65536L * (int32_t)g_vibrato_depth_percent * (int32_t)g_vibrato_pos)
              / (100L * VIBRATO_MAX_STEP);
    scalar_q16 = (uint32_t)((int32_t)65536L + delta_q16);

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        uint64_t prod = (uint64_t)g_base_phase_inc[i] * (uint64_t)scalar_q16;
        g_cur_phase_inc[i] = (uint32_t)(prod >> 16);
    }
}

/* Timer2 Compare A ISR at 31.25 kHz: mix 3 oscillators into one PWM stream. */
ISR(TIMER2_COMPA_vect)
{
    int32_t mixed = 0;

    if (g_muted) {
        return;
    }

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        g_phase_acc[i] += g_cur_phase_inc[i];
        mixed += (int32_t)pgm_read_word(&sine_table[(uint8_t)(g_phase_acc[i] >> 24)]) - 32768L;
    }

    mixed /= 4;
    mixed += 32768L;

    if (mixed < 0L) {
        mixed = 0L;
    } else if (mixed > 65535L) {
        mixed = 65535L;
    }

    pwm_audio_write((uint16_t)mixed);
}

void synth_init(void)
{
    uint8_t sreg = SREG;
    cli();

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        g_phase_acc[i] = 0UL;
        g_base_phase_inc[i] = 0UL;
        g_cur_phase_inc[i] = 0UL;
    }

    g_muted = 1U;
    g_vibrato_depth_percent = 0U;
    g_vibrato_pos = 0;
    g_vibrato_dir = 1;

    TCCR2A = (1 << WGM21);
    TCCR2B = 0x00;
    OCR2A  = 127U;
    TCNT2  = 0U;
    TIFR2  = (1 << OCF2A);
    TIMSK2 = (1 << OCIE2A);
    TCCR2B = (1 << CS21);   /* prescaler = 8 -> 31.25 kHz */

    pwm_audio_write(32768U);
    pwm_audio_disable();

    SREG = sreg;
}

void synth_set_chord(uint8_t chord_idx)
{
    if (chord_idx >= NUM_CHORDS) {
        return;
    }

    uint8_t sreg = SREG;
    cli();

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        uint8_t note_idx = pgm_read_byte(&chord_notes[chord_idx][i]);
        g_phase_acc[i] = 0UL;
        g_base_phase_inc[i] = pgm_read_dword(&note_phase_inc[note_idx]) << 1;
    }

    g_vibrato_pos = 0;
    g_vibrato_dir = 1;
    synth_apply_pitch();
    g_muted = 0U;
    pwm_audio_enable();

    SREG = sreg;
}

void synth_mute(void)
{
    uint8_t sreg = SREG;
    cli();

    g_muted = 1U;
    pwm_audio_disable();

    SREG = sreg;
}

void synth_set_vibrato_depth(uint8_t depth_percent)
{
    uint8_t sreg = SREG;
    cli();
    g_vibrato_depth_percent = depth_percent;
    synth_apply_pitch();
    SREG = sreg;
}

void synth_reset_vibrato(void)
{
    uint8_t sreg = SREG;
    cli();
    g_vibrato_pos = 0;
    g_vibrato_dir = 1;
    synth_apply_pitch();
    SREG = sreg;
}

void synth_vibrato_tick(void)
{
    uint8_t sreg = SREG;
    cli();

    if (g_vibrato_depth_percent == 0U) {
        g_vibrato_pos = 0;
        g_vibrato_dir = 1;
        synth_apply_pitch();
        SREG = sreg;
        return;
    }

    g_vibrato_pos += g_vibrato_dir;

    if (g_vibrato_pos >= VIBRATO_MAX_STEP) {
        g_vibrato_pos = VIBRATO_MAX_STEP;
        g_vibrato_dir = -1;
    } else if (g_vibrato_pos <= -VIBRATO_MAX_STEP) {
        g_vibrato_pos = -VIBRATO_MAX_STEP;
        g_vibrato_dir = 1;
    }

    synth_apply_pitch();
    SREG = sreg;
}
