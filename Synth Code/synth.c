#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "synth.h"
#include "spi_dac.h"
#include "notes.h"

/*
 * Correct OCR1A table for F_CPU = 16 MHz, Timer1 prescaler = 8.
 * f_out = F_CPU / (2 * 8 * (1 + OCR1A))
 * Covers E2 through E4.
 */
static const uint16_t note_ocr1a[NUM_GUITAR_NOTES] PROGMEM = {
    12134U, 11453U, 10810U, 10203U,  9630U,
     9090U,  8580U,  8098U,  7644U,  7214U,
     6810U,  6427U,  6066U,  5726U,  5404U,
     5101U,  4815U,  4544U,  4289U,  4049U,
     3821U,  3607U,  3404U,  3213U,  3033U
};

void synth_init(void)
{
    pwm_audio_write(3033U);  /* safe default */
    pwm_audio_disable();
}

void synth_set_note(uint8_t note_idx)
{
    if (note_idx >= NUM_GUITAR_NOTES) {
        return;
    }

    uint8_t sreg = SREG;
    cli();
    pwm_audio_write((uint16_t)pgm_read_word(&note_ocr1a[note_idx]));
    pwm_audio_enable();
    SREG = sreg;
}

void synth_mute(void)
{
    uint8_t sreg = SREG;
    cli();
    pwm_audio_disable();
    SREG = sreg;
}
