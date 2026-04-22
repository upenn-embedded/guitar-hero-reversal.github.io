#include <avr/io.h>
#include "spi_dac.h"

/*
 * PB2 / OC1B high-rate PWM audio output for the final speaker path.
 * Timer1 runs 8-bit Fast PWM at 62.5 kHz with no prescaler.
 *
 * Use:
 *   PB2 -> RC low-pass -> coupling capacitor -> amp input -> speaker
 */
void pwm_audio_init(void)
{
    DDRB  |= (1 << DDB2);   /* PB2 / OC1B as output */
    PORTB &= ~(1 << PORTB2);
    OCR1B  = 128U;          /* midscale */

    /* Fast PWM 8-bit, timer running, OC1B initially disconnected. */
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS10);
}

void pwm_audio_enable(void)
{
    /* Non-inverting PWM on OC1B */
    TCCR1A = (1 << WGM10) | (1 << COM1B1);
}

void pwm_audio_disable(void)
{
    /* Disconnect OC1B and force PB2 low */
    TCCR1A = (1 << WGM10);
    PORTB &= ~(1 << PORTB2);
}

void pwm_audio_write(uint16_t sample)
{
    OCR1B = (uint8_t)(sample >> 8);
}
