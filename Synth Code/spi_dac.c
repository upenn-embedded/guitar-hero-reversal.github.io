#include <avr/io.h>
#include "spi_dac.h"

/*
 * PB2 buzzer driver using hardware toggle on OC1B.
 * Timer1 runs in CTC mode:
 *   - OCR1A sets the period (frequency)
 *   - OCR1B is placed at half-period so OC1B toggles once per timer cycle
 * Resulting output frequency:
 *   f_out = F_CPU / (2 * prescaler * (1 + OCR1A))
 */

void pwm_audio_init(void)
{
    DDRB  |= (1 << DDB2);     /* PB2 / OC1B as output */
    PORTB &= ~(1 << PORTB2);  /* idle low */

    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCNT1  = 0U;

    OCR1A  = 3033U;           /* default ~330 Hz at prescaler 8 */
    OCR1B  = OCR1A / 2U;      /* toggle in the middle of cycle */

    /* CTC mode selected, timer stopped until enabled. */
    TCCR1B = (1 << WGM12);
}

void pwm_audio_enable(void)
{
    TCNT1 = 0U;

    /* Toggle OC1B on compare match, CTC mode, prescaler = 8. */
    TCCR1A = (1 << COM1B0);
    TCCR1B = (1 << WGM12) | (1 << CS11);
}

void pwm_audio_disable(void)
{
    /* Disconnect OC1B and stop timer. */
    TCCR1A = 0x00;
    TCCR1B = (1 << WGM12);
    PORTB &= ~(1 << PORTB2);  /* force low while muted */
}

void pwm_audio_write(uint16_t sample)
{
    /* For PB2/OC1B in CTC mode, OCR1A sets frequency. */
    OCR1A = sample;
    OCR1B = sample / 2U;
}