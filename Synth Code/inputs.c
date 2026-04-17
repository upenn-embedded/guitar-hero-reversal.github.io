#include <avr/io.h>
#include "inputs.h"

/* Active-high controller inputs: 0 V idle, 5 V pressed. */
#define GREEN_PIN        PD2
#define RED_PIN          PD3
#define YELLOW_PIN       PD4
#define BLUE_PIN         PD5
#define ORANGE_PIN       PD6
#define STRUM_PIN        PD7
#define DEBOUNCE_THRESH  3U

static uint8_t fret_cnt[5] = {0U, 0U, 0U, 0U, 0U};
static uint8_t fret_state   = 0U;
static uint8_t last_fret    = FRET_NONE;

static uint8_t strum_cnt   = 0U;
static uint8_t strum_state = 0U;

static uint8_t read_frets_raw(void)
{
    uint8_t raw = 0U;

    if (PIND & (1 << GREEN_PIN))  raw |= (1U << 0);
    if (PIND & (1 << RED_PIN))    raw |= (1U << 1);
    if (PIND & (1 << YELLOW_PIN)) raw |= (1U << 2);
    if (PIND & (1 << BLUE_PIN))   raw |= (1U << 3);
    if (PIND & (1 << ORANGE_PIN)) raw |= (1U << 4);

    return raw;
}

void inputs_init(void)
{
    DDRD  &= ~((1 << GREEN_PIN) | (1 << RED_PIN) | (1 << YELLOW_PIN) |
               (1 << BLUE_PIN)  | (1 << ORANGE_PIN) | (1 << STRUM_PIN));

    /* No internal pull-ups because the controller drives 0 V / 5 V actively. */
    PORTD &= ~((1 << GREEN_PIN) | (1 << RED_PIN) | (1 << YELLOW_PIN) |
               (1 << BLUE_PIN)  | (1 << ORANGE_PIN) | (1 << STRUM_PIN));
}

void inputs_tick(void)
{
    uint8_t raw_frets = read_frets_raw();

    for (uint8_t i = 0U; i < 5U; i++) {
        uint8_t pressed = (raw_frets >> i) & 0x01U;

        if (pressed) {
            if (fret_cnt[i] < DEBOUNCE_THRESH) {
                fret_cnt[i]++;
            }
        } else {
            if (fret_cnt[i] > 0U) {
                fret_cnt[i]--;
            }
        }

        if ((fret_cnt[i] == DEBOUNCE_THRESH) && !(fret_state & (1U << i))) {
            fret_state |= (1U << i);
            on_button_press(i);
        }

        if ((fret_cnt[i] == 0U) && (fret_state & (1U << i))) {
            fret_state &= ~(1U << i);
            on_button_release(i);
        }
    }

    /* Priority: green, then red, yellow, blue, orange if multiple are held. */
    {
        uint8_t cur_fret = FRET_NONE;
        for (uint8_t i = 0U; i < 5U; i++) {
            if (fret_state & (1U << i)) {
                cur_fret = i;
                break;
            }
        }

        if (cur_fret != last_fret) {
            last_fret = cur_fret;
            on_fret_change(cur_fret);
        }
    }

    {
        uint8_t strum_raw = (PIND & (1 << STRUM_PIN)) ? 1U : 0U;

        if (strum_raw) {
            if (strum_cnt < DEBOUNCE_THRESH) {
                strum_cnt++;
            }
        } else {
            if (strum_cnt > 0U) {
                strum_cnt--;
            }
        }

        if ((strum_cnt == DEBOUNCE_THRESH) && !strum_state) {
            strum_state = 1U;
            on_strum_press();
        }
        if ((strum_cnt == 0U) && strum_state) {
            strum_state = 0U;
            on_strum_release();
        }
    }
}
