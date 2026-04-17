#ifndef INPUTS_H
#define INPUTS_H

#include <stdint.h>

#define FRET_NONE  0xFFU

void on_fret_change(uint8_t fret);
void on_button_press(uint8_t fret);
void on_button_release(uint8_t fret);
void on_strum_press(void);
void on_strum_release(void);

void inputs_init(void);
void inputs_tick(void);

#endif /* INPUTS_H */
