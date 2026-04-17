#ifndef SYNTH_H
#define SYNTH_H

#include <stdint.h>

void synth_init(void);
void synth_set_note(uint8_t note_idx);
void synth_mute(void);

#endif /* SYNTH_H */
