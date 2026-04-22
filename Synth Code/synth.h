#ifndef SYNTH_H
#define SYNTH_H

#include <stdint.h>

#define CHORD_TONES 3U
#define NUM_CHORDS  5U

#define CHORD_D4   0U
#define CHORD_F4   1U
#define CHORD_G4   2U
#define CHORD_AB4  3U
#define CHORD_D5   4U

void synth_init(void);
void synth_set_chord(uint8_t chord_idx);
void synth_mute(void);
void synth_set_vibrato_depth(uint8_t depth_percent);
void synth_reset_vibrato(void);
void synth_vibrato_tick(void);

#endif /* SYNTH_H */
