#ifndef NOTES_H
#define NOTES_H

#include <stdint.h>
#include <avr/pgmspace.h>

#define NUM_GUITAR_NOTES  25U

#define GUITAR_NOTE_E2   0U
#define GUITAR_NOTE_F2   1U
#define GUITAR_NOTE_FS2  2U
#define GUITAR_NOTE_G2   3U
#define GUITAR_NOTE_GS2  4U
#define GUITAR_NOTE_A2   5U
#define GUITAR_NOTE_AS2  6U
#define GUITAR_NOTE_B2   7U
#define GUITAR_NOTE_C3   8U
#define GUITAR_NOTE_CS3  9U
#define GUITAR_NOTE_D3   10U
#define GUITAR_NOTE_DS3  11U
#define GUITAR_NOTE_E3   12U
#define GUITAR_NOTE_F3   13U
#define GUITAR_NOTE_FS3  14U
#define GUITAR_NOTE_G3   15U
#define GUITAR_NOTE_GS3  16U
#define GUITAR_NOTE_A3   17U
#define GUITAR_NOTE_AS3  18U
#define GUITAR_NOTE_B3   19U
#define GUITAR_NOTE_C4   20U
#define GUITAR_NOTE_CS4  21U
#define GUITAR_NOTE_D4   22U
#define GUITAR_NOTE_DS4  23U
#define GUITAR_NOTE_E4   24U

#define GH_NOTE_GREEN   GUITAR_NOTE_E4
#define GH_NOTE_RED     GUITAR_NOTE_B3
#define GH_NOTE_YELLOW  GUITAR_NOTE_G3
#define GH_NOTE_BLUE    GUITAR_NOTE_D3
#define GH_NOTE_ORANGE  GUITAR_NOTE_A2

extern const uint32_t note_phase_inc[NUM_GUITAR_NOTES] PROGMEM;

#endif /* NOTES_H */
