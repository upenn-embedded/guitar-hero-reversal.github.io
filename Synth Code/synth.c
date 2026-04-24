/*
 * synth.c ? Karplus-Strong plucked-string synthesizer
 * ESE3500 Final Project ? Guitar Synthesizer Controller
 * Team 3: Synth Specialist (Guitar Hero Edition)
 * University of Pennsylvania ? Spring 2026
 *
 * Authors: Adam Shalabi, Brandon Parkansky, Panos Dimtsoudis
 *
 * ?? How Karplus-Strong works ?????????????????????????????????????
 *
 *   1. At note onset (pluck), fill a short circular delay buffer with
 *      white noise.  The buffer length N determines the fundamental
 *      frequency:  f0 ? Fs / N  (Fs = 15625 Hz here).
 *
 *   2. Each sample tick: output buf[ptr], then replace it with the
 *      average of buf[ptr] and buf[(ptr+1) % N], advance ptr.
 *
 *   The averaging is a one-pole low-pass filter.  High harmonics lose
 *   energy faster than low ones, so the tone starts bright (all
 *   harmonics from the noise burst) and evolves toward a pure sine at
 *   the fundamental ? exactly the acoustic behaviour of a plucked
 *   string.  No wavetable, no static waveform.
 *
 * ?? Three-voice major triad chord ????????????????????????????????
 *
 *   Three independent delay lines run simultaneously:
 *     Voice 0 = root note
 *     Voice 1 = root + 4 semitones (major third)
 *     Voice 2 = root + 7 semitones (perfect fifth)
 *
 *   Their outputs are summed and written to OCR1B (8-bit Fast PWM).
 *
 * ?? Buffer sizing ?????????????????????????????????????????????????
 *
 *   At Fs = 15625 Hz, the range C3?G5 (the root + chord range)
 *   requires delay lines of 20?119 samples.
 *   3 voices ? 120 bytes = 360 bytes SRAM.
 *
 * ?? Decay tuning ??????????????????????????????????????????????????
 *
 *   The basic average  (cur + nxt) >> 1  gives ~150?500 ms decay.
 *   The weighted blend  (3*cur + nxt) >> 2  extends to ~1?2 s.
 *   KS_USE_SLOW_DECAY below switches between them.
 *
 * ?? Timer assignments ?????????????????????????????????????????????
 *
 *   Timer1 ? 8-bit Fast PWM, no prescaler, OC1B ? audio output pin PB2
 *   Timer2 ? CTC, prescaler 8, OCR2A=127 ? sample ISR at 15625 Hz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#include "synth.h"
#include "spi_dac.h"
#include "notes.h"

/* ??? Configuration ???????????????????????????????????????????????? */

#define CHORD_TONES       3U
#define KS_MAX_LEN      190U   /* max buffer covers E2 = 190 samples */
#define KS_GATE_MS     2200U   /* total gate: 800 hold + 1400 fade */
#define KS_HOLD_THRESH 1400U   /* gate value at which fade begins */
/*
 * Fade math: after the hold phase, gate counts from KS_HOLD_THRESH down to 0. gain = gate * 255 / KS_HOLD_THRESH.
 */
#define VIBRATO_MAX_STEP   8   /* kept for API compatibility, not used   */
#define KS_FILTER_PERIOD   4U   /* smaller = smoother, less buzzy attack */
#define DRIVE_KNEE        58    /* soft-clip knee: lower = more crunch */
#define DRIVE_LIMIT       86    /* absolute output limit after soft clip */

/*
 * KS_USE_SLOW_DECAY:
 *   0 = basic average  (cur + nxt) >> 1     ? bright, short (~200?500 ms)
 *   1 = weighted blend (3*cur + nxt) >> 2   ? warmer, longer (~1?2 s)
 *
 * Set to 1 for more guitar-like sustain.
 */
#define KS_USE_SLOW_DECAY  1

/* ??? Delay-line length table ?????????????????????????????????????? */
/*
 * One entry per semitone from E2 through C7. len = round(Fs / freq) where Fs = 15625 Hz. The extra C7 entry lets a C6 root use its octave voice.
 */
static const uint8_t ks_len_table[57] PROGMEM = {
    /* E2   F2   F#2  G2   G#2  A2   A#2  B2  */
      190, 179, 169, 159, 150, 142, 134, 127,
    /* C3   C#3  D3   D#3  E3   F3   F#3  G3  */
      119, 113, 106, 100,  95,  89,  84,  80,
    /* G#3  A3   A#3  B3   C4   C#4  D4   D#4 */
       75,  71,  67,  63,  60,  56,  53,  50,
    /* E4   F4   F#4  G4   G#4  A4   A#4  B4  */
       47,  45,  42,  40,  38,  36,  34,  32,
    /* C5   C#5  D5   D#5  E5   F5   F#5  G5  */
       30,  28,  27,  25,  24,  22,  21,  20,
    /* G#5  A5   A#5  B5   C6   C#6  D6   D#6 */
       19,  18,  17,  16,  15,  14,  13,  13,
    /* E6   F6   F#6  G6   G#6  A6   A#6  B6   C7 */
       12,  11,  11,  10,   9,   9,   8,   8,   7,
};
/* Index offset: subtract GUITAR_NOTE_E2 (0) before indexing this table. */

/* ??? Voice structure ?????????????????????????????????????????????? */
typedef struct {
    uint8_t buf[KS_MAX_LEN];   /* circular delay line (uint8_t: 0?255)  */
    uint8_t ptr;               /* current read/write head               */
    uint8_t len;               /* active delay length ? vibrato modifies this */
    uint8_t base_len;          /* delay length at pluck time ? vibrato reference */
} ks_voice_t;

/* ??? Module state ????????????????????????????????????????????????? */
static ks_voice_t        g_voice[CHORD_TONES]; /* 3 x 190 = 570 bytes */
static volatile uint8_t  g_muted      = 1U;
static volatile uint16_t g_gate_ms    = 0U;   /* safety timeout              */
static volatile uint8_t  g_out_gain    = 255U; /* 255=full, 0=silent (fade)  */
static volatile uint8_t  g_attack_gain = 255U; /* 0->255 ramp removes clicks */
static volatile uint8_t  g_ks_skip    = 0U;   /* K-S filter skip counter ? reset on each pluck */

/* Vibrato LFO state */
static volatile uint8_t  g_vibrato_depth = 0U;  /* 0?4, set by whammy ADC  */
static          int8_t   g_vibrato_pos   = 0;   /* current LFO position    */
static          int8_t   g_vibrato_dir   = 1;   /* LFO direction (+1/-1)   */

/* PRNG for pluck-noise initialisation (XOR-shift, no SRAM table).
 * Seed is varied per pluck to avoid identical tones on repeated notes. */
static uint8_t g_prng = 0xA5U;

static uint8_t prng_next(void)
{
    uint8_t x = g_prng;
    x ^= (uint8_t)(x << 3U);
    x ^= (uint8_t)(x >> 5U);
    x ^= (uint8_t)(x << 4U);
    g_prng = x;
    return x;
}

static int16_t soft_clip(int16_t x)
{
    if (x > DRIVE_KNEE) {
        x = (int16_t)(DRIVE_KNEE + ((x - DRIVE_KNEE) >> 2));
        if (x > DRIVE_LIMIT) { x = DRIVE_LIMIT; }
    } else if (x < -DRIVE_KNEE) {
        x = (int16_t)(-DRIVE_KNEE + ((x + DRIVE_KNEE) >> 2));
        if (x < -DRIVE_LIMIT) { x = -DRIVE_LIMIT; }
    }

    return x;
}

static uint8_t clamp_u8(int16_t x)
{
    if (x < 0)   { return 0U; }
    if (x > 255) { return 255U; }
    return (uint8_t)x;
}

/* ??? ISR ?????????????????????????????????????????????????????????? */
/*
 * TIMER2_COMPA_vect ? fires at 15625 Hz (Fs).
 *
 * For each of the CHORD_TONES voices:
 *   1. Read current sample (curr) from the delay line.
 *   2. Average it with its neighbour (Karplus-Strong filter).
 *   3. Write the filtered value back and advance the pointer.
 *   4. Accumulate curr into the output mix.
 *
 * The three mixed samples are summed, scaled, and written to OCR1B
 * (the 8-bit Fast PWM compare register that drives the audio pin).
 *
 * Cycle budget: Fs=15625 Hz, Fcpu=16 MHz ? 1024 cycles/sample.
 * Measured ISR cost: ~130 cycles (?13% of budget).
 */
ISR(TIMER2_COMPA_vect)
{
    if (g_muted) {
        return;
    }

    int16_t mixed = 0;

    /*
     * Voice weights for a power chord:
     *   root   = loudest
     *   fifth  = almost as loud
     *   octave = quieter, just adds bite/definition
     *
     * This sounds less "organ-like" than three equal-volume strings.
     */
    static const uint8_t voice_gain[CHORD_TONES] = {7U, 6U, 3U};

    g_ks_skip++;

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        ks_voice_t *v = &g_voice[i];

        uint8_t curr    = v->buf[v->ptr];
        uint8_t nxt_ptr = (uint8_t)(v->ptr + 1U);
        if (nxt_ptr >= v->len) { nxt_ptr = 0U; }
        uint8_t nxt     = v->buf[nxt_ptr];

        uint8_t filt = curr;

        /*
         * The old version filtered only every 16th sample.  That gave
         * long sustain, but it also left too much raw noise in the chord.
         * Filtering every 4th sample gives a quicker picked-string settle,
         * which is much better for Smoke-on-the-Water style riffs.
         */
        if (g_ks_skip >= KS_FILTER_PERIOD) {
            int16_t avg = (int16_t)(((uint16_t)curr + (uint16_t)nxt + 1U) >> 1U);

            /*
             * Tiny damping toward 128.  This keeps the power chord from
             * ringing forever without needing a harsh gate.
             */
            int16_t centered = (int16_t)(avg - 128);
            centered = (int16_t)((centered * 253) >> 8);
            filt = clamp_u8((int16_t)(128 + centered));
        }

        v->buf[v->ptr] = filt;
        v->ptr         = nxt_ptr;

        mixed += (int16_t)(((int16_t)curr - 128) * (int16_t)voice_gain[i]);
    }

    if (g_ks_skip >= KS_FILTER_PERIOD) { g_ks_skip = 0U; }

    /* Sum of gains = 16, so this returns the mix to roughly +/-127. */
    mixed >>= 4;

    /*
     * Softer amp-style clipping.  The old hard clip at +/-38 turned the
     * chord into a square-ish fuzz.  This keeps the crunch, but preserves
     * more note identity so the fifth/octave sound clearer.
     */
    mixed = soft_clip(mixed);

    if (g_attack_gain < 255U) {
        uint16_t next_attack = (uint16_t)g_attack_gain + 6U;
        g_attack_gain = (next_attack > 255U) ? 255U : (uint8_t)next_attack;
    }

    {
        uint16_t total_gain = ((uint16_t)g_out_gain * (uint16_t)g_attack_gain) >> 8;
        mixed = (int16_t)(((int16_t)mixed * (int16_t)total_gain) >> 8);
    }

    int16_t out = (int16_t)128 + mixed;
    if (out < 0)   { out = 0; }
    if (out > 255) { out = 255; }

    OCR1B = (uint8_t)out;
}

/* ??? Internal helpers ????????????????????????????????????????????? */

/*
 * ks_pluck() ? initialises one delay line for the given note index.
 *
 * Fills the buffer with white noise (from the XOR-shift PRNG).
 * Noise gives the broadband spectral content that the K-S filter then
 * sculpts into a pitched tone over the first few milliseconds.
 *
 * The buffer length is looked up from ks_len_table.  Valid range:
 * GUITAR_NOTE_C3 (8) through G5 (39).
 */
/*
 * ks_pluck() ? initialises one delay line for the given note index.
 *
 * Electric guitar pick excitation:
 *   A real pick strikes the string at roughly 1/8 of the string length
 *   from the bridge.  This concentrates energy near the pluck point and
 *   creates the characteristic bright, cutting attack.
 *
 *   The first 1/4 of the buffer is filled with full-amplitude noise
 *   (the pick contact zone ? maximum displacement, maximum energy).
 *   The rest is filled with low-amplitude noise (the quiet zone of the
 *   string away from the pick).
 *
 *   This asymmetric energy distribution biases the K-S filter toward
 *   odd harmonics at onset, giving the "twang" of a picked electric
 *   string before it settles to a warm fundamental.
 */
static void ks_pluck(ks_voice_t *v, uint8_t note_idx, uint8_t voice_idx)
{
    uint8_t tidx = (uint8_t)(note_idx - GUITAR_NOTE_E2);
    v->len      = pgm_read_byte(&ks_len_table[tidx]);
    v->base_len = v->len;
    v->ptr      = 0U;

    /*
     * Picked-string excitation.
     *
     * The old code filled a large part of the delay line with full-scale
     * white noise.  That is technically valid Karplus-Strong, but for
     * power chords it sounds more like static than a guitar.
     *
     * This version starts with a triangular string displacement plus a
     * small amount of noise.  The result has a clearer pitch immediately,
     * while still keeping enough pick attack to sound guitar-like.
     */
    static const uint8_t voice_primes[3] = {17U, 97U, 193U};
    g_prng = (uint8_t)(0xA5U ^ (note_idx * voice_primes[voice_idx])
                               ^ (voice_idx * 73U));

    prng_next(); prng_next(); prng_next();

    uint8_t half = (uint8_t)(v->len >> 1U);
    if (half == 0U) { half = 1U; }

    for (uint8_t i = 0U; i < v->len; i++) {
        int16_t tri;

        if (i < half) {
            tri = (int16_t)(-88 + (((int16_t)176 * (int16_t)i) / (int16_t)half));
        } else {
            uint8_t denom = (uint8_t)(v->len - half);
            if (denom == 0U) { denom = 1U; }
            tri = (int16_t)(88 - (((int16_t)176 * (int16_t)(i - half)) / (int16_t)denom));
        }

        /* Small independent noise gives the pick edge without static. */
        int16_t noise = (int16_t)(((int16_t)prng_next() - 128) >> 4);

        /* Slightly reduce the octave voice so it does not dominate. */
        if (voice_idx == 2U) {
            tri = (int16_t)((tri * 3) >> 2);
        }

        v->buf[i] = clamp_u8((int16_t)(128 + tri + noise));
    }
}

/* ??? Public API ??????????????????????????????????????????????????? */

/*
 * synth_init() ? configures Timer2 (K-S ISR) and Timer1 (PWM output).
 * Call before sei() in main().
 */
void synth_init(void)
{
    uint8_t sreg = SREG;
    cli();

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        g_voice[i].ptr = 0U;
        g_voice[i].len = 1U;
        for (uint8_t j = 0U; j < KS_MAX_LEN; j++) {
            g_voice[i].buf[j] = 128U;   /* silence = midscale */
        }
    }

    g_muted          = 1U;
    g_gate_ms        = 0U;
    g_out_gain       = 255U;
    g_attack_gain    = 255U;
    g_ks_skip        = 0U;
    g_vibrato_depth  = 0U;
    g_prng           = 0xA5U;

    /* Timer2: CTC mode, prescaler 8 ? ISR at 15625 Hz
     *   OCR2A = 127 ? 16 000 000 / 8 / 128 = 15625 Hz            */
    TCCR2A = (1U << WGM21);      /* CTC mode                       */
    TCCR2B = 0x00U;
    OCR2A  = 127U;
    TCNT2  = 0U;
    TIFR2  = (1U << OCF2A);      /* clear pending flag             */
    TIMSK2 = (1U << OCIE2A);     /* enable compare-A interrupt     */
    TCCR2B = (1U << CS21);       /* prescaler = 8                  */

    pwm_audio_write(128U << 8U); /* hold output at midscale        */
    pwm_audio_disable();

    SREG = sreg;
}

/*
 * synth_set_note() ? plucks a power chord rooted at note_idx.
 *
 * Valid root range: GUITAR_NOTE_E2 (0) to GUITAR_NOTE_C6 (44).
 *
 * Voicing:
 *   Voice 0 = root
 *   Voice 1 = root + 7 st  (perfect fifth)
 *   Voice 2 = root + 12 st (octave)
 *
 * Each voice is seeded with an independent PRNG state so the three
 * strings sound decorrelated ? three separate plucks, not one louder one.
 */
void synth_set_note(uint8_t note_idx)
{
    if (note_idx > GUITAR_NOTE_C6) {
        return;
    }

    uint8_t notes[CHORD_TONES];
    notes[0] = note_idx;
    notes[1] = (uint8_t)(note_idx + 7U);    /* perfect fifth */
    notes[2] = (uint8_t)(note_idx + 12U);   /* octave        */

    uint8_t sreg = SREG;
    cli();

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        ks_pluck(&g_voice[i], notes[i], i);  /* pass voice index for independent seed */
    }

    /*
     * Reset LFO and filter state so every re-strum starts identically.
     *
     * Without this, g_vibrato_pos can be anywhere from -8 to +8 when
     * the note triggers, causing synth_vibrato_tick() to immediately
     * apply a pitch offset on the first tick.  Each re-strum then
     * captures a different LFO phase and sounds at a slightly different
     * pitch ? the "fluctuation" the user hears.
     *
     * g_ks_skip reset ensures the K-S averaging filter always fires at
     * the same point relative to the pluck onset, giving consistent
     * harmonic evolution and attack timbre on every strum.
     */
    g_vibrato_pos = 0;
    g_vibrato_dir = 1;
    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        g_voice[i].len = g_voice[i].base_len;   /* undo any lingering vibrato offset */
    }
    g_ks_skip  = 0U;

    g_gate_ms  = KS_GATE_MS;
    g_out_gain    = 255U;
    g_attack_gain = 0U;
    g_muted       = 0U;
    pwm_audio_enable();

    SREG = sreg;
}

/*
 * synth_set_chord() ? legacy function, maps a chord index (0?4) to a
 * root note and calls synth_set_note().  Kept for API compatibility.
 */
void synth_set_chord(uint8_t chord_idx)
{
    static const uint8_t roots[5] PROGMEM = {
        GUITAR_NOTE_D3,
        GUITAR_NOTE_F3,
        GUITAR_NOTE_G3,
        GUITAR_NOTE_A3,
        GUITAR_NOTE_D4,
    };

    if (chord_idx >= 5U) { return; }
    synth_set_note(pgm_read_byte(&roots[chord_idx]));
}

/*
 * synth_mute() ? immediately silences all output.
 * Fills delay lines with midscale (128) to avoid clicks on re-trigger.
 */
void synth_mute(void)
{
    uint8_t sreg = SREG;
    cli();

    g_gate_ms  = 0U;
    g_out_gain    = 255U;
    g_attack_gain = 255U;
    g_muted       = 1U;
    pwm_audio_disable();

    /* Reset buffers to silence so next pluck starts clean */
    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        for (uint8_t j = 0U; j < g_voice[i].len; j++) {
            g_voice[i].buf[j] = 128U;
        }
    }

    SREG = sreg;
}

/*
 * synth_decay_tick_1ms() ? called every 1 ms from Timer0 ISR.
 * Counts down the safety gate and forces silence when it expires.
 * The K-S algorithm itself handles the natural string decay; this is
 * just a backstop so the synth never hangs silently outputting noise.
 */
void synth_decay_tick_1ms(void)
{
    if (g_muted) { return; }

    if (g_gate_ms == 0U) {
        g_out_gain = 0U;
        g_muted    = 1U;
        pwm_audio_disable();
        return;
    }

    g_gate_ms--;

    /*
     * Hold phase: full volume for the first part of the note. Fade phase: scale linearly from 255 to 0.
     */
    if (g_gate_ms >= KS_HOLD_THRESH) {
        g_out_gain = 255U;
    } else {
        g_out_gain = (uint8_t)(((uint32_t)g_gate_ms * 255UL) / KS_HOLD_THRESH);
    }
}

uint8_t synth_is_active(void)
{
    return (uint8_t)(!g_muted);
}

void synth_set_vibrato_depth(uint8_t depth_percent)
{
    g_vibrato_depth = depth_percent;
}

void synth_reset_vibrato(void)
{
    uint8_t sreg = SREG;
    cli();

    g_vibrato_pos = 0;
    g_vibrato_dir = 1;

    /* Restore all voices to their original unmodulated lengths */
    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        if (g_voice[i].ptr >= g_voice[i].base_len) {
            g_voice[i].ptr = 0U;
        }
        g_voice[i].len = g_voice[i].base_len;
    }

    SREG = sreg;
}

/*
 * synth_vibrato_tick() ? called every 10 ms from main loop.
 *
 * Implements pitch vibrato by varying each voice's delay-line length
 * around its base_len.  A shorter delay = higher pitch; longer = lower.
 *
 * LFO: triangle wave, ?VIBRATO_MAX_STEP (8) steps.
 *   Period = 2 ? 16 steps ? 10 ms = 320 ms ? 3 Hz ? typical guitar vibrato.
 *
 * Sample offset from depth_percent (0?4, set by whammy ADC):
 *   depth 0    ? no vibrato
 *   depth 1?2  ? ?1 sample  (~25 cents at C4)
 *   depth 3?4  ? ?2 samples (~55 cents at C4)
 *
 * Pitch range per ?1 sample at common notes:
 *   C3  (len?119): ?1 sample = ?11 cents  (subtle)
 *   C4  (len?60):  ?1 sample = ?28 cents  (natural guitar vibrato)
 *   C5  (len?30):  ?1 sample = ?57 cents  (expressive)
 *
 * Safety: if the new length would put ptr out of bounds, ptr is reset
 * to 0 ? this causes a very brief click on extreme whammy pushes but
 * is inaudible at moderate depths.
 */
void synth_vibrato_tick(void)
{
    if (g_vibrato_depth == 0U || g_muted) { return; }

    /* Advance triangle LFO */
    g_vibrato_pos += g_vibrato_dir;
    if (g_vibrato_pos >= (int8_t)VIBRATO_MAX_STEP) {
        g_vibrato_pos = (int8_t)VIBRATO_MAX_STEP;
        g_vibrato_dir = -1;
    } else if (g_vibrato_pos <= -(int8_t)VIBRATO_MAX_STEP) {
        g_vibrato_pos = -(int8_t)VIBRATO_MAX_STEP;
        g_vibrato_dir = 1;
    }

    /* Map depth_percent (1?4) to a maximum sample offset (1 or 2) */
    uint8_t max_offset = (g_vibrato_depth >= 3U) ? 2U : 1U;

    /* Scale LFO position to sample offset:
     * delta = vibrato_pos * max_offset / VIBRATO_MAX_STEP */
    int8_t delta = (int8_t)(
        (int16_t)g_vibrato_pos * (int16_t)max_offset
        / (int16_t)VIBRATO_MAX_STEP
    );

    uint8_t sreg = SREG;
    cli();

    for (uint8_t i = 0U; i < CHORD_TONES; i++) {
        int16_t new_len = (int16_t)g_voice[i].base_len + (int16_t)delta;

        /* Clamp to safe buffer range */
        if (new_len < 15)                  { new_len = 15; }
        if (new_len > (int16_t)KS_MAX_LEN) { new_len = (int16_t)KS_MAX_LEN; }

        /* Reset ptr if it would be out of bounds after length change */
        if (g_voice[i].ptr >= (uint8_t)new_len) {
            g_voice[i].ptr = 0U;
        }
        g_voice[i].len = (uint8_t)new_len;
    }

    SREG = sreg;
}