#include <stdint.h>
#include "envelope.h"

#define TICKS_PER_MS    31U
#define ATTACK_MS       10U
#define DECAY_MS        50U
#define SUSTAIN_LEVEL   40000U
#define RELEASE_MS      200U

typedef enum {
    ENV_IDLE = 0,
    ENV_ATTACK,
    ENV_DECAY,
    ENV_SUSTAIN,
    ENV_RELEASE
} env_state_t;

static volatile env_state_t env_state = ENV_IDLE;
static volatile uint16_t env_gain = 0U;

static uint16_t attack_step;
static uint16_t decay_step;
static uint16_t release_step;

void envelope_init(void)
{
    uint16_t v;

    v = (uint16_t)(65535U / ((uint32_t)ATTACK_MS * TICKS_PER_MS));
    attack_step = (v < 1U) ? 1U : v;

    v = (uint16_t)((65535U - SUSTAIN_LEVEL) /
                   ((uint32_t)DECAY_MS * TICKS_PER_MS));
    decay_step = (v < 1U) ? 1U : v;

    v = (uint16_t)(SUSTAIN_LEVEL /
                   ((uint32_t)RELEASE_MS * TICKS_PER_MS));
    release_step = (v < 1U) ? 1U : v;

    env_state = ENV_IDLE;
    env_gain  = 0U;
}

void envelope_trigger(void)
{
    env_state = ENV_ATTACK;
}

void envelope_release(void)
{
    if (env_state != ENV_IDLE) {
        env_state = ENV_RELEASE;
    }
}

void envelope_tick(void)
{
    switch (env_state) {
    case ENV_IDLE:
        break;

    case ENV_ATTACK:
        if (env_gain >= (uint16_t)(65535U - attack_step)) {
            env_gain = 65535U;
            env_state = ENV_DECAY;
        } else {
            env_gain += attack_step;
        }
        break;

    case ENV_DECAY:
        if (env_gain <= (uint16_t)(SUSTAIN_LEVEL + decay_step)) {
            env_gain = SUSTAIN_LEVEL;
            env_state = ENV_SUSTAIN;
        } else {
            env_gain -= decay_step;
        }
        break;

    case ENV_SUSTAIN:
        break;

    case ENV_RELEASE:
        if (env_gain <= release_step) {
            env_gain = 0U;
            env_state = ENV_IDLE;
        } else {
            env_gain -= release_step;
        }
        break;

    default:
        env_gain = 0U;
        env_state = ENV_IDLE;
        break;
    }
}

uint16_t envelope_get(void)
{
    return env_gain;
}
