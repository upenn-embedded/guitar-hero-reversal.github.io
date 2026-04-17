#ifndef ENVELOPE_H
#define ENVELOPE_H

#include <stdint.h>

void envelope_init(void);
void envelope_trigger(void);
void envelope_release(void);
void envelope_tick(void);
uint16_t envelope_get(void);

#endif /* ENVELOPE_H */
