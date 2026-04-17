#ifndef SPI_DAC_H
#define SPI_DAC_H

#include <stdint.h>

void pwm_audio_init(void);
void pwm_audio_enable(void);
void pwm_audio_disable(void);
void pwm_audio_write(uint16_t sample);

#endif