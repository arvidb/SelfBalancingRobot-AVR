#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdio.h>

#define ENCODER_LEFT 0
#define ENCODER_RIGHT 1

void encoders_init(void);
void encoders_reset(void);
int32_t encoders_get(uint8_t encoderIdx);
#endif /* ENCODERS_H */
