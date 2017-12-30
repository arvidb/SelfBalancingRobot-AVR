#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <avr/io.h>

typedef enum {
    BREAK,
    FORWARD,
    REVERSE,
} e_motor_dir;

void init_motor_controllers(void);
void update_motors(uint16_t pwr1, uint16_t pwr2, e_motor_dir dir1, e_motor_dir dir2);

#endif /* MOTORS_H */
