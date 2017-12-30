#include "motors.h"

#define MC1_PWM OCR0A
#define MC2_PWM OCR0B

// Sets the power and direction of a motor
void set_motor_direction(volatile uint8_t *port, uint8_t a1Mask, uint8_t a2Mask, e_motor_dir dir)
{
    // Set direction
    switch(dir)
    {
        case BREAK:
            // 1A1=1A2
            *port |= a1Mask;
            *port |= a2Mask;
            break;
        case FORWARD:
            // 1A1=1, 1A2=0
            *port |= a1Mask;
            *port &= ~a2Mask;
            break;
        case REVERSE:
            // 1A1=0, 1A2=1
            *port &= ~a1Mask;
            *port |= a2Mask;
            break;
    }
}

void update_motors(uint16_t pwr1, uint16_t pwr2, e_motor_dir dir1, e_motor_dir dir2) {

    if (pwr1 > 255) pwr1 = 255; // Cap max
    if (pwr2 > 255) pwr2 = 255; // Cap max
    
    MC1_PWM = 255-pwr1;
    MC2_PWM = 255-pwr2;
    
    set_motor_direction(&PORTD, _BV(PORTD3), _BV(PORTD4), dir1); // Motor 1
    set_motor_direction(&PORTB, _BV(PORTB1), _BV(PORTB2), dir2); // Motor 2
}

void init_motor_controllers()
{
    // Set DIR bits and enable PWM to output
    DDRD |= _BV(DDD5) | _BV(DDD6); // PWM
    DDRD |= _BV(DDD3) | _BV(DDD4); // DIR1
    DDRB |= _BV(DDB1) | _BV(DDB2); // DIR2
    
    TCCR0A = (1<<COM0A0) | (1<<COM0A1); // Set OC0A on compare match
    TCCR0A |= (1<<COM0B0) | (1<<COM0B1); // Set OC0B on compare match
    
    //TCCR0A |= (1<<WGM00) | (1<<WGM01); // Fast PWM
    TCCR0A |= (1<<WGM00); // Phase-correct PWM
    
    TCCR0B = (1<<CS00);// no prescale
    //TCCR0B = (1<<CS01);// 8 prescale
}
