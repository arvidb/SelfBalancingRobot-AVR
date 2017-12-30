#include "encoders.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "peripheral/led.h"

#define PCINT_MASK  (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11))
#define LEFT_ENCODER_MASK  (_BV(PCINT8) | _BV(PCINT9))

volatile int32_t encoderTicks[2] = {0};
volatile uint8_t LastPCINTState = 0;


void encoders_init(void) {

    // PINT0 & PINT1 Input pins
    DDRC &= ~(_BV(DDC0) | _BV(DDC1) | _BV(DDC2) | _BV(DDC3));
    
    // Pullups PINT0 & PINT1
    PORTC |= (_BV(PORTC0) | _BV(PORTC1) | _BV(PORTC2) | _BV(PORTC3));
    
    PCICR |= _BV(PCIE1);    // set PCIE0 to enable PCMSK0 scan
    PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));  // set PCINT0 to trigger an interrupt on state change
    
    LastPCINTState = ( PINC & PCINT_MASK );
}

int32_t encoders_get(uint8_t encoderIdx) {

    return encoderTicks[encoderIdx];
}

void encoders_reset() {
    
    encoderTicks[ENCODER_LEFT] = 0;
    encoderTicks[ENCODER_RIGHT] = 0;
}

ISR (PCINT1_vect)
{
    //http://makeatronics.blogspot.se/2013/02/efficiently-reading-quadrature-with.html
    
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    
    static uint8_t enc_val1 = 0;
    static uint8_t enc_val2 = 0;
    
    uint8_t current = (PINC & PCINT_MASK);
    uint8_t change = current ^ LastPCINTState;
    
    LastPCINTState = current;
    
    if (change & LEFT_ENCODER_MASK) {
        // Encoder A
        
        enc_val1 = enc_val1 << 2; // Save last value
        enc_val1 = enc_val1 | (current & 0x3); // PC0 & PC1 (0b0011)
        
        encoderTicks[ENCODER_LEFT] += lookup_table[enc_val1 & 0xF]; // (0b1111)
    } else {
    
        // Encoder B
        
        enc_val2 = enc_val2 << 2; // Save last value
        enc_val2 = enc_val2 | ((current & 0xC) >> 2); // PC2 & PC3 (0b1100)
        
        encoderTicks[ENCODER_RIGHT] += lookup_table[enc_val2 & 0xF]; // (0b1111)
    }
}
