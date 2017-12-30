#ifndef LED_H
#define LED_H

#include <avr/io.h>
#include <stdlib.h>

#define LED1 (PORTB5)

inline void led_init(void) {
    
    // Set Led as output
    DDRB |= _BV(DDB5);
}

inline void led_toggle(uint8_t led) {

    PORTB ^= _BV(led);
}

inline void led_on(uint8_t led) {
    
    PORTB |= _BV(led);
}

inline void led_off(uint8_t led) {
    
    PORTB &= ~(_BV(led));
}

#endif /* LED_H */
