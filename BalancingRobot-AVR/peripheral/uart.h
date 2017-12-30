#ifndef UART_H
#define UART_H

#define BAUD 9600

#include <stdio.h>
#include <util/setbaud.h>
#include <avr/io.h>
#include <stdlib.h>

typedef struct {
    uint8_t command;
    uint16_t value;
} command_packet_t;

void usart_init(void);
void usart_send_packet(uint8_t* payload, uint16_t length);
int8_t usart_get_packet(command_packet_t *cmdPkg);
int8_t usart_get_char(uint8_t *data);

inline void usart_send_byte(uint8_t data)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = data;
}

inline void usart_put_char(char c)
{
	if (c == '\n') usart_put_char('\r');
	
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
}

inline void usart_put_string(char *s){
	while(*s){
		usart_put_char(*s);
		s++;
	}
}

inline void usart_put_int(int i){
	char buf[10];
    ltoa(i, buf, 10);
    usart_put_string(buf);
}

#endif /* UART_H */
