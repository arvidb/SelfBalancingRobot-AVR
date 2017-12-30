#include "uart.h"
#include <avr/interrupt.h>
#include "common.h"

#define UART_BUFFER_SIZE 32 // Must be power of two!
#define UART_BUFFER_MASK ( UART_BUFFER_SIZE - 1)

static volatile uint8_t UART_rx_buffer[UART_BUFFER_SIZE];
static volatile uint8_t UART_rx_head;
static volatile uint8_t UART_rx_tail;
static volatile uint8_t UART_rx_buffer_size;

ISR(USART_RX_vect)
{
    uint16_t tmphead;
    uint8_t data;
    
    /* read UART status register and UART data register */
    data = UDR0;
    
    /* calculate buffer index */
    tmphead = ( UART_rx_head + 1) & UART_BUFFER_MASK;
    
    if ( tmphead == UART_rx_tail ) {
        // Overflow
    } else {
        /* store new index */
        UART_rx_head = tmphead;
        /* store received data in buffer */
        UART_rx_buffer[tmphead] = data;
        
        UART_rx_buffer_size++;
        if (UART_rx_buffer_size > UART_BUFFER_SIZE) {
            UART_rx_buffer_size = UART_BUFFER_SIZE;
        }
    }
}

void usart_init(void)
{
	/*Set baud rate */
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	
#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */

    UCSR0B |= _BV(RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
}

void usart_send_packet(uint8_t* payload, uint16_t length)
{
	uint8_t checksum = 0;
	
	// Send magic header
	usart_send_byte(0x7E);
    
	// Payload length
	usart_send_byte(length & 0xFF);
	usart_send_byte(length >> 8);
	
	// Payload
	while(length--)
	{
		checksum ^= *payload;
		usart_send_byte(*payload);
        payload++;
	}
	
    usart_send_byte(0xE7);
    
	// XOR checksum
	usart_send_byte(checksum);
}

int8_t usart_get_packet(command_packet_t *cmdPkg)
{
    // Check if we have received enough data (CMD + SYNC)
    if (UART_rx_buffer_size > sizeof(command_packet_t)) {
        
        uint8_t data;
        
        // Read first byte
        if (usart_get_char(&data) == RET_OK) {
            
            // Is it the sync byte?
            if (data == CMD_SYNC) {
                
                // Continue reading the command packet
                if (usart_get_char(&data) == RET_OK) {
                    cmdPkg->command = data;
                    
                    if (usart_get_char(&data) == RET_OK) {
                        
                        cmdPkg->value = data;
                        
                        if (usart_get_char(&data) == RET_OK) {
                            
                            cmdPkg->value |= (uint16_t)data << 8;
                            return RET_OK;
                        }
                        
                        return RET_OK;
                    }
                }
            }
        }
    }
    
    return RET_ERROR;
}

int8_t usart_get_char(uint8_t *data)
{
    uint16_t tmptail;
    
    if ( UART_rx_head == UART_rx_tail ) {
        /* no data available */
        return RET_ERROR;
    }
    
    /* calculate /store buffer index */
    tmptail = (UART_rx_tail + 1) & UART_BUFFER_MASK;
    UART_rx_tail = tmptail;
    
    /* get data from receive buffer */
    *data = UART_rx_buffer[tmptail];
    
    if (UART_rx_buffer_size > 0) {
        UART_rx_buffer_size--;
    }
    
    return RET_OK;
}
