#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include "i2c.h"

#define F_SCL 100000L // SCL frequency
#define PRESCALER 1
#define TWBR_VALUE ((((F_CPU / F_SCL) / PRESCALER) - 16 ) / 2)

void i2c_init(void) {

    TWSR = 0;               /* Disable prescaler */
    TWBR = TWBR_VALUE;
}

uint8_t i2c_start(uint8_t address){
	
    // reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait for end of transmission
    loop_until_bit_is_set(TWCR, TWINT);
    
	// check if the start condition was successfully transmitted
    uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;
    
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
    loop_until_bit_is_set(TWCR, TWINT);
    
	// check if the device has acknowledged the READ / WRITE mode
    twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
    
	return 0;
}

uint8_t i2c_write(uint8_t data) {
    
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
    loop_until_bit_is_set(TWCR, TWINT);
    
    uint8_t twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;
    
	return 0;
}

uint8_t i2c_read_ack(void){
    
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	// wait for end of transmission
    loop_until_bit_is_set(TWCR, TWINT);
    
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_read_nack(void){
    
	// start receiving without acknowledging reception
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
    loop_until_bit_is_set(TWCR, TWINT);
    
	// return received data from TWDR
	return TWDR;
}

void i2c_stop(void){
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}


/*
 * Helper functions
 */

void i2c_write_bit(uint8_t deviceAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    i2c_read_byte(deviceAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    i2c_write_byte(deviceAddr, regAddr, b);
}

void i2c_write_bits(uint8_t deviceAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		uint8_t b = 0;
		if (i2c_read_byte(deviceAddr, regAddr, &b) != 0) { //get current data
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			i2c_write_byte(deviceAddr, regAddr, b);
		}
	}
}

void i2c_write_byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
    i2c_write_bytes(deviceAddr, regAddr, 1, &data);
}

void i2c_write_bytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
	if (length == 0) return;
    
    //write data
    i2c_start(deviceAddr | I2C_WRITE);
    i2c_write(regAddr); //reg
    for (uint8_t i = 0; i < length; i++) {
        i2c_write((uint8_t) data[i]);
    }
    i2c_stop();
}

int8_t i2c_read_byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data) {
    
    return i2c_read_bytes(deviceAddr, regAddr, 1, data);
}

int8_t i2c_read_bytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    
    int8_t count = 0;
    
    if (length == 0)
        return 0;
    
    i2c_start(deviceAddr | I2C_WRITE);
    i2c_write(regAddr);
    
    _delay_us(10);
    
    i2c_start(deviceAddr | I2C_READ);
    
    do {
        
        if(count==length-1)
            data[count] = i2c_read_nack();
        else
            data[count] = i2c_read_ack();
    } while (++count < length);
    
    i2c_stop();
    
    return count;
}
