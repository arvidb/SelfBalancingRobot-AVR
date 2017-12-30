#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_WRITE 0
#define I2C_READ 1

void i2c_init(void);
uint8_t i2c_start(uint8_t address);
uint8_t i2c_write(uint8_t data);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
void i2c_stop(void);

void i2c_write_bit(uint8_t deviceAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void i2c_write_bits(uint8_t deviceAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void i2c_write_byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t data);
void i2c_write_bytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t length, uint8_t* data);
int8_t i2c_read_byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t *data);
int8_t i2c_read_bytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

#endif /* I2C_H */
