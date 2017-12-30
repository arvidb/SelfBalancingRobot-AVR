#ifndef COMMON_H
#define COMMON_H

/*
    Pin Configuration
 
    PD0 - UART RX
    PD1 - UART TX
    PD2 - Encoder 1 Interrupt
    PD3 - Encoder 2 Interrupt
    PD4 - Motor1 B
    PD5 - PWM 1
    PD6 - PWM 2

    PB1 - Motor2 A
    PB2 - Motor2 B
    PB3 - Motor1 A
    PB5 - Led 1
    PB6 - Encoder 1 B
    PB7 - Encoder 2 B
 
    PC0 - Encoder 1 A
    PC1 - Encoder 1 B
    PC2 - Encoder 2 A
    PC3 - Encoder 2 B
 
    PC4 - I2C SCL
    PC5 - I2C SDA
 */

#define DATA_DELTA_TIME (0.01) // 10ms loop

#define RAD_TO_DEG(rad) (57.296f * rad)
#define DEG_TO_RAD(deg) (0.01745f * deg)

#define RET_OK      (0)
#define RET_ERROR   (-1)

typedef enum {
    CMD_SET_MOTOR_SPEED,
    CMD_SET_MOTOR1_GAIN,
    CMD_SET_MOTOR1_DEAD_ZONE,
    CMD_SET_MOTOR2_SPEED,
    CMD_SET_MOTOR2_GAIN,
    CMD_SET_MOTOR2_DEAD_ZONE,
    CMD_SET_PID_ANGLE_P,
    CMD_SET_PID_ANGLE_I,
    CMD_SET_PID_ANGLE_D,
    CMD_SET_PID_SPEED_P,
    CMD_SET_PID_SPEED_I,
    CMD_SET_PID_SPEED_D,
    CMD_SYNC = 0xFF,
} e_commands;

#define EEPROM_PID_P             0x00
#define EEPROM_PID_I             0x02
#define EEPROM_PID_D             0x04
#define EEPROM_MOTOR1_GAIN       0x06
#define EEPROM_MOTOR1_DEAD_ZONE  0x07
#define EEPROM_MOTOR2_GAIN       0x08
#define EEPROM_MOTOR2_DEAD_ZONE  0x09
#define EEPROM_PID2_P            0x0A
#define EEPROM_PID2_I            0x0C
#define EEPROM_PID2_D            0x0E

#endif /* COMMON_H */
