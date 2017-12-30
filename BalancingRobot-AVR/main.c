#define ENABLE_MPU6050
#define DEBUG
#define USE_KALMAN
#define USE_ENCODERS

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "common.h"
#include "simpletask.h"

#include "peripheral/uart.h"
#include "peripheral/i2c.h"
#include "peripheral/mpu6050.h"
#include "peripheral/motors.h"
#include "peripheral/led.h"
#ifdef USE_ENCODERS
#include "peripheral/encoders.h"
#endif

#include "algorithms/pid.h"
#ifdef USE_KALMAN
#include "algorithms/kalman.h"
#endif

#ifdef USE_KALMAN
kalman_filter_t kalman;
#endif
pid_regulator_t PID_angle;
#ifdef USE_ENCODERS
pid_regulator_t PID_speed;
#endif

uint8_t motor_gain[2] = {0};
uint8_t dead_zone[2] = {0};

#ifdef DEBUG
int16_t motor_speed_override = -1;
#endif

void task_process_commands(void *arg);
void task_send_data(void *arg);
void task_update_motors(void *arg);
void task_update_sensors(void *arg);
void task_heartbeat(void *arg);

typedef struct {

    struct {
        float roll, pitch;
    } filtered;
    
    struct {
        float roll, pitch;
    } raw;
    
    struct {
        uint16_t p, i, d;
    } speed_PID;
    
    struct {
        uint16_t p, i, d;
    } angle_PID;
    
    float angle_PID_output;
    float speed_PID_output;
    uint8_t motor_gain[2], dead_zone[2];
    int32_t encoders[2];
} robot_data_packet_t;

#ifndef USE_KALMAN
struct complementary_t {

    float filter_xterm[3];
    float filter_yterm[3];
    
    float x_angle;
    float y_angle;
} complementary;

void second_order_complementary_filter(float acc_x, float acc_y, float gyro_x, float gyro_y)
{
#define timeConstant 0.1

    complementary.filter_xterm[0] = (acc_x - complementary.x_angle) * timeConstant * timeConstant;
    complementary.filter_yterm[0] = (acc_y - complementary.y_angle) * timeConstant * timeConstant;
    complementary.filter_xterm[2] = (dt * complementary.filter_xterm[0]) + complementary.filter_xterm[2];
    complementary.filter_yterm[2] = (dt * complementary.filter_yterm[0]) + complementary.filter_yterm[2];
    complementary.filter_xterm[1] = complementary.filter_xterm[2] + (acc_x - complementary.x_angle) * 2 * timeConstant + gyro_x;
    complementary.filter_yterm[1] = complementary.filter_yterm[2] + (acc_y - complementary.y_angle) * 2 * timeConstant + gyro_y;
    complementary.x_angle = (dt * complementary.filter_xterm[1]) + complementary.x_angle;
    complementary.y_angle = (dt * complementary.filter_yterm[1]) + complementary.y_angle;
}
#endif

void task_process_commands(void *arg) {
    (void)arg;
    
    command_packet_t cmd;
    if (usart_get_packet(&cmd) == RET_OK) {
    //uint8_t data;
    //if (usart_getchar(&data) == 0) {
     
        // Got data
        switch (cmd.command) {
#ifdef DEBUG
            case CMD_SET_MOTOR_SPEED:
                motor_speed_override = cmd.value;
                break;
#endif
            case CMD_SET_MOTOR1_GAIN:
                motor_gain[0] = cmd.value;
                eeprom_update_byte((uint8_t*)EEPROM_MOTOR1_GAIN, motor_gain[0]);
                break;
            case CMD_SET_MOTOR1_DEAD_ZONE:
                dead_zone[0] = cmd.value;
                eeprom_update_byte((uint8_t*)EEPROM_MOTOR1_DEAD_ZONE, dead_zone[0]);
                break;
            case CMD_SET_MOTOR2_GAIN:
                motor_gain[1] = cmd.value;
                eeprom_update_byte((uint8_t*)EEPROM_MOTOR2_GAIN, motor_gain[1]);
                break;
            case CMD_SET_MOTOR2_DEAD_ZONE:
                dead_zone[1] = cmd.value;
                eeprom_update_byte((uint8_t*)EEPROM_MOTOR2_DEAD_ZONE, dead_zone[1]);
                break;
                
            case CMD_SET_PID_ANGLE_P:
                pid_set_tunings(&PID_angle, cmd.value, PID_angle.Ki, PID_angle.Kd);
                eeprom_update_word((uint16_t*)EEPROM_PID_P, PID_angle.Kp);
                break;
            case CMD_SET_PID_ANGLE_I:
                pid_set_tunings(&PID_angle, PID_angle.Kp, cmd.value, PID_angle.Kd);
                eeprom_update_word((uint16_t*)EEPROM_PID_I, PID_angle.Ki);
                break;
            case CMD_SET_PID_ANGLE_D:
                pid_set_tunings(&PID_angle, PID_angle.Kp, PID_angle.Ki, cmd.value);
                eeprom_update_word((uint16_t*)EEPROM_PID_D, PID_angle.Kd);
                break;
                
            case CMD_SET_PID_SPEED_P:
                pid_set_tunings(&PID_speed, cmd.value, PID_speed.Ki, PID_speed.Kd);
                eeprom_update_word((uint16_t*)EEPROM_PID2_P, PID_speed.Kp);
                break;
            case CMD_SET_PID_SPEED_I:
                pid_set_tunings(&PID_speed, PID_speed.Kp, cmd.value, PID_speed.Kd);
                eeprom_update_word((uint16_t*)EEPROM_PID2_I, PID_speed.Ki);
                break;
            case CMD_SET_PID_SPEED_D:
                pid_set_tunings(&PID_speed, PID_speed.Kp, PID_speed.Ki, cmd.value);
                eeprom_update_word((uint16_t*)EEPROM_PID2_D, PID_speed.Kd);
                break;
            default:
                break;
        }
    }
}

void load_settings(void) {

    /* Read PID settings */
    {
        uint16_t tmp;
        uint16_t p = 0;
        uint16_t i = 0;
        uint16_t d = 0;
        tmp = eeprom_read_word((uint16_t*)EEPROM_PID_P);
        if (tmp != UINT16_MAX) p = tmp;
        
        tmp = eeprom_read_word((uint16_t*)EEPROM_PID_I);
        if (tmp != UINT16_MAX) i = tmp;
        
        tmp = eeprom_read_word((uint16_t*)EEPROM_PID_D);
        if (tmp != UINT16_MAX) d = tmp;
        
        pid_set_tunings(&PID_angle, p, i, d);
        
        p = i = d = 0;
        tmp = eeprom_read_word((uint16_t*)EEPROM_PID2_P);
        if (tmp != UINT16_MAX) p = tmp;
        
        tmp = eeprom_read_word((uint16_t*)EEPROM_PID2_I);
        if (tmp != UINT16_MAX) i = tmp;
        
        tmp = eeprom_read_word((uint16_t*)EEPROM_PID2_D);
        if (tmp != UINT16_MAX) d = tmp;
        
        pid_set_tunings(&PID_speed, p, i, d);
    }
    
    /* Read Motor settings */
    {
        uint8_t tmp;
        tmp = eeprom_read_byte((uint8_t*)EEPROM_MOTOR1_GAIN);
        if (tmp != UINT8_MAX) motor_gain[0] = tmp;
        
        tmp = eeprom_read_byte((uint8_t*)EEPROM_MOTOR1_DEAD_ZONE);
        if (tmp != UINT8_MAX) dead_zone[0] = tmp;
        
        tmp = eeprom_read_byte((uint8_t*)EEPROM_MOTOR2_GAIN);
        if (tmp != UINT8_MAX) motor_gain[1] = tmp;
        
        tmp = eeprom_read_byte((uint8_t*)EEPROM_MOTOR2_DEAD_ZONE);
        if (tmp != UINT8_MAX) dead_zone[1] = tmp;
    }
}

mpu6050_sensor_data_t sensor_data_previous;
void task_update_sensors(void *arg) {
    (void)arg;
    
#ifdef ENABLE_MPU6050
    mpu6050_sensor_data_t tmp;
    float roll = 0;
    float x_rad_sec = 0;
    for (int i=0; i < 5; i++) {
        mpu6050_get_sensor_data(&tmp);
        mpu6050_get_euler_angles(&tmp, &tmp.org.roll, &tmp.org.pitch);
        roll += tmp.org.roll;
        x_rad_sec += sensor_data_previous.gyroscope.x_rad_sec;
    }
    
    roll /= 5;
    x_rad_sec /= 5;
    
    sensor_data_previous = tmp;
    
    float angle_set = 0; // Wanted
    float angle_cur = 0; // Current
#ifdef USE_ENCODERS
    int32_t left = encoders_get(ENCODER_LEFT);
    int32_t right = encoders_get(ENCODER_RIGHT);
    
    float speedAvg = (left+right)/2.0f;
    if (speedAvg < 500) {
        pid_calculate(&PID_speed, 0, speedAvg/100.0, DATA_DELTA_TIME);
        angle_set = -PID_speed.output;
    } else {
    
        // Give up and update reference point
        encoders_reset();
    }
#endif
    
#ifdef USE_KALMAN
    kalman_calculate(&kalman, roll, -x_rad_sec, DATA_DELTA_TIME);
    angle_cur = kalman.angle;
#else
    second_order_complementary_filter(lastSensorData.org.roll,
                                      lastSensorData.org.pitch,
                                      lastSensorData.gyroscope.x_rad_sec,
                                      lastSensorData.gyroscope.y_rad_sec);
    angleCur = complementary.x_angle;
#endif
    
    pid_calculate(&PID_angle, angle_set, angle_cur, DATA_DELTA_TIME);
#endif
    
    task_update_motors(NULL);
}

void task_send_data(void *arg) {
    (void)arg;
    
    robot_data_packet_t pkg;
    
    pkg.angle_PID.p = PID_angle.Kp;
    pkg.angle_PID.i = PID_angle.Ki;
    pkg.angle_PID.d = PID_angle.Kd;
    
    pkg.speed_PID.p = PID_speed.Kp;
    pkg.speed_PID.i = PID_speed.Ki;
    pkg.speed_PID.d = PID_speed.Kd;
    
#ifdef USE_KALMAN
    pkg.filtered.roll = kalman.angle;
    pkg.filtered.pitch = kalman.bias;
#else
    pkg.filtered.roll = complementary.x_angle;
    pkg.filtered.pitch = complementary.y_angle;
#endif
    pkg.raw.roll = sensor_data_previous.org.roll;
    pkg.raw.pitch = sensor_data_previous.org.pitch;
    
    //pkg.anglePid = anglePID.output;
    pkg.motor_gain[0] = motor_gain[0];
    pkg.motor_gain[1] = motor_gain[1];
    pkg.dead_zone[0] = dead_zone[0];
    pkg.dead_zone[1] = dead_zone[1];
    
    pkg.encoders[0] = encoders_get(ENCODER_LEFT);
    pkg.encoders[1] = encoders_get(ENCODER_RIGHT);
    
    pkg.speed_PID_output = PID_speed.output;
    usart_send_packet((uint8_t *)&pkg, sizeof(robot_data_packet_t));
}

void task_heartbeat(void *arg) {
    (void)arg;
    
    led_toggle(LED1);
}

void task_update_motors(void *arg) {
    (void)arg;
    
    float throttle = fabsf(PID_angle.output);
    
    float motor1Pwr = (1.0 + ((float)motor_gain[0]/100.0)) * throttle + dead_zone[0];
    float motor2Pwr = (1.0 + ((float)motor_gain[1]/100.0)) * throttle + dead_zone[1];
    
    e_motor_dir dir = PID_angle.output > 0 ? REVERSE : FORWARD;
    
#ifdef DEBUG
    if (motor_speed_override >= 0) {
        motor1Pwr = motor_speed_override;
        motor2Pwr = motor_speed_override;
        dir = FORWARD;
    }
#endif
    
    update_motors(motor1Pwr, motor2Pwr, dir, dir);
}

void init(void) {
    
    // Disable interrupts
    cli();
    
    simpletask_init();
    
    led_init();

    usart_init();

#ifdef USE_ENCODERS
    encoders_init();
#endif
    
#ifdef ENABLE_MPU6050
    i2c_init();
    _delay_us(10);
    
    mpu6050_init();
    
    // Give it some time to initialize
    _delay_ms(100);
    
    mpu6050_calibrate_gyro(); // takes ~ 1 second
#endif
 
    init_motor_controllers();
    
    // Enable interrupts
    sei();
    
    pid_init(&PID_angle);
#ifdef USE_ENCODERS
    pid_init(&PID_speed);
#endif
#ifdef USE_KALMAN
    kalman_init(&kalman);
#endif
    
    load_settings();
}

int main(void) {
    
    init();
    
    simpletask_add(task_process_commands, 500);
    simpletask_add(task_update_sensors, 10);
    //simpletask_add(task_update_motors, 15);
    simpletask_add(task_send_data, 125);
    simpletask_add(task_heartbeat, 1000);

    while (1) {
        simpletask_run();
    }
    
	return 0; // never reached
}
