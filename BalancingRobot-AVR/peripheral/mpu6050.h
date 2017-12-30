//
//  mpu6050.h
//  BalancingRobot-AVR
//
//  Created by Arvid Bjorkqvist on 2014-05-02.
//  Copyright (c) 2014 Arvid Bjorkqvist. All rights reserved.
//

#ifndef BalancingRobot_AVR_mpu6050_h
#define BalancingRobot_AVR_mpu6050_h

#include <stdint.h>

typedef struct {

    struct {
        int16_t x, y, z;
    } accellerometer;
    
    struct {
        int16_t x, y, z;
    } gyroscope;
    
    struct {
        float x, y;
    } filtered;
    /*
    struct {
        int16_t p, i, d;
    } pid;*/
} mpu6050_sensor_data_raw_t;

typedef struct {
    
    struct {
        float x_g, y_g, z_g;
    } accellerometer;
    
    struct {
        float x_rad_sec, y_rad_sec, z_rad_sec;
    } gyroscope;
    
    struct {
        float x, y;
    } filtered;

    struct {
        float roll, pitch;
    } org;
    
} mpu6050_sensor_data_t;

void mpu6050_init(void);
void mpu6050_calibrate_gyro(void);
void mpu6050_get_raw_sensor_data(mpu6050_sensor_data_raw_t* rawData);
void mpu6050_get_sensor_data(mpu6050_sensor_data_t* data);
void mpu6050_get_gyro_rates(mpu6050_sensor_data_raw_t* rawData, float *x_rate, float *y_rate);
void mpu6050_get_euler_angles(mpu6050_sensor_data_t* rawData, float *roll, float *pitch);
#endif
