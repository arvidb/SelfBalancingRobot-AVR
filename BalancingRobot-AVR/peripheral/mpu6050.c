#include "mpu6050.h"
#include "mpu6050_config.h"
#include <util/delay.h>

#include "i2c.h"

#define gyro_xsensitivity 66.5
#define gyro_ysensitivity 66.5
#define gyro_zsensitivity 65.5

static int32_t gyro_offset_x;
static int32_t gyro_offset_y;
static int32_t gyro_offset_z;

void mpu6050_init(void) {
    /*
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 0x01);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_CONFIG, 0x03);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 0b00001000);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 0b00001000);
    */
    //Sets sample rate to 8000/1+7 = 1000Hz
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 0x07);
    //i2c_write_byte(MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 0x03);
    //Disable FSync, 256Hz DLPF
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_CONFIG, 0x00);
    //Disable gyro self tests, scale of 500 degrees/s
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 0x8);
    //Disable accel self tests, scale of +-2g, no DHPF
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 0x00);
    
    
    //Freefall threshold of |0mg|
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_FF_THR, 0x00);
    //Freefall duration limit of 0
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_FF_DUR, 0x00);
    //Motion threshold of 0mg
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_MOT_THR, 0x00);
    //Motion duration of 0s
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_MOT_DUR, 0x00);
    //Zero motion threshold
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_ZRMOT_THR, 0x00);
    //Zero motion duration threshold
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_ZRMOT_DUR, 0x00);
    //Disable sensor output to FIFO buffer
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_FIFO_EN, 0x00);
    
    //AUX I2C setup
    //Sets AUX I2C to single master control, plus other config
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_MST_CTRL, 0x00);
    //Setup AUX I2C slaves
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV0_ADDR, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV0_REG, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV0_CTRL, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV1_ADDR, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV1_REG, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV1_CTRL, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV2_ADDR, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV2_REG, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV2_CTRL, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV3_ADDR, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV3_REG, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV3_CTRL, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV4_ADDR, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV4_REG, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV4_DO, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV4_CTRL, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV4_DI, 0x00);
    
    //Setup INT pin and AUX I2C pass through
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, 0x00);
    //Enable data ready interrupt
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_INT_ENABLE, 0x00);
    
    //Slave out, dont care
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV0_DO, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV1_DO, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV2_DO, 0x00);
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_SLV3_DO, 0x00);
    //More slave config
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_I2C_MST_DELAY_CTRL, 0x00);
    //Reset sensor signal paths
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_SIGNAL_PATH_RESET, 0x00);
    //Motion detection control
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_MOT_DETECT_CTRL, 0x00);
    //Disables FIFO, AUX I2C, FIFO and I2C reset bits to 0
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_USER_CTRL, 0x00);
    //Sets clock source to gyro reference w/ PLL
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 0x2);
    //Controls frequency of wakeups in accel low power mode plus the sensor standby modes
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_PWR_MGMT_2, 0x00);
    //Data transfer to and from the FIFO buffer
    i2c_write_byte(MPU6050_ADDR, MPU6050_RA_FIFO_R_W, 0x00);
}

void mpu6050_calibrate_gyro(void)
{
    const int nSamples = 100;
    
    int32_t xOffs = 0;
    int32_t yOffs = 0;
    int32_t zOffs = 0;
    
    gyro_offset_x = 0;
    gyro_offset_y = 0;
    gyro_offset_z = 0;
    
	int x = 0;
	for(x = 0; x < nSamples; x++)
	{
        // TODO: Read gyro only
        mpu6050_sensor_data_raw_t rawData;
        mpu6050_get_raw_sensor_data(&rawData);
        
		xOffs += rawData.gyroscope.x;
		yOffs += rawData.gyroscope.y;
		zOffs += rawData.gyroscope.z;

		_delay_ms(1);
	}
    
    xOffs /= nSamples;
    yOffs /= nSamples;
    zOffs /= nSamples;
    
    gyro_offset_x = xOffs;
    gyro_offset_y = yOffs;
    gyro_offset_z = zOffs;
}

void mpu6050_get_raw_sensor_data(mpu6050_sensor_data_raw_t* rawData) {

#define VAL16_BIG(buffer, idx) ((int16_t)(((int16_t)buffer[idx] << 8) | buffer[idx+1]))
    
    uint8_t buffer[14];
    
    i2c_read_bytes(MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buffer);
    
    // Get raw values
    rawData->accellerometer.x = VAL16_BIG(buffer, 0);
    rawData->accellerometer.y = VAL16_BIG(buffer, 2);
    rawData->accellerometer.z = VAL16_BIG(buffer, 4);
    
    int32_t tmp;
    tmp = VAL16_BIG(buffer, 8) - gyro_offset_x;
    rawData->gyroscope.x = (tmp > INT16_MIN && tmp < INT16_MAX) ? tmp : -1;
    tmp = VAL16_BIG(buffer, 10) - gyro_offset_y;
    rawData->gyroscope.y = (tmp > INT16_MIN && tmp < INT16_MAX) ? tmp : -1;
    tmp = VAL16_BIG(buffer, 12) - gyro_offset_z;
    rawData->gyroscope.z = (tmp > INT16_MIN && tmp < INT16_MAX) ? tmp : -1;
}

float gyro_ang_x = 0;
float gyro_ang_y = 0;
void mpu6050_get_sensor_data(mpu6050_sensor_data_t* data) {

    mpu6050_sensor_data_raw_t rawData;
    mpu6050_get_raw_sensor_data(&rawData);
    
    // Convert data to g and rad/sec
    data->accellerometer.x_g = (float)rawData.accellerometer.x;
    data->accellerometer.y_g = (float)rawData.accellerometer.y;
    data->accellerometer.z_g = (float)rawData.accellerometer.z;
    
    data->gyroscope.x_rad_sec = (float)rawData.gyroscope.x / gyro_xsensitivity;
    data->gyroscope.y_rad_sec = (float)rawData.gyroscope.y / gyro_ysensitivity;
    data->gyroscope.z_rad_sec = (float)rawData.gyroscope.z / gyro_zsensitivity;
    
    //gyro_ang_x += data->gyroscope.x_rad_sec * 0.01;
    //gyro_ang_y += data->gyroscope.y_rad_sec * 0.01;
}

void mpu6050_get_gyro_rates(mpu6050_sensor_data_raw_t* rawData, float *x_rate, float *y_rate) {
    
    *x_rate = (float)rawData->gyroscope.x / gyro_xsensitivity;
    *y_rate = (float)rawData->gyroscope.y / gyro_ysensitivity;
}

void mpu6050_get_euler_angles(mpu6050_sensor_data_t* data, float *roll, float *pitch) {

#define RAD_TO_DEG(rad) (57.296f * rad)
#define DEG_TO_RAD(deg) (0.01745f * deg)
    static float fXg = 0;
    static float fYg = 0;
    static float fZg = 0;
    
    fXg = data->accellerometer.x_g;
    fYg = data->accellerometer.y_g;
    fZg = data->accellerometer.z_g;
    
    *roll  = RAD_TO_DEG(atan2(-fYg, fZg));
    *pitch = RAD_TO_DEG(atan2(fXg, sqrt(fYg*fYg + fZg*fZg)));
    
    //*roll = RAD_TO_DEG(atan((float)fYg / sqrt(pow((float)fZg, 2) + pow((float)fXg, 2))));
    //*pitch = RAD_TO_DEG(atan((float)-fXg / sqrt(pow((float)fZg, 2) + pow((float)fYg, 2))));
}
