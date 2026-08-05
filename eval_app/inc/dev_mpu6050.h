#ifndef __DEV_MPU6050_H__
#define __DEV_MPU6050_H__

typedef struct {
    short accel_x, accel_y, accel_z;
    short gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

int mpu6050_init(const char *i2c_dev);
int mpu6050_read(mpu6050_data_t *data);

#endif