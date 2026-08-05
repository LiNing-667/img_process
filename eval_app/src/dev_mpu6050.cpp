#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "dev_mpu6050.h"

#define MPU6050_ADDR 0x68
static int i2c_fd = -1;

int mpu6050_init(const char *i2c_dev) {
    i2c_fd = open(i2c_dev, O_RDWR);
    if (i2c_fd < 0) return -1;
    if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_ADDR) < 0) return -1;
    
    // 唤醒 MPU6050 (PWR_MGMT_1 写 0)
    unsigned char cmd[2] = {0x6B, 0x00};
    write(i2c_fd, cmd, 2);
    return 0;
}

int mpu6050_read(mpu6050_data_t *data) {
    if (i2c_fd < 0) return -1;
    unsigned char reg = 0x3B;
    unsigned char buf[14];
    write(i2c_fd, &reg, 1);
    if (read(i2c_fd, buf, 14) != 14) return -1;
    
    data->accel_x = (buf[0] << 8) | buf[1];
    data->accel_y = (buf[2] << 8) | buf[3];
    data->accel_z = (buf[4] << 8) | buf[5];
    data->gyro_x  = (buf[8] << 8) | buf[9];
    data->gyro_y  = (buf[10] << 8) | buf[11];
    data->gyro_z  = (buf[12] << 8) | buf[13];
    return 0;
}