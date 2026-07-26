/**
 * @file pilot_config.h
 * @brief 系统硬编码(夹爪舵机，PCA9685地址)
 */
#pragma once
#include <termios.h>

namespace SystemConfig {
    const char SERIAL_PORT_MONITOR[] = "/dev/ttyS1"; 
    const char SERIAL_PORT_CHASSIS[] = "/dev/ttyS2"; 
    const int BAUD_RATE = B115200;
    
    const char I2C_DEV_ARM[] = "/dev/i2c-0"; 
    const int PCA_ADDR_ARM0 = 0x40;         
    const int PCA_ADDR_ARM1 = 0x41;         
}

const float Arm0_open = 60.0f;
const float Arm0_close = 150.0f;
const float Arm1_open = 140.0f;
const float Arm1_close = 30.0f;