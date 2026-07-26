/**
 * @file hw_i2c.cpp
 * @brief I2C 总线底层
 */
#include "hw_i2c.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

I2CDevice::I2CDevice(const std::string &name) : fd_(-1), dev_name_(name) {}

bool I2CDevice::init() {
    fd_ = open(dev_name_.c_str(), O_RDWR);
    return fd_ >= 0;
}

void I2CDevice::writeReg(int addr, uint8_t reg, uint8_t val) {
    if (fd_ < 0) return;
    std::lock_guard<std::mutex> lock(bus_mtx_);
    if (ioctl(fd_, I2C_SLAVE, addr) >= 0) {
        uint8_t buf[2] = {reg, val};
        write(fd_, buf, 2);
    }
}

// 实例化
#include "pilot_config.h"
I2CDevice g_i2c_arm(SystemConfig::I2C_DEV_ARM);