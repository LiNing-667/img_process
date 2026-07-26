/**
 * @file hw_i2c.h
 * @brief I2C 总线底层
 */
#pragma once
#include <string>
#include <mutex>
#include <stdint.h>

class I2CDevice {
private:
    int fd_;
    std::mutex bus_mtx_;
    std::string dev_name_;

public:
    I2CDevice(const std::string &name);
    bool init();
    void writeReg(int addr, uint8_t reg, uint8_t val);
};

// 全局暴露的 I2C 总线
extern I2CDevice g_i2c_arm;