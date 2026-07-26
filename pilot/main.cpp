/**
 * @file main.cpp
 * @brief 机械控制下位机 (Motor Node) 
 */
#include <iostream>
#include "pilot_config.h"    // 系统硬编码(夹爪舵机，PCA9685地址)
#include "arm_controller.h"  // 机械臂控制
#include "chassis_controller.h" // 麦轮底盘控制
#include "serial_router.h"   // 监控端指令的路由与分发

int main() {
    std::cout << "[Pilot] 启动运动控制核心下位机 " << std::endl;

    g_arm.init();

    if (!g_car.init()) {std::cerr << "[Pilot] 警告：无法连接 STM32 底盘串口，小车功能已禁用。" << std::endl;}
    
    SerialRouter router;
    if (!router.start()) {
        std::cerr << "[Pilot] 致命错误：串口打开失败: " << SystemConfig::SERIAL_PORT_MONITOR << std::endl;
        return -1;
    }
    
    std::cout << "[Pilot] 串口指令网关启动。\n[Pilot] 等待上位机 (Monitor) 接入指令...\n" << std::endl;

    while (true) {
        router.spinOnce();
    }
    return 0;
}