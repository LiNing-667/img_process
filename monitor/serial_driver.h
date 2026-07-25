//这里负责接管系统信号和基础串口初始化

#pragma once

// 暴露给主程序的系统初始化接口
namespace SystemInit {
    void initAll();
}
// 信号处理也放在这里
void signalHandler(int signum);