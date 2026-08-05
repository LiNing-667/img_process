/**
 * @file serial_driver.cpp
 * @brief 系统初始化与系统信号管理
 */
#include "serial_driver.h"
#include "global_state.h"
#include "config.h"
#include "monitor_log.h"
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <csignal>

using namespace std;

void signalHandler(int signum)
{
    // 信号处理上下文禁止加锁/网络发送，这里仅保留终端输出
    std::cout << "\n\n========================================================" << std::endl;
    std::cout << "[系统] 收到强制终止信号 (" << signum << ")，正在安全释放硬件资源..." << std::endl;
    if (g_cap_ptr && g_cap_ptr->isOpened())
    {
        g_cap_ptr->release();
        std::cout << "[系统] 摄像头节点已安全释放。" << std::endl;
    }
    if (g_serial_fd >= 0)
    {
        close(g_serial_fd);
        std::cout << "[系统] 串口已安全关闭。" << std::endl;
    }
    std::cout << "========================================================\n"
              << std::endl;
    exit(signum);
}

int initSerialPort(const char *portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
        return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
        return -1;
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        return -1;
    return fd;
}

namespace SystemInit
{
    void initAll()
    {
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        monitor_log << "[Monitor] 初始化 2K0300 系统..." << std::endl;
        g_serial_fd = initSerialPort(SystemConfig::SERIAL_PORT);
        if (g_serial_fd < 0)
        {
            cerr << "[警告] 串口打开失败，将无法向 Pilot 发送数据！" << endl;
        }
        else
        {
            monitor_log << "[Monitor] 串口通信就绪！" << std::endl;
        }
    }
}