/**
 * @file pilot_global.cpp
 * @brief 跨模块需要的变量
 */
#include "pilot_global.h"
#include <unistd.h>

int g_monitor_fd = -1;

void sendToMonitor(const std::string &msg) {
    if (g_monitor_fd >= 0) {
        write(g_monitor_fd, msg.c_str(), msg.length());
    }
}