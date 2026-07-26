/**
 * @file pilot_global.h
 * @brief 跨模块需要的变量
 */
#pragma once
#include <string>

extern int g_monitor_fd;
void sendToMonitor(const std::string &msg);