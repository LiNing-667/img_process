/**
 * @file serial_router.h
 * @brief 监控端指令的路由与分发
 */
#pragma once
#include <string>

class SerialRouter {
private:
    int fd_;
    std::string rx_buffer_;
    int initPort(const char *portname);
    void dispatchCommand(const std::string &cmd_str);

public:
    SerialRouter();
    bool start();
    void spinOnce();
};