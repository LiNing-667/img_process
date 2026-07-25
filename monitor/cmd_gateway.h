/**
 * @file cmd_gateway.h
 * @brief 指令网关与接收线程 (写定的工作流在.cpp)
 */
#pragma once
#include <string>

void processTextCommand(const std::string &cmd_line);

namespace CommunicationManager {
    void startThreads();
}