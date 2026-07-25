/**
 * @file pilot_comm.cpp
 * @brief Pilot 机械控制板串口通信封装
 */
#include "pilot_comm.h"
#include "global_state.h"
#include <iostream>
#include <unistd.h>
#include <cstring>

void PilotCommunicator::sendDemoCommand(const std::string &demo_name, const Pose6D &pose)
{
    if (g_serial_fd < 0)
    {
        std::cout << "[警告] 串口未打开，无法下发指令！" << std::endl;
        return;
    }
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
             demo_name.c_str(), pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
    std::cout << "\n=============================================" << std::endl;
    std::cout << "[通信层] -> 发送宏指令至 Pilot: " << buffer;
    std::cout << "=============================================\n" << std::endl;
    write(g_serial_fd, buffer, strlen(buffer));
}