/**
 * @file monitor_log.h
 * @brief 统一日志出口：信息同时输出到终端(stdout) 和 上位机(PC)
 *
 * 用法（与 std::cout 完全一致）：
 *   monitor_log        << "任务完成: " << value << std::endl;   // 终端 + 上位机(0x82)
 *   monitor_log_local  << "逐帧残差: " << err  << std::endl;    // 仅终端（高频逐帧日志，
 *                                                               //  避免刷爆上位机链路）
 */
#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// 向上位机(PC)发送一条下行文本消息 (CMD_DOWNLINK_MSG, 0x82)。
// 在 network_server.cpp 中实现；PC 未连接时内部自动忽略。
void pc_send_downlink(const std::string &text);

// 向上位机(PC)上报机械臂关节角 (CMD_ARM_JOINTS, 0x03: arm_id + 1B计数 + N×float)。
// 供 monitor 收到 Pilot 的 JOINTS 上报后转发，PC 据此更新 3D 预览。
void pc_send_arm_joints(uint8_t arm_id, const std::vector<float> &angles);

// 向上位机(PC)上报小车坐标 (CMD_VEHICLE_POS, 0x02: 4×float → x,y,z,yaw)。
// 供 monitor 收到 Pilot 的 POS 上报后转发，PC 据此更新平面坐标图。
void pc_send_vehicle_pos(float x, float y, float z, float yaw);

// 流式日志代理：以临时对象形式存在于整条语句内，语句结束时统一输出到终端并转发上位机。
class MonitorLog
{
public:
    template <typename T>
    MonitorLog &operator<<(const T &v)
    {
        oss_ << v;
        return *this;
    }

    // 重载流操纵符（如 std::endl）
    MonitorLog &operator<<(std::ostream &(*manip)(std::ostream &))
    {
        return *this;
    }

    ~MonitorLog()
    {
        std::string s = oss_.str();
        if (!s.empty())
        {
            std::cout << s << std::endl; // 终端
            pc_send_downlink(s);         // 上位机
        }
    }

private:
    std::ostringstream oss_;
};

// 双输出日志：终端 + 上位机
#define monitor_log MonitorLog()
// 仅终端日志：供高频逐帧调试信息使用
#define monitor_log_local std::cout
