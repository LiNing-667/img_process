/**
 * @file cmd_gateway.cpp
 * @brief 指令网关与接收线程 (这是系统的心脏，负责接收终端输入、解析 Pilot 回传，并转化为 g_demo_task) (包含了写定的工作流)
 */
#include "cmd_gateway.h"
#include "global_state.h"
#include <iostream>
#include <unistd.h>
#include <thread>
#include <cstring>
#include <algorithm>

using namespace std;

void processTextCommand(const std::string &cmd_line)
{
    if (cmd_line.empty()) return;
    std::string lower_cmd = cmd_line;
    for (auto &c : lower_cmd) c = tolower(c);

    if (lower_cmd == "fix")
    {
        g_trigger_aruco_fix = true;
        std::cout << "\n>>> [静态标定] 已下发单次 ArUco 捕捉指令..." << std::endl;
        return;
    }
    if (lower_cmd == "nod")
    {
        g_auto_cam_running = true;
        g_cam_pan = 113.0f;
        g_cam_tilt = 50.0f;
        if (g_serial_fd >= 0)
        {
            char buf[64];
            sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
            write(g_serial_fd, buf, strlen(buf));
        }
        std::cout << "\n>>> [自适应云台] 启动！开始提取车板特征曲线..." << std::endl;
        return;
    }
    if (lower_cmd == "find")
    {
        std::cout << "\n>>> [状态机] 启动巡航搜索，下发云台就位指令 DEMO220..." << std::endl;
        if (g_serial_fd >= 0)
        {
            std::string send_str = "DEMO220 0 0 0 0 0 0 0 0 0 0\r\n";
            write(g_serial_fd, send_str.c_str(), send_str.length());
        }
        return;
    }

    if (lower_cmd == "start")
    {
        std::thread([]() {
            std::cout << "\n=============================================" << std::endl;
            std::cout << ">>> 全自动装配宏动作链启动" << std::endl;
            std::cout << "=============================================\n" << std::endl;

            auto send_serial_cmd = [](const std::string& cmd, int wait_ms) {
                if (g_serial_fd >= 0) {
                    std::string full_cmd = cmd + "\r\n";
                    write(g_serial_fd, full_cmd.c_str(), full_cmd.length());
                    std::cout << ">>> [动作链] 下发指令: " << cmd << " (等待 " << wait_ms/1000.0 << " 秒)" << std::endl;
                }
                usleep(wait_ms * 1000);
            };

            auto do_nod = []() {
                std::cout << ">>> [动作链] 触发云台视觉调平 (Nod)..." << std::endl;
                g_auto_cam_running = true;
                g_cam_pan = 113.0f; g_cam_tilt = 50.0f;
                if (g_serial_fd >= 0) {
                    char buf[64]; sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
                    write(g_serial_fd, buf, strlen(buf));
                }
                usleep(1000000);  
                g_auto_cam_running = true;
                while (g_auto_cam_running) { usleep(100000); }
            };

            auto do_vision_demo = [](int arm_id, int class_id, int action_id, const std::string& cmd, int wait_ms) {
                std::cout << ">>> [动作链] 派发视觉任务: " << cmd << " (ARM" << arm_id << " 锁定 ID=" << class_id << ")" << std::endl;
                {
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.arm_id = arm_id;     
                    g_demo_task.class_id = class_id;   
                    g_demo_task.action_id = action_id;  
                    g_demo_task.raw_cmd = cmd;
                }
                usleep(wait_ms * 1000); 
            };

            send_serial_cmd("MS 28", 3000);  
            send_serial_cmd("ME 92.5", 3000);  
            send_serial_cmd("MW 15", 4000); 

            send_serial_cmd("MS 30", 3000);
            send_serial_cmd("ME 92.5", 3000); 

            send_serial_cmd("MW 45", 4000);

            send_serial_cmd("MS 40", 4000);
            send_serial_cmd("ME 92.5", 3000); 
            send_serial_cmd("MW 45", 4000);

            send_serial_cmd("ME 92.3", 3000); 
            send_serial_cmd("MW 38", 3000); 

            send_serial_cmd("MS 38", 4000);
            send_serial_cmd("ME 92.5", 3000); 
            send_serial_cmd("MW 35", 3000);

            send_serial_cmd("ME 92.5", 3000); 
            send_serial_cmd("MW 40", 4000);

            send_serial_cmd("MS 12", 3000);
            send_serial_cmd("MQ 91.5", 3000); 
            send_serial_cmd("MW 75", 3000); 

            std::cout << ">>> 动作链结束！" << std::endl; 
        }).detach();
        return;
    }

    if (lower_cmd.rfind("demo", 0) == 0 && lower_cmd.length() == 7)
    {
        int x = lower_cmd[4] - '0';
        int y = lower_cmd[5] - '0';
        int z = lower_cmd[6] - '0';
        if (x >= 0 && x <= 1 && y >= 0 && z >= 0)
        {
            std::lock_guard<std::mutex> lock(g_task_mtx);
            g_demo_task.pending = true;
            g_demo_task.arm_id = x;
            g_demo_task.class_id = y;
            g_demo_task.action_id = z;
            std::string upper_cmd = lower_cmd;
            for (auto &c : upper_cmd) c = toupper(c);
            g_demo_task.raw_cmd = upper_cmd;
            std::cout << "[Monitor] 已接收视觉任务 -> 目标臂: ARM" << x << " | 物体ID: " << y << std::endl;
            return;
        }
    }
    if (lower_cmd.rfind("do", 0) == 0 && lower_cmd.length() == 5)
    {
        std::string upper_cmd = lower_cmd;
        for (auto &c : upper_cmd) c = toupper(c);
        if (g_serial_fd >= 0)
        {
            std::string send_str = upper_cmd + " 0 0 0 0 0 0 0 0 0 0\r\n";
            write(g_serial_fd, send_str.c_str(), send_str.length());
            std::cout << "[Monitor] 下发盲操作 -> " << upper_cmd << std::endl;
        }
        return;
    }

    // 未知指令透传
    if (g_serial_fd >= 0)
    {
        std::string send_str = cmd_line + "\r\n";
        write(g_serial_fd, send_str.c_str(), send_str.length());
        std::cout << "[串口发往Pilot] -> " << cmd_line << std::endl;
    }
}

void terminalCommandThreadFunc()
{
    std::cout << "========================================================" << std::endl;
    std::cout << "[终端] 串口遥控模式就绪！输入 demoxyz 触发视觉检测与抓取！" << std::endl;
    std::cout << "========================================================" << std::endl;

    std::string cmd_line;
    while (std::getline(std::cin, cmd_line))
    {
        processTextCommand(cmd_line);
    }
}

void serialReadThreadFunc()
{
    char buffer[256];
    std::string rx_buffer;
    while (true)
    {
        if (g_serial_fd < 0)
        {
            usleep(100000);
            continue;
        }
        int n = read(g_serial_fd, buffer, sizeof(buffer) - 1);
        if (n > 0)
        {
            buffer[n] = '\0';
            rx_buffer += buffer;
            size_t pos;
            while ((pos = rx_buffer.find_first_of("\r\n")) != std::string::npos)
            {
                std::string line = rx_buffer.substr(0, pos);
                rx_buffer.erase(0, pos + 1);
                if (line.empty()) continue;

                if (line.rfind("H", 0) == 0 && line.length() >= 3)
                {
                    std::cout << "\n[Monitor 接收] 收到底层组装完成信号: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHECK_" + line;
                }
                else if (line.rfind("FIX_111", 0) == 0 || line.rfind("FIX_131", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到底层视觉对齐请求: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = line;
                }
                else if (line.rfind("FIND_ACK_", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到云台就位确认: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = line;
                }
                else if (line.rfind("CHASSIS_DONE", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到小车底盘就位确认: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHASSIS_DONE";
                }
            }
        }
        else
        {
            usleep(10000);
        }
    }
}

namespace CommunicationManager
{
    void startThreads()
    {
        thread cmd_thread(terminalCommandThreadFunc);
        cmd_thread.detach();
        thread rx_thread(serialReadThreadFunc);
        rx_thread.detach();
    }
}