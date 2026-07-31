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

    // 拦截 findx (例如 find1, find2)
    if (lower_cmd.rfind("find", 0) == 0 && lower_cmd.length() == 5)
    {
        int target_id = lower_cmd[4] - '0';
        std::cout << "\n>>> [状态机] 启动单帧寻物流 (锁定ID=" << target_id << ")..." << std::endl;

        // 开启独立线程，专门用来等云台到位，绝对不卡死主图像管线
        std::thread([target_id]() {
            // 1. 获取 Nod 记忆角度 (如果没 Nod 过，给个默认兜底)
            if (g_calibrated_pan < 0 || g_calibrated_tilt < 0) {
                g_calibrated_pan = 113.0f; g_calibrated_tilt = 45.0f;
            }

            // 2. 摄像头转到特定姿态 (水平维持 nod，向下绝对角度 30 度)
            if (g_serial_fd >= 0) {
                char buf[64];
                sprintf(buf, "CAM %.1f 30.0\r\n", g_calibrated_pan);
                write(g_serial_fd, buf, strlen(buf));
            }

            // 3. 闭眼等待云台转动及画面完全稳定
            usleep(1500000); 

            // 4. 下发单帧视觉处理任务给 VisionEngine
            {
                std::lock_guard<std::mutex> lock(g_task_mtx);
                g_demo_task.pending = true;
                g_demo_task.raw_cmd = "HSV_FIND_ONESHOT";
                g_demo_task.class_id = target_id; // 传递 x 的 ID 给 PNP
                g_demo_task.arm_id = 1;           // 强制在 ARM1 参考系下结算
            }
        }).detach();
        return;
    }

    if (lower_cmd == "start")
    {
        std::thread([]()
        {
            std::cout << "\n=============================================" << std::endl;
            std::cout << ">>> 全自动装配宏动作链启动 (主调度流)" << std::endl;
            std::cout << "=============================================\n" << std::endl;

            auto move_car = [](float tx, float ty, float tyaw, int wait_ms) {
                if (g_serial_fd >= 0) {
                    char buf[64];
                    sprintf(buf, "MOVE %.1f %.1f %.1f\r\n", tx, ty, tyaw);
                    write(g_serial_fd, buf, strlen(buf));
                    std::cout << ">>> [动作链] 下发底盘寻路: X:" << tx << " Y:" << ty << " Yaw:" << tyaw 
                              << " (预估等待 " << wait_ms/1000.0 << " 秒)" << std::endl;
                }
                // 直接靠 Monitor 端粗略预留等待时间，无需强制等待 Pilot 的回执锁死线程
                usleep(wait_ms * 1000);
            };

            auto do_vision_demo = [](int arm_id, int class_id, int action_id, const std::string& cmd, int wait_ms) {
                std::cout << "\n>>> [动作链] 派发视觉任务: " << cmd << " (ARM" << arm_id << " 锁定 ID=" << class_id << ")" << std::endl;
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

            // 直通下发任意单指令
            auto send_raw = [](const std::string& cmd, int wait_ms) {
                std::cout << "\n>>> [动作链] 强插直通指令: " << cmd << std::endl;
                if (g_serial_fd >= 0) {
                    std::string full_cmd = cmd + "\r\n";
                    write(g_serial_fd, full_cmd.c_str(), full_cmd.length());
                }
                usleep(wait_ms * 1000);
            };
            // ==========================================================
            // 【新增核心】：全自动闭环对齐 A/B 状态机循环引擎
            // ==========================================================
            // 【新增全局标志】：告诉 VisionEngine 这是一次全新的对齐，记录原点
            extern bool g_reset_align_memory;

            auto auto_align_loop = [](const std::string& align_cmd) {
                std::cout << "\n>>> [动作链] 开始全自动闭环对齐: " << align_cmd << std::endl;
                extern bool g_wf_align_done;
                extern bool g_wf_align_success;
                
                g_reset_align_memory = true; // 每次发起全新对齐前，重置记忆！
                
                int max_retry = 7; // 设置最大微调次数(7次)，防止因意外一直原地死循环

                while(max_retry-- > 0) {
                    g_wf_align_done = false;
                    g_wf_align_success = false;
                    
                    // 1. 发起一次 PnP 视觉解算与对齐
                    {
                        std::lock_guard<std::mutex> lock(g_task_mtx);
                        g_demo_task.pending = true;
                        g_demo_task.raw_cmd = align_cmd; 
                    }
                    
                    // 2. 轮询等待底盘的 A/B 反馈 (加了 15 秒强制超时保护)
                    int timeout = 15000; 
                    while(!g_wf_align_done && !g_wf_align_success && timeout > 0) {
                        usleep(50000); // 50ms 检查一次
                        timeout -= 50;
                    }
                    
                    // 3. 处理反馈
                    if (g_wf_align_success) {
                        std::cout << ">>> [动作链] 完美达标 (收到 B 信号)！" << align_cmd << " 调整结束。" << std::endl;
                        break; // 准了，跳出循环进入下一步抓取！
                    }
                    
                    if (g_wf_align_done) {
                        std::cout << ">>> [动作链] 动作完成 (收到 A 信号)，等待 1.5 秒画面稳定后重试..." << std::endl;
                        usleep(2500000); // 等待车身晃动结束、镜头对焦完毕
                    } else if (timeout <= 0) {
                        std::cout << ">>> [动作链] 严重警告: 等待底层动作超时，强行跳出对齐！" << std::endl;
                        break;
                    }
                }
            };

            //=======================================================
            // 动作链正式开始编排：你可以像写剧本一样写在这里
            //=======================================================
            
            // 示例剧本：
            
            // 1. 出发去 X=10cm, Y=20cm, 车头朝右转90度
            // path_plan(10, 20, 90); 
            // do_vision_demo(0, 0, 0, "DEMO000", 20000);
            
            // 2. 去右上方接应另一个物体，保持朝右
            // path_plan(30, 20, 90); 
            // do_vision_demo(1, 3, 1, "DEMO131", 15000);
            
            // 3. 倒车退回原点，车头依然朝向正前
            // path_plan(0, 0, 0);

            // ================== 请在下方自由编写 ==================

            send_raw("MR", 1000);

            move_car(-25, 30, -90, 12000);
            auto_align_loop("align01");
            do_vision_demo(1, 3, 1, "DEMO131", 22000);

            move_car(-20, 70, -90, 13000);
            //auto_align_loop("align02");
            do_vision_demo(0, 0, 0, "DEMO000", 20000);

            move_car(25, 70, 90, 15000);
            auto_align_loop("align91");
            //do_vision_demo(0, 9, 1, "DEMO091", 30000);

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
    if (lower_cmd.rfind("align", 0) == 0) 
    {
        std::cout << "[Monitor] 拦截到 PC 终端对齐请求，已派发给视觉引擎: " << lower_cmd << std::endl;
        std::lock_guard<std::mutex> lock(g_task_mtx);
        g_demo_task.pending = true;
        // 注意这里一定要传 lower_cmd，这样才能把 "align01" 完整传给视觉引擎
        g_demo_task.raw_cmd = lower_cmd; 
        return; // 必须 return，坚决不让它透传发给 Pilot！
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
                else if (line.rfind("CHECK_091", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到右臂悬空信号，开始视觉验核！" << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHECK_091";
                }
                else if (line.rfind("CHECK_001", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到 DEMO001 悬空信号，开始视觉验核！" << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHECK_001";
                }
                else if (line.rfind("CHECK_002", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到 DEMO002 悬空信号，开始视觉验核！" << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHECK_002";
                }
                else if (line.rfind("CHECK_003", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到 DEMO003 悬空信号，开始视觉验核！" << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHECK_003";
                }
                else if (line.rfind("CHASSIS_DONE", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到小车底盘就位确认: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHASSIS_DONE";
                }

                // 【已有的】拦截以 align 开头的视觉对齐指令
                else if (line.rfind("align", 0) == 0) 
                 {
                     std::cout << "[Monitor] 拦截到对齐请求，已派发给视觉引擎: " << line << std::endl;
                     std::lock_guard<std::mutex> lock(g_task_mtx);
                     g_demo_task.pending = true;
                     g_demo_task.raw_cmd = line; 
                 }
                // ======================================================
                // 【新增】：拦截 Pilot 发回的 A 信号 (ALIGN_DONE)
                // ======================================================
                else if (line.rfind("ALIGN_DONE", 0) == 0)
                {
                    std::cout << "\n[Monitor 接收] 收到小车对齐动作完成信号(A): " << line << std::endl;
                    extern bool g_wf_align_done;
                    g_wf_align_done = true; // 触发 A 信号
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