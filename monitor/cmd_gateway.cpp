/**
 * @file cmd_gateway.cpp
 * @brief 指令网关与接收线程 (这是系统的心脏，负责接收终端输入、解析 Pilot 回传，并转化为 g_demo_task) (包含了写定的工作流)
 */
#include "cmd_gateway.h"
#include "global_state.h"
#include "monitor_log.h"
#include <iostream>
#include <unistd.h>
#include <thread>
#include <cstring>
#include <algorithm>

using namespace std;

// ==========================================================
// 视觉任务单槽提交协议 (解决 g_demo_task 写竞争)
// 所有来源都必须通过这两个函数写入任务，禁止再直接裸写 g_demo_task
// ==========================================================
bool task_try_submit(const DemoTask &task)
{
    std::lock_guard<std::mutex> lock(g_task_mtx);
    if (g_demo_task.pending)
    {
        monitor_log << "  ⚠ [任务网关] 视觉任务槽被占用，拒绝新任务: "
                    << task.raw_cmd << " (当前排队: " << g_demo_task.raw_cmd << ")" << std::endl;
        return false;
    }
    g_demo_task = task;
    g_demo_task.pending = true;
    return true;
}

void task_force_submit(const DemoTask &task)
{
    std::lock_guard<std::mutex> lock(g_task_mtx);
    if (g_demo_task.pending)
        monitor_log << "  ⚠ [任务网关] 强制覆盖排队中的任务: "
                    << g_demo_task.raw_cmd << " -> " << task.raw_cmd << std::endl;
    g_demo_task = task;
    g_demo_task.pending = true;
}

// ==========================================================
// 【新增】：stop 与 restart
// ==========================================================
static std::atomic<bool> g_macro_paused{false};
static std::mutex g_macro_pause_mtx;
static std::condition_variable g_macro_pause_cv;
static std::string g_macro_last_step = "初始未启动";
// 用于在动作链每个环节后检查是否需要挂起的护栏函数
void checkMacroPausePoint(const std::string &step_desc)
{
    g_macro_last_step = step_desc;
    if (g_macro_paused.load())
    {
        monitor_log << "\n>>> [宏动作链-挂起] 已安全停留在节点: [" << step_desc << "]，等待 RESTART 指令..." << std::endl;
        std::unique_lock<std::mutex> lock(g_macro_pause_mtx);
        g_macro_pause_cv.wait(lock, []()
                              { return !g_macro_paused.load(); });
        monitor_log << "\n>>> [宏动作链-唤醒] 接收到恢复信号，从 [" << step_desc << "] 节点继续执行！" << std::endl;
    }
}

void processTextCommand(const std::string &cmd_line)
{
    if (cmd_line.empty())
        return;
    std::string lower_cmd = cmd_line;
    for (auto &c : lower_cmd)
        c = tolower(c);

    if (lower_cmd == "stop")
    {
        g_macro_paused = true;
        monitor_log << "\n>>> [宏控制] 收到 STOP 指令！当前正在执行的底层动作完成后将安全挂起..." << std::endl;
        return;
    }
    if (lower_cmd == "restart")
    {
        {
            std::lock_guard<std::mutex> lock(g_macro_pause_mtx);
            g_macro_paused = false;
        }
        g_macro_pause_cv.notify_all();
        monitor_log << "\n>>> [宏控制] 收到 RESTART 指令！通知主调度线程恢复运行..." << std::endl;
        return;
    }
    if (lower_cmd == "fix")
    {
        g_trigger_aruco_fix = true;
        monitor_log << "\n>>> [静态标定] 已下发单次 ArUco 捕捉指令..." << std::endl;
        return;
    }
    if (lower_cmd == "nod")
    {
        g_auto_cam_running = true;
        g_cam_pan = 45.0f;
        g_cam_tilt = 50.0f;
        if (g_serial_fd >= 0)
        {
            char buf[64];
            sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
            write(g_serial_fd, buf, strlen(buf));
        }
        monitor_log << "\n>>> [自适应云台] 启动！开始提取车板特征曲线..." << std::endl;
        return;
    }

    // 拦截 findx (例如 find1, find2)
    if (lower_cmd.rfind("find", 0) == 0 && lower_cmd.length() == 5)
    {
        int target_id = lower_cmd[4] - '0';
        monitor_log << "\n>>> [状态机] 启动单帧寻物流 (锁定ID=" << target_id << ")..." << std::endl;

        // 开启独立线程，专门用来等云台到位，绝对不卡死主图像管线
        std::thread([target_id]()
                    {
            // 1. 获取 Nod 记忆角度 (如果没 Nod 过，给个默认兜底)
            if (g_calibrated_pan < 0 || g_calibrated_tilt < 0) {
                g_calibrated_pan = 45.0f; g_calibrated_tilt = 45.0f;
            }

            // 2. 摄像头转到特定姿态 (水平维持 nod，向下绝对角度 30 度)
            if (g_serial_fd >= 0) {
                char buf[64];
                sprintf(buf, "CAM %.1f 30.0\r\n", g_calibrated_pan);
                write(g_serial_fd, buf, strlen(buf));
            }

            // 3. 闭眼等待云台转动及画面完全稳定
            usleep(1500000); 

            // 4. 下发单帧视觉处理任务给 VisionEngine (任务槽忙则放弃本次)
            {
                DemoTask t;
                t.raw_cmd = "HSV_FIND_ONESHOT";
                t.class_id = target_id; // 传递 x 的 ID 给 PNP
                t.arm_id = 1;           // 强制在 ARM1 参考系下结算
                if (!task_try_submit(t))
                    monitor_log << "  ⚠ [find" << target_id << "] 视觉任务槽忙，本次寻物流已放弃" << std::endl;
            } })
            .detach();
        return;
    }

    if (lower_cmd == "start")
    {
        std::thread([]()
                    {
            monitor_log << "\n=============================================" << std::endl;
            monitor_log << ">>> 全自动装配宏动作链启动 (主调度流)" << std::endl;
            monitor_log << "=============================================\n" << std::endl;

            // 每次启动全新的 start 时，确保不处于暂停状态
            g_macro_paused = false;

            // ---- 具备实时挂起能力的事件驱动等待器 ----
            auto wait_demo = [](int timeout_ms = 350000, const std::string& step_name = "机械臂动作") {
                g_wf_demo_done = false;
                while (!g_wf_demo_done && timeout_ms > 0) { 
                    // 【核心修改】：在等待的每一帧都进行护栏探测，实现瞬间冻结
                    checkMacroPausePoint("等待 " + step_name + " 完成"); 
                    usleep(50000); 
                    timeout_ms -= 50; 
                }
                if (timeout_ms <= 0) monitor_log << "  ⚠ [超时] DEMO 未在时限内完成！" << std::endl;
            };

            auto wait_cmd = [](int timeout_ms = 150000, const std::string& step_name = "直通指令") {
                g_wf_cmd_done = false;
                while (!g_wf_cmd_done && timeout_ms > 0) { 
                    checkMacroPausePoint("等待 " + step_name + " 完成"); 
                    usleep(50000); 
                    timeout_ms -= 50; 
                }
                if (timeout_ms <= 0) monitor_log << "  ⚠ [超时] 直通指令未在时限内完成！" << std::endl;
            };

            auto wait_chassis = [](int timeout_ms = 200000, const std::string& step_name = "底盘移动") {
                g_wf_chassis_done = false;
                while (!g_wf_chassis_done && timeout_ms > 0) { 
                    checkMacroPausePoint("等待 " + step_name + " 到位"); 
                    usleep(50000); 
                    timeout_ms -= 50; 
                }
                if (timeout_ms <= 0) monitor_log << "  ⚠ [超时] 底盘未在时限内到达！" << std::endl;
            };

            // 【新增】：用于替代剧本中所有 raw usleep() 的可打断延时器
            auto macro_delay = [](int delay_us, const std::string& step_name = "流程间延时") {
                while (delay_us > 0) {
                    checkMacroPausePoint(step_name);
                    usleep(50000);
                    delay_us -= 50000;
                }
            };

            auto move_car = [&](float tx, float ty, float tyaw) {
                if (g_serial_fd >= 0) {
                    char buf[64];
                    sprintf(buf, "MOVE %.1f %.1f %.1f\r\n", tx, ty, tyaw);
                    write(g_serial_fd, buf, strlen(buf));
                    monitor_log << ">>> [动作链] 下发底盘寻路: X:" << tx << " Y:" << ty << " Yaw:" << tyaw << std::endl;
                }
                char desc[64];
                sprintf(desc, "底盘移动到 (%.1f, %.1f)", tx, ty);
                wait_chassis(200000, desc);
            };

            auto do_vision_demo = [&](int arm_id, int class_id, int action_id, const std::string& cmd) {
                monitor_log << "\n>>> [动作链] 派发视觉任务: " << cmd << " (ARM" << arm_id << " 锁定 ID=" << class_id << ")" << std::endl;
                {
                    DemoTask t;
                    t.arm_id = arm_id;
                    t.class_id = class_id;
                    t.action_id = action_id;
                    t.raw_cmd = cmd;
                    task_force_submit(t); // 主流水线：强制占用任务槽
                }
                wait_demo(350000, "视觉任务 " + cmd);
            };
            
            auto do_031_sequence = [&]() {
                monitor_log << "\n>>> [动作链] 下发 DO031 (换手)，并等待视觉闭环装配全流程完成..." << std::endl;
                if (g_serial_fd >= 0) {
                    std::string full_cmd = "DO031\r\n";
                    write(g_serial_fd, full_cmd.c_str(), full_cmd.length());
                }
                wait_demo(350000, "DO031 换手流程"); 
            };
            
            auto send_raw = [&](const std::string& cmd) {
                monitor_log << "\n>>> [动作链] 强插直通指令: " << cmd << std::endl;
                if (g_serial_fd >= 0) {
                    std::string full_cmd = cmd + "\r\n";
                    write(g_serial_fd, full_cmd.c_str(), full_cmd.length());
                }
                wait_cmd(150000, "直通指令 " + cmd);
            };

            auto auto_align_loop = [](const std::string& align_cmd) {
                monitor_log << "\n>>> [动作链] 开始全自动闭环对齐: " << align_cmd << std::endl;
                extern bool g_wf_align_done;
                extern bool g_wf_align_success;
                extern bool g_reset_align_memory;
                
                g_reset_align_memory = true;
                int max_retry = 7; 

                while(max_retry-- > 0) {
                    // 【核心修改】：在每次发起微调之前，允许瞬间挂起
                    checkMacroPausePoint("准备下发对齐循环: " + align_cmd);

                    g_wf_align_done = false;
                    g_wf_align_success = false;
                    
                    {
                        DemoTask t;
                        t.raw_cmd = align_cmd;
                        task_force_submit(t); // 主流水线：强制占用任务槽
                    }
                    
                    int timeout = 15000; 
                    while(!g_wf_align_done && !g_wf_align_success && timeout > 0) {
                        // 【核心修改】：在等待底盘调整的过程中，允许瞬间挂起
                        checkMacroPausePoint("等待对齐底层反馈: " + align_cmd);
                        usleep(50000);
                        timeout -= 50;
                    }
                    
                    if (g_wf_align_success) {
                        monitor_log << ">>> [动作链] 完美达标 (收到 B 信号)！" << align_cmd << " 调整结束。" << std::endl;
                        break; 
                    }
                    
                    if (g_wf_align_done) {
                        monitor_log << ">>> [动作链] 动作完成 (收到 A 信号)，等待 1.5 秒画面稳定后重试..." << std::endl;
                        
                        // 【核心修改】：把不可打断的 usleep(2500000) 换成支持暂停的轮询
                        int wait_stable = 2500000;
                        while(wait_stable > 0) {
                            checkMacroPausePoint("对齐后画面冷却缓冲: " + align_cmd);
                            usleep(50000);
                            wait_stable -= 50000;
                        }
                    } else if (timeout <= 0) {
                        monitor_log << ">>> [动作链] 严重警告: 等待底层动作超时，强行跳出对齐！" << std::endl;
                        break;
                    }
                }
            };

            //=======================================================
            // 动作链正式开始编排：像写剧本一样写在这里
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

            send_raw("MR");
            //抓连接件1
            move_car(-25, 45, -90);          
            auto_align_loop("align01");
            do_vision_demo(1, 3, 1, "DEMO131");
            //抓底座1
            move_car(-10, 90, -90);  
            auto_align_loop("align02");
            do_vision_demo(0, 0, 0, "DEMO000");
            //拼1
            move_car(25, 90, 90);
            auto_align_loop("align91");
            do_vision_demo(0, 9, 1, "DEMO091");
            do_031_sequence();
            //抓墙1
            move_car(0, 100, 0);
            auto_align_loop("align03");
            do_vision_demo(0, 2, 1, "DEMO021");
            //拼2
            move_car(25, 75, 90);
            auto_align_loop("align92");
            do_vision_demo(0, 0, 1, "DEMO001");
            //抓角柱1
            move_car(-25, 10, -90);
            auto_align_loop("align04");
            do_vision_demo(1, 1, 1, "DEMO111");
            //抓墙2
            move_car(-5, 100, 0);
            auto_align_loop("align03");
            do_vision_demo(0, 2, 1, "DEMO021");
            //拼3
            move_car(40, 30, 0);
            auto_align_loop("align93");
            do_vision_demo(0, 0, 2, "DEMO002");
            //抓连接件2
            move_car(-25, 55, -90);          
            auto_align_loop("align01");
            do_vision_demo(1, 3, 1, "DEMO131");
            //抓底座2
            move_car(-25, 90, -90);  
            auto_align_loop("align02");
            do_vision_demo(0, 0, 0, "DEMO000");
            usleep(1500000);
            //拼1
            move_car(50, 30, 0);
            auto_align_loop("align91");
            do_vision_demo(0, 9, 1, "DEMO091");
            //差不多了......

            monitor_log << ">>> 动作链结束！" << std::endl; })
            .detach();
        return;
    }

    if (lower_cmd.rfind("demo", 0) == 0 && lower_cmd.length() == 7)
    {
        int x = lower_cmd[4] - '0';
        int y = lower_cmd[5] - '0';
        int z = lower_cmd[6] - '0';
        if (x >= 0 && x <= 1 && y >= 0 && z >= 0)
        {
            std::string upper_cmd = lower_cmd;
            for (auto &c : upper_cmd)
                c = toupper(c);
            DemoTask t;
            t.arm_id = x;
            t.class_id = y;
            t.action_id = z;
            t.raw_cmd = upper_cmd;
            if (task_try_submit(t))
                monitor_log << "[Monitor] 已接收视觉任务 -> 目标臂: ARM" << x << " | 物体ID: " << y << std::endl;
            else
                monitor_log << "  ⚠ [Monitor] 视觉任务槽忙，已拒绝: " << upper_cmd << std::endl;
            return;
        }
    }
    if (lower_cmd.rfind("do", 0) == 0 && lower_cmd.length() == 5)
    {
        std::string upper_cmd = lower_cmd;
        for (auto &c : upper_cmd)
            c = toupper(c);
        if (g_serial_fd >= 0)
        {
            std::string send_str = upper_cmd + " 0 0 0 0 0 0 0 0 0 0\r\n";
            write(g_serial_fd, send_str.c_str(), send_str.length());
            monitor_log << "[Monitor] 下发盲操作 -> " << upper_cmd << std::endl;
        }
        return;
    }
    if (lower_cmd.rfind("align", 0) == 0)
    {
        monitor_log << "[Monitor] 拦截到 PC 终端对齐请求，已派发给视觉引擎: " << lower_cmd << std::endl;
        {
            DemoTask t;
            t.raw_cmd = lower_cmd; // 一定要传 lower_cmd，把 "align01" 完整传给视觉引擎
            task_try_submit(t);
        }
        return; // 必须 return，坚决不让它透传发给 Pilot！
    }

    // 未知指令透传
    if (g_serial_fd >= 0)
    {
        std::string send_str = cmd_line + "\r\n";
        write(g_serial_fd, send_str.c_str(), send_str.length());
        monitor_log << "[串口发往Pilot] -> " << cmd_line << std::endl;
    }
}

void terminalCommandThreadFunc()
{
    monitor_log << "========================================================" << std::endl;
    monitor_log << "[终端] 串口遥控模式就绪！输入 demoxyz 触发视觉检测与抓取！" << std::endl;
    monitor_log << "========================================================" << std::endl;

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
                if (line.empty())
                    continue;

                if (line.rfind("H", 0) == 0 && line.length() >= 3)
                {
                    monitor_log << "\n[Monitor 接收] 收到底层组装完成信号: " << line << std::endl;
                    DemoTask t;
                    t.raw_cmd = "CHECK_" + line;
                    task_try_submit(t);
                }
                else if (line.rfind("FIX_111", 0) == 0 || line.rfind("FIX_131", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到底层视觉对齐请求: " << line << std::endl;
                    DemoTask t;
                    t.raw_cmd = line;
                    task_try_submit(t);
                }
                else if (line.rfind("FIND_ACK_", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到云台就位确认: " << line << std::endl;
                    DemoTask t;
                    t.raw_cmd = line;
                    task_try_submit(t);
                }
                else if (line.rfind("CHECK_091", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到右臂悬空信号，开始视觉验核！" << std::endl;
                    DemoTask t;
                    t.raw_cmd = "CHECK_091";
                    task_try_submit(t);
                }
                else if (line.rfind("CHECK_001", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到 DEMO001 悬空信号，开始视觉验核！" << std::endl;
                    DemoTask t;
                    t.raw_cmd = "CHECK_001";
                    task_try_submit(t);
                }
                else if (line.rfind("CHECK_002", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到 DEMO002 悬空信号，开始视觉验核！" << std::endl;
                    DemoTask t;
                    t.raw_cmd = "CHECK_002";
                    task_try_submit(t);
                }
                else if (line.rfind("CHECK_003", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到 DEMO003 悬空信号，开始视觉验核！" << std::endl;
                    DemoTask t;
                    t.raw_cmd = "CHECK_003";
                    task_try_submit(t);
                }
                else if (line.rfind("CHASSIS_DONE", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到小车底盘就位确认: " << line << std::endl;
                    DemoTask t;
                    t.raw_cmd = "CHASSIS_DONE";
                    task_try_submit(t);
                }
                else if (line.rfind("JOINTS", 0) == 0) // 机械臂关节角上报 (Pilot → Monitor → 上位机)
                {
                    int arm_id = 0;
                    float a[6] = {0, 0, 0, 0, 0, 0};
                    if (sscanf(line.c_str(), "JOINTS %d %f %f %f %f %f %f",
                               &arm_id, &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]) >= 7)
                    {
                        monitor_log << "\n[Monitor 接收] 机械臂关节角上报 ARM" << arm_id << ": "
                                    << a[0] << " " << a[1] << " " << a[2] << " "
                                    << a[3] << " " << a[4] << " " << a[5] << std::endl;
                        std::vector<float> angles(a, a + 6);
                        pc_send_arm_joints((uint8_t)arm_id, angles); // 转发上位机更新 3D 预览
                    }
                }

                // 【已有的】拦截以 align 开头的视觉对齐指令
                else if (line.rfind("align", 0) == 0)
                {
                    monitor_log << "[Monitor] 拦截到对齐请求，已派发给视觉引擎: " << line << std::endl;
                    DemoTask t;
                    t.raw_cmd = line;
                    task_try_submit(t);
                }
                // ======================================================
                // 【新增】：拦截 Pilot 发回的 A 信号 (ALIGN_DONE)
                // ======================================================
                else if (line.rfind("ALIGN_DONE", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到小车对齐动作完成信号(A): " << line << std::endl;
                    extern bool g_wf_align_done;
                    g_wf_align_done = true; // 触发 A 信号
                }
                // ======================================================
                // 【新增】：DEMO_DONE — 机械臂动作完成
                // ======================================================
                else if (line.rfind("DEMO_DONE", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到机械臂动作完成信号: " << line << std::endl;
                    g_wf_demo_done = true;
                }
                // ======================================================
                // 【新增】：CMD_DONE — DOxxx / MR / MQ 等直通指令完成
                // ======================================================
                else if (line.rfind("CMD_DONE", 0) == 0)
                {
                    monitor_log << "\n[Monitor 接收] 收到直通指令完成信号: " << line << std::endl;
                    g_wf_cmd_done = true;
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