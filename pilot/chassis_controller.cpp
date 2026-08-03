/**
 * @file chassis_controller.cpp
 * @brief 麦轮底盘闭环控制
 */
#include "chassis_controller.h"
#include "pilot_config.h"
#include "pilot_global.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <algorithm>

// 初始化全局动力配置实例
ChassisDynamicsConfig g_dynamics;
ChassisController g_car;

ChassisController::ChassisController() : fd_(-1), running_(false) {}

int ChassisController::initPort(const char *portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
        return -1;
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, SystemConfig::BAUD_RATE);
    cfsetispeed(&tty, SystemConfig::BAUD_RATE);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

void ChassisController::sendCmd(const std::string &cmd)
{
    if (fd_ >= 0)
        write(fd_, cmd.c_str(), cmd.length());
}

void ChassisController::parseEncoder(const std::string &msg)
{
    size_t idx = msg.find("$MAll:");
    if (idx == std::string::npos)
        idx = msg.find("$MAII:");
    if (idx == std::string::npos)
        idx = msg.find("$MALL:");
    if (idx == std::string::npos)
        return;

    long m1, m2, m3, m4;
    std::string data = msg.substr(idx);
    if (sscanf(data.c_str() + 6, "%ld,%ld,%ld,%ld#", &m1, &m2, &m3, &m4) == 4)
    {
        if (first_encoder)
        {
            last_m1 = m1;
            last_m2 = m2;
            last_m3 = m3;
            last_m4 = m4;
            first_encoder = false;
            return;
        }
        float d1 = (m1 - last_m1) * MM_PER_PULSE;
        float d2 = (m2 - last_m2) * MM_PER_PULSE;
        float d3 = (m3 - last_m3) * MM_PER_PULSE;
        float d4 = (m4 - last_m4) * MM_PER_PULSE;
        last_m1 = m1;
        last_m2 = m2;
        last_m3 = m3;
        last_m4 = m4;

        float dx_local = (d1 + d2 + d3 + d4) / 4.0f;
        float dy_local = (d1 - d2 - d3 + d4) / 4.0f;
        float dyaw = (d1 + d2 - d3 - d4) / (4.0f * K);

        float cy = std::cos(curr_yaw_.load());
        float sy = std::sin(curr_yaw_.load());
        curr_x_ = curr_x_.load() + (dx_local * cy - dy_local * sy);
        curr_y_ = curr_y_.load() + (dx_local * sy + dy_local * cy);
        curr_yaw_ = curr_yaw_.load() + dyaw;
    }
}

void ChassisController::pidLoop()
{
    float kp_x = 3.0f, kp_y = 3.0f, kp_yaw = 15.0f, ki_yaw = 0.2f, integral_yaw = 0.0f; 
    // 使用全局动力配置替换硬编码
    float MIN_POWER = g_dynamics.PID_MIN_POWER;
    float MAX_POWER = g_dynamics.PID_MAX_POWER;
    int retry_counter = 0;
    float last_vx = 0.0f, last_vy = 0.0f, last_vw = 0.0f;

    while (running_)
    {
        if (first_encoder)
        {
            if (++retry_counter >= 50)
            {
                sendCmd("$upload:1,0,0#");
                retry_counter = 0;
            }
        }
        if (is_testing_.load())
        {
            usleep(20000);
            continue;
        }

        if (vel_mode_.load())
        {
            float vx = vel_vx_.load(), vy = vel_vy_.load(), vw = vel_vw_.load();
            float m1 = vx + vy + vw * K;
            float m2 = vx - vy + vw * K;
            float m3 = vx - vy - vw * K;
            float m4 = vx + vy - vw * K;
            float max_m = std::max({std::abs(m1), std::abs(m2), std::abs(m3), std::abs(m4)});
            if (max_m > 0.1f && max_m < MIN_POWER)
            {
                float s = MIN_POWER / max_m;
                m1 *= s;
                m2 *= s;
                m3 *= s;
                m4 *= s;
            }
            auto cl = [](float v)
            { return std::max(-800.0f, std::min(800.0f, v)); };
            char cmd[64];
            sprintf(cmd, "$spd:%d,%d,%d,%d#", (int)cl(m1), (int)cl(m2), (int)cl(m3), (int)cl(m4));
            sendCmd(cmd);
            target_x_ = curr_x_.load();
            target_y_ = curr_y_.load();
            target_yaw_ = curr_yaw_.load();
            usleep(20000);
            continue;
        }

        float err_x = target_x_.load() - curr_x_.load();
        float err_y = target_y_.load() - curr_y_.load();
        float err_yaw = target_yaw_.load() - curr_yaw_.load();

        float cy = std::cos(curr_yaw_.load());
        float sy = std::sin(curr_yaw_.load());
        float err_local_x = err_x * cy + err_y * sy;
        float err_local_y = -err_x * sy + err_y * cy;

        float abs_yaw = std::abs(err_yaw);
        float dist = std::sqrt(err_local_x * err_local_x + err_local_y * err_local_y);
        float vx = 0, vy = 0, vw = 0;

        if (dist < 15.0f && abs_yaw < 0.05f)
        {
            integral_yaw = 0.0f;
            last_vx = 0.0f;
            last_vy = 0.0f;
            last_vw = 0.0f;
            target_x_ = curr_x_.load();
            target_y_ = curr_y_.load();
            target_yaw_ = curr_yaw_.load();
            sendCmd("$spd:0,0,0,0#");
        }
        else
        {
            if (abs_yaw < 0.3f)
                integral_yaw += err_yaw;
            else
                integral_yaw = 0.0f;
            integral_yaw = std::max(-15.0f, std::min(15.0f, integral_yaw));

            float target_vx = err_local_x * kp_x;
            float target_vy = err_local_y * kp_y;
            float target_vw = err_yaw * kp_yaw + integral_yaw * ki_yaw;

            auto limit = [](float v, float max_v)
            { return std::max(-max_v, std::min(max_v, v)); };
            target_vx = limit(target_vx, MAX_POWER);
            target_vy = limit(target_vy, MAX_POWER);
            target_vw = limit(target_vw, 300.0f / K);  //旋转力度

            auto ramp = [](float current, float target, float step)
            {
                if (current < target)
                    return std::min(current + step, target);
                if (current > target)
                    return std::max(current - step, target);
                return target;
            };

            // 使用全局动力配置替换斜率硬编码
            vx = ramp(last_vx, target_vx, g_dynamics.RAMP_STEP_X);
            vy = ramp(last_vy, target_vy, g_dynamics.RAMP_STEP_Y);
            vw = ramp(last_vw, target_vw, g_dynamics.RAMP_STEP_YAW);

            last_vx = vx;
            last_vy = vy;
            last_vw = vw;

            float m1 = vx + vy + vw * K;
            float m2 = vx - vy + vw * K;
            float m3 = vx - vy - vw * K;
            float m4 = vx + vy - vw * K;

            float max_m = std::max({std::abs(m1), std::abs(m2), std::abs(m3), std::abs(m4)});
            if (max_m > 0.1f && max_m < MIN_POWER)
            {
                float scale = MIN_POWER / max_m;
                m1 *= scale;
                m2 *= scale;
                m3 *= scale;
                m4 *= scale;
            }

            auto clamp = [](float v)
            { return std::max(-800.0f, std::min(800.0f, v)); };
            char cmd[64];
            sprintf(cmd, "$spd:%d,%d,%d,%d#", (int)clamp(m1), (int)clamp(m2), (int)clamp(m3), (int)clamp(m4));
            sendCmd(cmd);
        }
        usleep(20000);
    }
}

bool ChassisController::init()
{
    fd_ = initPort(SystemConfig::SERIAL_PORT_CHASSIS);
    if (fd_ < 0)
        return false;
    sendCmd("$mtype:1#");
    usleep(100000);
    sendCmd("$mphase:30#");
    usleep(100000);
    sendCmd("$mline:11#");
    usleep(100000);
    sendCmd("$wdiameter:97#");
    usleep(100000);
    sendCmd("$deadzone:1900#");
    usleep(100000);
    sendCmd("$upload:1,0,0#");
    usleep(100000);

    running_ = true;
    rx_thread_ = std::thread([this]()
                             {
        char buf[128];
        while (running_) {
            int n = read(fd_, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                rx_buffer_ += buf;
                size_t pos;
                while ((pos = rx_buffer_.find('#')) != std::string::npos) {
                    std::string packet = rx_buffer_.substr(0, pos + 1);
                    parseEncoder(packet);
                    rx_buffer_.erase(0, pos + 1);
                }
                if (rx_buffer_.length() > 512) rx_buffer_.clear();
            } else { usleep(5000); }
        } });
    pid_thread_ = std::thread(&ChassisController::pidLoop, this);
    std::cout << "[Chassis] 底盘闭环控制中枢已就绪。" << std::endl;
    return true;
}

void ChassisController::blindTest()
{
    std::thread([this]()
                {
        is_testing_ = true; usleep(50000);      
        std::cout << "\n>>> [物理层排查] 盲走测试开始 (纯开环 2 秒) <<<" << std::endl;
        
        // 盲走测试同步替换为基准测试速度
        char cmd[64];
        int spd = (int)g_dynamics.CMD_SPEED_BASE;
        sprintf(cmd, "$spd:%d,%d,%d,%d#", spd, spd, spd, spd);
        sendCmd(cmd); 
        usleep(2000000); 
        sendCmd("$spd:0,0,0,0#"); usleep(10000);
        sendCmd("$mtype:1#"); 
        std::cout << "\n>>> [物理层排查] 盲走测试结束，已切断动力。 <<<\n" << std::endl;
        resetPosition();
        is_testing_ = false; })
        .detach();
}

void ChassisController::resetPosition()
{
    // 清空底层 PID 里程计
    curr_x_ = 0;
    curr_y_ = 0;
    curr_yaw_ = 0;
    target_x_ = 0;
    target_y_ = 0;
    target_yaw_ = 0;

    // 【关键新增】：彻底清空全局路径规划坐标大脑！
    nav_x_ = 0.0f;
    nav_y_ = 0.0f;
    nav_yaw_ = 0.0f;

    sendCmd("$spd:0,0,0,0#");
    usleep(10000);
    sendCmd("$mtype:1#");
}
void ChassisController::emergencyStop()
{
    target_x_ = curr_x_.load();
    target_y_ = curr_y_.load();
    target_yaw_ = curr_yaw_.load();
    sendCmd("$spd:0,0,0,0#");
    usleep(10000);
    sendCmd("$mtype:1#");
}

void ChassisController::setAbsoluteTarget(float x_cm, float y_cm)
{
    target_x_ = x_cm * 10.0f;
    target_y_ = y_cm * 10.0f;
}

void ChassisController::moveRelative(float dx_cm, float dy_cm)
{
    // 1. 计算直线直线距离
    float dist = std::sqrt(dx_cm * dx_cm + dy_cm * dy_cm);

    // 2. 小距离微调分支 (阈值设为 5.0 厘米，可以根据实车情况调整)
    if (dist > 0.1f && dist <= 1000.0f)
    {
        std::cout << "[Chassis] 距离小 (" << dist << "cm)，启动开环微动补偿！" << std::endl;

        // 开启一个独立线程执行短促的开环脉冲，防止阻塞主串口接收
        std::thread([this, dx_cm, dy_cm, dist]()
                    {
            // 设置一个能绝对克服静摩擦力的基础速度 (建议在 150~250 之间)
            float kick_speed = g_dynamics.CMD_KICK_SPEED; 
            
            // 估算时间：假设速度200对应约 15cm/s，那么 1cm 大概需要 0.066 秒
            // 这里的标定值已抽离为 g_dynamics.KICK_CM_PER_SEC，可以根据小车实际爆发速度微调
            float time_sec = dist / g_dynamics.KICK_CM_PER_SEC; 
            int sleep_us = (int)(time_sec * 1000000);

            // 计算 X 和 Y 方向的速度分量
            float dir_x = dx_cm / dist;
            float dir_y = dy_cm / dist;

            // 切入速度直控模式，爆发起步
            setVelocity(dir_x * kick_speed, dir_y * kick_speed, 0.0f);
            
            // 延时等待行驶完成
            usleep(sleep_us);
            
            // 紧急刹车并退出速度模式
            stopVelocity();
            
            // 同步刷新 PID 的目标靶点，防止切回 PID 后发生反向回调
            target_x_ = curr_x_.load();
            target_y_ = curr_y_.load(); })
            .detach();

        return; // 小距离处理完毕，直接返回，不再执行后续 PID
    }

    // 3. 常规大距离 PID 逻辑 (保持原样)
    float cy = std::cos(curr_yaw_.load());
    float sy = std::sin(curr_yaw_.load());
    float dx_mm = dx_cm * 10.0f, dy_mm = dy_cm * 10.0f;
    target_x_ = curr_x_.load() + (dx_mm * cy - dy_mm * sy);
    target_y_ = curr_y_.load() + (dx_mm * sy + dy_mm * cy);
}

void ChassisController::turnRelative(float deg) { target_yaw_ = target_yaw_.load() + (deg * M_PI / 180.0f); }
void ChassisController::setVelocity(float vx, float vy, float vw)
{
    vel_mode_.store(true);
    vel_vx_.store(vx);
    vel_vy_.store(vy);
    vel_vw_.store(vw);
}
void ChassisController::stopVelocity()
{
    vel_mode_.store(false);
    sendCmd("$spd:0,0,0,0#");
    target_x_ = curr_x_.load();
    target_y_ = curr_y_.load();
    target_yaw_ = curr_yaw_.load();
}

void ChassisController::planPath(float tx, float ty, float tyaw)
{
    std::thread([this, tx, ty, tyaw]()
                {
                    auto normalize_yaw = [](float y)
                    {
                        while (y > 180.0f)
                            y -= 360.0f;
                        while (y <= -180.0f)
                            y += 360.0f;
                        return y;
                    };

                    auto move_wait = [this](float forward_cm, float right_cm, float speed)
                    {
                        float dist = std::sqrt(forward_cm * forward_cm + right_cm * right_cm);
                        if (dist < 0.5f)
                            return;
                        this->moveRelative(forward_cm, right_cm);
                        // 使用全局缓冲常数
                        int wait_ms = (int)((dist / speed) * 1000) + g_dynamics.WAIT_MS_OFFSET;
                        usleep(wait_ms * 1000);
                    };

                    auto turn_wait = [this](float deg, float speed)
                    {
                        if (std::abs(deg) < 1.0f)
                            return;
                        this->turnRelative(deg);
                        // 使用全局缓冲常数
                        int wait_ms = (int)((std::abs(deg) / speed) * 1000) + g_dynamics.WAIT_MS_OFFSET;
                        usleep(wait_ms * 1000);
                    };

                    float dyaw = normalize_yaw(tyaw - nav_yaw_);
                    float rad = nav_yaw_ * M_PI / 180.0f;
                    float dx_global = tx - nav_x_;
                    float dy_global = ty - nav_y_;

                    // ==============================================================
                    // 【终极坐标系对齐】：Yaw=0 代表车头朝向 +Y 轴 (正前方)
                    // 局部前后 = X全局差*sin + Y全局差*cos
                    // 局部左右 = X全局差*cos - Y全局差*sin
                    // ==============================================================
                    float dy_local = dx_global * std::sin(rad) + dy_global * std::cos(rad); // 局部向前 Fwd
                    float dx_local = dx_global * std::cos(rad) - dy_global * std::sin(rad); // 局部向右 Right

                    std::cout << "\n>>> [路径规划] 起点:(" << nav_x_ << "," << nav_y_ << " 朝向:" << nav_yaw_ << ")" << std::endl;
                    std::cout << ">>> [路径规划] 目标:(" << tx << "," << ty << " 朝向:" << tyaw << ")" << std::endl;
                    std::cout << ">>> [路径规划] 解算动作: 需前后 " << dy_local << " cm, 需左右 " << dx_local << " cm" << std::endl;

                    // ==============================================================
                    // 情况 4：需要旋转180度
                    // ==============================================================
                    if (std::abs(dyaw) > 179.0f)
                    {
                        // 拔出工位：后退 15cm
                        move_wait(-15.0f, 0.0f, g_dynamics.PLAN_MOVE_SPEED);

                        // 同步更新全局里程碑（套用正运动学）
                        nav_x_ += -15.0f * std::sin(rad);
                        nav_y_ += -15.0f * std::cos(rad);

                        // 让底盘执行一次 180 度的连续旋转 （补了10 度）
                        turn_wait(190.0f, g_dynamics.PLAN_TURN_SPEED);

                        nav_yaw_ = normalize_yaw(nav_yaw_ + 180.0f);
                        dyaw = 0.0f;
                        rad = nav_yaw_ * M_PI / 180.0f;
                        dx_global = tx - nav_x_;
                        dy_global = ty - nav_y_;

                        // 重新计算局部坐标
                        dy_local = dx_global * std::sin(rad) + dy_global * std::cos(rad);
                        dx_local = dx_global * std::cos(rad) - dy_global * std::sin(rad);
                    }

                    // ==============================================================
                    // 情况 1：不需要最终的朝向旋转
                    // ==============================================================
                    if (std::abs(dyaw) < 1.0f)
                    {
                        if (std::abs(dx_local) > 0.5f)
                        {
                            // 横向 < 15cm：直接斜线开环平移，不做旋转-前进-旋转的复杂路由
                            if (std::abs(dx_local) < 15.0f)
                            {
                                float dist_total = std::sqrt(dx_local * dx_local + dy_local * dy_local);
                                float speed = g_dynamics.CMD_SPEED_BASE;
                                float cm_per_sec = g_dynamics.CM_PER_SEC_FWD; // 斜向综合按照前向参数预估
                                float time_sec = dist_total / cm_per_sec + g_dynamics.T_COMP_STARTUP_SEC;

                                float vx = (dy_local / dist_total) * speed;
                                float vy = (dx_local / dist_total) * speed;

                                std::cout << ">>> [路径规划] 小横向斜线开环: 前后" << dy_local
                                          << "cm 左右" << dx_local << "cm" << std::endl;
                                this->setVelocity(vx, vy, 0.0f);
                                usleep((int)(time_sec * 1000000));
                                this->stopVelocity();
                                usleep(300000);

                                target_x_ = curr_x_.load();
                                target_y_ = curr_y_.load();
                            }
                            else
                            {
                                float first_y = dy_local - 15.0f;

                                if (std::abs(first_y) > 0.5f)
                                {
                                    move_wait(first_y, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                                }

                                if (dx_local > 0)
                                {
                                    turn_wait(90.0f, g_dynamics.PLAN_TURN_SPEED);
                                    move_wait(dx_local, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                                    turn_wait(-90.0f, g_dynamics.PLAN_TURN_SPEED);
                                }
                                else
                                {
                                    turn_wait(-90.0f, g_dynamics.PLAN_TURN_SPEED);
                                    move_wait(-dx_local, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                                    turn_wait(90.0f, g_dynamics.PLAN_TURN_SPEED);
                                }

                                move_wait(15.0f, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                            }
                        }
                        else
                        {
                            move_wait(dy_local, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                        }
                    }
                    // ==============================================================
                    // 情况 2 / 情况 3：需要旋转 90 度
                    // ==============================================================
                    else if (std::abs(dyaw) > 89.0f && std::abs(dyaw) < 91.0f)
                    {
                        move_wait(dy_local, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                        turn_wait(dyaw, g_dynamics.PLAN_TURN_SPEED);
                        float move_after = (dyaw > 0) ? dx_local : -dx_local;
                        move_wait(move_after, 0.0f, g_dynamics.PLAN_MOVE_SPEED);
                    }

                    // 终点状态结算
                    nav_x_ = tx;
                    nav_y_ = ty;
                    nav_yaw_ = normalize_yaw(tyaw);

                    sendToMonitor("CHASSIS_DONE\r\n"); })
        .detach();
}

void ChassisController::executeAlignManeuver(float dx, float dy, float dyaw)
{
    std::thread([this, dx, dy, dyaw]()
                {
        // ==========================================================
        // 【轴向映射】：小车(右+X, 前+Y) vs 机械臂(后+X, 右+Y)
        // ==========================================================
        float move_fwd   = -dx;   // 正数代表需要前进，负数代表后退
        float move_right = dy;    // 正数代表需要向右，负数代表向左
        float turn_a     = dyaw;  // 正数代表顺时针(右)旋转，负数逆时针(左)旋转

        // 异常大误差的“降维试探”保护机制
        if (std::abs(move_fwd) > 15.0f) {
            move_fwd = (move_fwd > 0) ? 5.0f : -5.0f;
        }
        if (std::abs(move_right) > 8.0f) {
            move_right = (move_right > 0) ? 5.0f : -5.0f;
        }
        if (std::abs(turn_a) > 40.0f) {
            turn_a = (turn_a > 0) ? 15.0f : -15.0f;
        }

        std::cout << "\n>>> [底盘开环对齐] 动作防暴走处理后实际执行 -> "
                  << "需" << (move_fwd >= 0 ? "前进 " : "后退 ") << std::abs(move_fwd) << " cm | "
                  << "需" << (move_right >= 0 ? "右移 " : "左移 ") << std::abs(move_right) << " cm | "
                  << "需" << (turn_a >= 0 ? "右转 " : "左转 ") << std::abs(turn_a) << " 度" 
                  << std::endl;

        // 改为调用全局动力配置
        float speed_base = g_dynamics.CMD_SPEED_BASE;    
        float cm_per_sec_fwd = g_dynamics.CM_PER_SEC_FWD; 
        float cm_per_sec_lat = g_dynamics.CM_PER_SEC_LAT; 
        float back_dist = 0.0f;       

        // ==========================================================
        // 【核心修复】：改为互斥状态机 (else if)。
        // 阈值匹配 Monitor 的判定条件。每次只执行一个优先级最高的动作，
        // 执行完立刻拍照刷新误差，绝对不再拿旧数据瞎跑！
        // ==========================================================
        if (std::abs(turn_a) > 2.0f) {
            std::cout << ">>> [底盘开环对齐] 动作1: 复合旋转补偿..." << std::endl;
            
            float retreat_cm = 10.0f;
            float t_move = retreat_cm / cm_per_sec_fwd + g_dynamics.T_COMP_STARTUP_SEC; 
            
            // 采用你的正切补偿思路：计算偏航角对应的弧度
            float rad = std::abs(turn_a) * M_PI / 180.0f;
            
            // 将所需横向距离转化为平滑的横向分速度
            // 乘以 (CM_PER_SEC_FWD / CM_PER_SEC_LAT) 是为了消除麦轮横纵阻力不同的物理差异
            float vy_diag_abs = speed_base * std::tan(rad) * (cm_per_sec_fwd / cm_per_sec_lat);
            
            // 限制一个最大横移速度，防止角度过大时暴走
            vy_diag_abs = std::min(vy_diag_abs, speed_base * 0.5f);

            // 决定左右方向
            float vy_diag = (turn_a < 0) ? vy_diag_abs : -vy_diag_abs; 

            // 1. 平滑斜向后退拔出
            this->setVelocity(-speed_base, vy_diag, 0.0f);
            usleep((int)(t_move * 1000000));
            this->stopVelocity();
            
            usleep(500000); 

            // 2. 底盘原地闭环旋转
            this->turnRelative(turn_a);
            int wait_ms = (int)((std::abs(turn_a) / g_dynamics.DEG_PER_SEC_TURN) * 1000) + g_dynamics.WAIT_MS_OFFSET;
            usleep(wait_ms * 1000);
            usleep(500000);

            // 3. 纯直线插回 (因为第一步的斜向后退已经把旋转带来的横向偏差预先抵消了，现在车头已经完全正对靶心！)
            this->setVelocity(speed_base, 0.0f, 0.0f);
            usleep((int)(t_move * 1000000)); 
            this->stopVelocity();
            usleep(300000);
        }

        else if (std::abs(move_right) > 1.5f) {
            std::cout << ">>> [底盘开环对齐] 动作2: 横向补偿平移..." << std::endl;
            this->setVelocity(-speed_base, 0.0f, 0.0f);
            usleep(300000);
            
            float t_lat = std::abs(move_right) / cm_per_sec_lat;
            float vy_val = (move_right > 0) ? speed_base : -speed_base; 
            this->setVelocity(-speed_base, vy_val, 0.0f);
            usleep((int)(t_lat * 1000000));
            
            this->stopVelocity();
            usleep(400000); 
            
            // 因为解耦了，所以顺带在这里收尾一点纵向偏差
            float back_dist = 0.3f * cm_per_sec_fwd + t_lat * (cm_per_sec_fwd * 0.4f); 
            float target_fwd = move_fwd + back_dist;
            if (std::abs(target_fwd) > 1.0f) {
                std::cout << ">>> [底盘开环对齐] 动作2附带: 纵向切入..." << std::endl;
                // 纵向切入强制加上起步时间补偿！
                float t_fwd = std::abs(target_fwd) / cm_per_sec_fwd + g_dynamics.T_COMP_SHORT_STARTUP_SEC;
                float vx_val = (target_fwd > 0) ? speed_base : -speed_base; 
                this->setVelocity(vx_val, 0.0f, 0.0f);
                usleep((int)(t_fwd * 1000000));
                this->stopVelocity();
                usleep(300000);
            }
        }
        
        // 纵向阈值从 2.0 降至 1.5 确保不会漏判
        else if (std::abs(move_fwd) > 1.5f) {
            std::cout << ">>> [底盘开环对齐] 动作3: 纯纵向归位..." << std::endl;
            
            if (move_fwd > 0) {
                std::cout << ">>> [底盘开环对齐] 策略: 目标为前进，执行防撞回拉起步..." << std::endl;
                
                // 【完美对称】：设定一个明确的后退距离（比如 10.0 厘米）
                float retreat_cm = 10.0f; 
                
                // 1. 后退阶段：距离换算时间，并同样加上起步补偿！
                float t_back = retreat_cm / cm_per_sec_fwd + g_dynamics.T_COMP_STARTUP_SEC;
                this->setVelocity(-speed_base, 0.0f, 0.0f);
                usleep((int)(t_back * 1000000)); 
                this->stopVelocity();
                
                // 2. 停顿 0.5 秒，让底盘彻底停稳，消除惯性，清空串口
                usleep(500000);
                
                // 3. 前进阶段：原需前进的距离 + 刚才后退补偿的距离，再现加起步！
                float total_fwd_cm = move_fwd + retreat_cm;
                float t_fwd = total_fwd_cm / cm_per_sec_fwd + g_dynamics.T_COMP_STARTUP_SEC;
                
                this->setVelocity(speed_base, 0.0f, 0.0f);
                usleep((int)(t_fwd * 1000000));
                this->stopVelocity();
                usleep(300000);
            }
            else {
                // 目标为后退，执行“先深度后退越过目标，再前进折返”的策略，确保最终以前进姿态入位！
                std::cout << ">>> [底盘开环对齐] 策略: 目标为后退，执行[先后退，再前进]的防撞起步..." << std::endl;
                
                float retreat_cm = 10.0f; 

                // 1. 深度后退阶段：一次性后退到底（原需后退的距离 + 多退的补偿距离）
                float total_back_cm = std::abs(move_fwd) + retreat_cm;
                float t_back = total_back_cm / cm_per_sec_fwd + g_dynamics.T_COMP_STARTUP_SEC;
                
                this->setVelocity(-speed_base, 0.0f, 0.0f); // 负数代表后退
                usleep((int)(t_back * 1000000)); 
                this->stopVelocity();

                // 2. 停顿 0.5 秒，让底盘彻底停稳，消除惯性
                usleep(500000);

                // 3. 前进折返阶段：向前开回刚才多退的距离
                float t_fwd = retreat_cm / cm_per_sec_fwd + g_dynamics.T_COMP_STARTUP_SEC;
                
                this->setVelocity(speed_base, 0.0f, 0.0f); // 正数代表前进
                usleep((int)(t_fwd * 1000000));
                this->stopVelocity();
                usleep(300000);
            }
        }

        // ==========================================================
        // 【关键策略】：斩断开环积分死区！
        // 不再在这里使用 nav_x_ += ... 更新坐标，完全移交你的 NAV_ADJ 闭环系统！
        // ==========================================================
        std::cout << ">>> [底盘开环对齐] 单步动作执行完毕！移交 PnP 视觉重新核对..." << std::endl;
        sendToMonitor("ALIGN_DONE\r\n"); })
        .detach();
}

// 【新增】：Pilot 接收 Monitor 传来的首尾 PnP 漂移结果并更新大脑
void ChassisController::applyNavAdjustment(float real_fwd, float real_right)
{
    float rad = nav_yaw_ * M_PI / 180.0f;
    float d_nav_x = real_right * std::cos(rad) + real_fwd * std::sin(rad);
    float d_nav_y = -real_right * std::sin(rad) + real_fwd * std::cos(rad);

    nav_x_ += d_nav_x;
    nav_y_ += d_nav_y;

    std::cout << ">>> [全局路由] PnP 端到端精度闭环完成！更新靶点补偿: X补 " << d_nav_x << " cm, Y补 " << d_nav_y << " cm" << std::endl;
}