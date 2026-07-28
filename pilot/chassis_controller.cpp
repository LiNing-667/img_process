/**
 * @file chassis_controller.cpp
 * @brief 麦轮底盘闭环控制
 */
#include "chassis_controller.h"
#include "pilot_config.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <algorithm>

ChassisController::ChassisController() : fd_(-1), running_(false) {}

int ChassisController::initPort(const char *portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, SystemConfig::BAUD_RATE);
    cfsetispeed(&tty, SystemConfig::BAUD_RATE);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0; tty.c_oflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

void ChassisController::sendCmd(const std::string &cmd) {
    if (fd_ >= 0) write(fd_, cmd.c_str(), cmd.length());
}

void ChassisController::parseEncoder(const std::string &msg) {
    size_t idx = msg.find("$MAll:");
    if (idx == std::string::npos) idx = msg.find("$MAII:");
    if (idx == std::string::npos) idx = msg.find("$MALL:");
    if (idx == std::string::npos) return;

    long m1, m2, m3, m4;
    std::string data = msg.substr(idx);
    if (sscanf(data.c_str() + 6, "%ld,%ld,%ld,%ld#", &m1, &m2, &m3, &m4) == 4) {
        if (first_encoder) {
            last_m1 = m1; last_m2 = m2; last_m3 = m3; last_m4 = m4;
            first_encoder = false; return;
        }
        float d1 = (m1 - last_m1) * MM_PER_PULSE;
        float d2 = (m2 - last_m2) * MM_PER_PULSE;
        float d3 = (m3 - last_m3) * MM_PER_PULSE;
        float d4 = (m4 - last_m4) * MM_PER_PULSE;
        last_m1 = m1; last_m2 = m2; last_m3 = m3; last_m4 = m4;

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

void ChassisController::pidLoop() {
    float kp_x = 3.0f, kp_y = 3.0f, kp_yaw = 15.0f, ki_yaw = 0.2f, integral_yaw = 0.0f;
    float MIN_POWER = 120.0f, MAX_POWER = 400.0f;
    int retry_counter = 0;
    float last_vx = 0.0f, last_vy = 0.0f, last_vw = 0.0f;

    while (running_) {
        if (first_encoder) {
            if (++retry_counter >= 50) { sendCmd("$upload:1,0,0#"); retry_counter = 0; }
        }
        if (is_testing_.load()) { usleep(20000); continue; }

        if (vel_mode_.load()) {
            float vx = vel_vx_.load(), vy = vel_vy_.load(), vw = vel_vw_.load();
            float m1 = vx + vy + vw * K; float m2 = vx - vy + vw * K;
            float m3 = vx - vy - vw * K; float m4 = vx + vy - vw * K;
            float max_m = std::max({std::abs(m1), std::abs(m2), std::abs(m3), std::abs(m4)});
            if (max_m > 0.1f && max_m < MIN_POWER) {
                float s = MIN_POWER / max_m;
                m1 *= s; m2 *= s; m3 *= s; m4 *= s;
            }
            auto cl = [](float v) { return std::max(-800.0f, std::min(800.0f, v)); };
            char cmd[64];
            sprintf(cmd, "$spd:%d,%d,%d,%d#", (int)cl(m1), (int)cl(m2), (int)cl(m3), (int)cl(m4));
            sendCmd(cmd);
            target_x_ = curr_x_.load(); target_y_ = curr_y_.load(); target_yaw_ = curr_yaw_.load();
            usleep(20000); continue;
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

        if (dist < 15.0f && abs_yaw < 0.05f) {
            integral_yaw = 0.0f; last_vx = 0.0f; last_vy = 0.0f; last_vw = 0.0f;
            target_x_ = curr_x_.load(); target_y_ = curr_y_.load(); target_yaw_ = curr_yaw_.load();
            sendCmd("$spd:0,0,0,0#");
        } else {
            if (abs_yaw < 0.3f) integral_yaw += err_yaw; else integral_yaw = 0.0f;
            integral_yaw = std::max(-15.0f, std::min(15.0f, integral_yaw));

            float target_vx = err_local_x * kp_x;
            float target_vy = err_local_y * kp_y;
            float target_vw = err_yaw * kp_yaw + integral_yaw * ki_yaw;

            auto limit = [](float v, float max_v) { return std::max(-max_v, std::min(max_v, v)); };
            target_vx = limit(target_vx, MAX_POWER);
            target_vy = limit(target_vy, MAX_POWER);
            target_vw = limit(target_vw, 300.0f / K);

            auto ramp = [](float current, float target, float step) {
                if (current < target) return std::min(current + step, target);
                if (current > target) return std::max(current - step, target);
                return target;
            };

            vx = ramp(last_vx, target_vx, 15.0f);
            vy = ramp(last_vy, target_vy, 8.0f);
            vw = ramp(last_vw, target_vw, 15.0f);

            last_vx = vx; last_vy = vy; last_vw = vw;

            float m1 = vx + vy + vw * K; float m2 = vx - vy + vw * K;
            float m3 = vx - vy - vw * K; float m4 = vx + vy - vw * K;

            float max_m = std::max({std::abs(m1), std::abs(m2), std::abs(m3), std::abs(m4)});
            if (max_m > 0.1f && max_m < MIN_POWER) {
                float scale = MIN_POWER / max_m;
                m1 *= scale; m2 *= scale; m3 *= scale; m4 *= scale;
            }

            auto clamp = [](float v) { return std::max(-800.0f, std::min(800.0f, v)); };
            char cmd[64];
            sprintf(cmd, "$spd:%d,%d,%d,%d#", (int)clamp(m1), (int)clamp(m2), (int)clamp(m3), (int)clamp(m4));
            sendCmd(cmd);
        }
        usleep(20000);
    }
}

bool ChassisController::init() {
    fd_ = initPort(SystemConfig::SERIAL_PORT_CHASSIS);
    if (fd_ < 0) return false;
    sendCmd("$mtype:1#"); usleep(100000);
    sendCmd("$mphase:30#"); usleep(100000);
    sendCmd("$mline:11#"); usleep(100000);
    sendCmd("$wdiameter:97#"); usleep(100000);
    sendCmd("$deadzone:1900#"); usleep(100000);
    sendCmd("$upload:1,0,0#"); usleep(100000);

    running_ = true;
    rx_thread_ = std::thread([this]() {
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
        } 
    });
    pid_thread_ = std::thread(&ChassisController::pidLoop, this);
    std::cout << "[Chassis] 底盘闭环控制中枢已就绪。" << std::endl;
    return true;
}

void ChassisController::blindTest() {
    std::thread([this]() {
        is_testing_ = true; usleep(50000);      
        std::cout << "\n>>> [物理层排查] 盲走测试开始 (纯开环 2 秒) <<<" << std::endl;
        sendCmd("$spd:250,250,250,250#"); usleep(2000000); 
        sendCmd("$spd:0,0,0,0#"); usleep(10000);
        sendCmd("$mtype:1#"); 
        std::cout << "\n>>> [物理层排查] 盲走测试结束，已切断动力。 <<<\n" << std::endl;
        resetPosition();
        is_testing_ = false; 
    }).detach();
}

void ChassisController::resetPosition() {
    curr_x_ = 0; curr_y_ = 0; curr_yaw_ = 0; target_x_ = 0; target_y_ = 0; target_yaw_ = 0;
    sendCmd("$spd:0,0,0,0#"); usleep(10000); sendCmd("$mtype:1#");
}

void ChassisController::emergencyStop() {
    target_x_ = curr_x_.load(); target_y_ = curr_y_.load(); target_yaw_ = curr_yaw_.load();
    sendCmd("$spd:0,0,0,0#"); usleep(10000); sendCmd("$mtype:1#");
}

void ChassisController::setAbsoluteTarget(float x_cm, float y_cm) { target_x_ = x_cm * 10.0f; target_y_ = y_cm * 10.0f; }

void ChassisController::moveRelative(float dx_cm, float dy_cm) {
    // 1. 计算直线直线距离
    float dist = std::sqrt(dx_cm * dx_cm + dy_cm * dy_cm);

    // 2. 小距离微调分支 (阈值设为 5.0 厘米，可以根据实车情况调整)
    if (dist > 0.1f && dist <= 10.0f) {
        std::cout << "[Chassis] 距离小 (" << dist << "cm)，启动开环微动补偿！" << std::endl;
        
        // 开启一个独立线程执行短促的开环脉冲，防止阻塞主串口接收
        std::thread([this, dx_cm, dy_cm, dist]() {
            // 设置一个能绝对克服静摩擦力的基础速度 (建议在 150~250 之间)
            float kick_speed = 200.0f; 
            
            // 估算时间：假设速度200对应约 15cm/s，那么 1cm 大概需要 0.066 秒
            // 这里的 15.0f 是基准标定值，你可以根据小车实际爆发速度微调
            float time_sec = dist / 15.0f; 
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
            target_y_ = curr_y_.load();
        }).detach();
        
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
void ChassisController::setVelocity(float vx, float vy, float vw) {
    vel_mode_.store(true); vel_vx_.store(vx); vel_vy_.store(vy); vel_vw_.store(vw);
}
void ChassisController::stopVelocity() {
    vel_mode_.store(false); sendCmd("$spd:0,0,0,0#");
    target_x_ = curr_x_.load(); target_y_ = curr_y_.load(); target_yaw_ = curr_yaw_.load();
}

ChassisController g_car;