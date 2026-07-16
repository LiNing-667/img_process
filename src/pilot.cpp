/**
 * @file pilot.cpp
 * @brief 运动控制下位机 (Cerebellum Node) - 多 I2C 总线支持版
 * @details 独立管理机械臂与小车接口
 */

#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <vector>
#include <atomic>

// Linux 系统与外设通信
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// 逆运动学算法库 
#include "ik_solver.h"


// ============================================================================
// [1] 系统全局配置 (硬件接口映射)
// ============================================================================
namespace SystemConfig {
    const char* SERIAL_PORT_MONITOR  = "/dev/ttyS1";  // 连接 Monitor 串口
    const char* SERIAL_PORT_CHASSIS  = "/dev/ttyS2"; // 连接 STM32 驱动板的串口
    const int   BAUD_RATE    = B115200;
    
     // 机械臂硬件分配
    const char* I2C_DEV_ARM  = "/dev/i2c-0";   // 机械臂所在的 I2C 接口
    const int   PCA_ADDR_ARM0 = 0x40;           // 机械臂驱动板地址
    const int   PCA_ADDR_ARM1 = 0x41;          // 第二个机械臂
}
float Arm0_open = 60.0;
float Arm0_close = 150.0;
float Arm1_open = 140.0;
float Arm1_close = 30.0;
// ============================================================================
// [2] I2C 总线对象
// ============================================================================
class I2CDevice {
private:
    int fd_;
    std::mutex bus_mtx_;
    std::string dev_name_;

public:
    I2CDevice(const std::string& name) : fd_(-1), dev_name_(name) {}

    bool init() {
        fd_ = open(dev_name_.c_str(), O_RDWR);
        return fd_ >= 0;
    }

    void writeReg(int addr, uint8_t reg, uint8_t val) {
        if (fd_ < 0) return;
        // 只有同一个接口的设备才需要互斥锁排队，不同接口并行不悖
        std::lock_guard<std::mutex> lock(bus_mtx_); 
        if (ioctl(fd_, I2C_SLAVE, addr) >= 0) {
            uint8_t buf[2] = {reg, val};
            write(fd_, buf, 2);
        }
    }
};

// 实例化 I2C 硬件通道
I2CDevice g_i2c_arm(SystemConfig::I2C_DEV_ARM);


// ============================================================================
// [3] 机械臂运动控制核心
// ============================================================================
class RoboticArmController {
private:
    struct ArmState {
        bool initialized = false;
        float p[3]; float z[3]; float x[3];  
        float current_angles[6]; 
    };

    int i2c_addr_;
    ArmState states_[2]; 
    std::atomic<int> cmd_version_{0}; 

    // 为ARM2 (摄像头云台) 增加独立的记忆与防抖锁
    std::atomic<float> curr_pan_{40.0f};  // CH7 默认 40度直视
    std::atomic<float> curr_tilt_{43.0f}; // CH8 默认 43度俯视车板
    std::atomic<int> cam_cmd_version_{0};

    void normalizeVec(float v[3]) {
        float len = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (len > 1e-6f) { v[0]/=len; v[1]/=len; v[2]/=len; }
    }

public:
    RoboticArmController(int addr) : i2c_addr_(addr) {}

    bool init() {
        if (!g_i2c_arm.init()) {
            std::cerr << "[Arm] 致命错误：无法打开机械臂 I2C 总线 " << SystemConfig::I2C_DEV_ARM << std::endl;
            return false;
        }
        
        auto initBoard = [&](int addr) {
            g_i2c_arm.writeReg(addr, 0x00, 0x00); usleep(50000);
            float freq = 50.0f; 
            uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * freq) - 1.0f + 0.5f);
            g_i2c_arm.writeReg(addr, 0x00, 0x10); 
            g_i2c_arm.writeReg(addr, 0xFE, prescale); 
            g_i2c_arm.writeReg(addr, 0x00, 0x00); usleep(50000);
            g_i2c_arm.writeReg(addr, 0x00, 0xA0); 
        };

        initBoard(SystemConfig::PCA_ADDR_ARM0); // 唤醒 ARM0
        initBoard(SystemConfig::PCA_ADDR_ARM1); // 唤醒 ARM1

        std::cout << "[Arm] 双臂 PCA9685 驱动模块已就绪" << std::endl;
        return true;
    }

    void setServoAngle(int arm_id, uint8_t channel, float angle) {
        int addr = (arm_id == 0) ? SystemConfig::PCA_ADDR_ARM0 : SystemConfig::PCA_ADDR_ARM1;
        
        if (angle < 0) angle = 0; if (angle > 250) angle = 250; //最大度数
        uint16_t off = 102 + (uint16_t)((angle / 180.0f) * (512 - 102));
        uint8_t reg_base = 0x06 + 4 * channel;
        g_i2c_arm.writeReg(addr, reg_base, 0);              
        g_i2c_arm.writeReg(addr, reg_base + 1, 0);          
        g_i2c_arm.writeReg(addr, reg_base + 2, off & 0xFF); 
        g_i2c_arm.writeReg(addr, reg_base + 3, off >> 8);   
    }

    void setJointsDirect(int arm_id, const std::vector<float>& angles) {
        int ch_offset = (arm_id == 0) ? 0 : 9; 

        if (arm_id == 0) {
            setServoAngle(arm_id, ch_offset + 0, angles[0] + 110.0f);
            setServoAngle(arm_id, ch_offset + 1, -angles[1] + 120.0f);
            setServoAngle(arm_id, ch_offset + 2, angles[2] + 110.0f);
            setServoAngle(arm_id, ch_offset + 3, -angles[3] + 110.0f);
            setServoAngle(arm_id, ch_offset + 4, (180.0f - angles[4]) + 12.0f);
            setServoAngle(arm_id, ch_offset + 5, angles[5] + 99.0f);
        } else {
            setServoAngle(arm_id, ch_offset + 0, angles[0] + 108.0f);
            setServoAngle(arm_id, ch_offset + 1, -angles[1] + 108.0f);
            setServoAngle(arm_id, ch_offset + 2, angles[2] + 108.0f);
            setServoAngle(arm_id, ch_offset + 3, -angles[3] + 108.0f);
            setServoAngle(arm_id, ch_offset + 4, (180.0f - angles[4]) + 18.0f);
            setServoAngle(arm_id, ch_offset + 5, angles[5] + 41.0f);
        }
    }

    void moveSmooth(int arm_id, float t_px, float t_py, float t_pz, float t_zx, float t_zy, float t_zz, float t_xx, float t_xy, float t_xz) {
        int my_version = ++cmd_version_; 

        float target_angles[6];
        float t_z_axis[3] = {t_zx, t_zy, t_zz};
        float t_x_axis[3] = {t_xx, t_xy, t_xz};
        
        int status = ik6_pose(t_px, t_py, t_pz, t_z_axis, t_x_axis, target_angles);
        if (status != 0) {
            std::cerr << "[Arm" << arm_id << "] 警告：目标超出物理极限！" << std::endl;
            return;
        }

        ArmState& curr = states_[arm_id];
        std::vector<float> tgt(target_angles, target_angles + 6);

        if (!curr.initialized) {
            setJointsDirect(arm_id, tgt);
            for(int i=0; i<6; i++) curr.current_angles[i] = tgt[i];
            curr.p[0] = t_px; curr.p[1] = t_py; curr.p[2] = t_pz;
            curr.z[0] = t_zx; curr.z[1] = t_zy; curr.z[2] = t_zz;
            curr.x[0] = t_xx; curr.x[1] = t_xy; curr.x[2] = t_xz;
            curr.initialized = true;
            return;
        }

        std::vector<float> start(curr.current_angles, curr.current_angles + 6);

        for(int i=0; i<6; i++) curr.current_angles[i] = tgt[i];
        curr.p[0] = t_px; curr.p[1] = t_py; curr.p[2] = t_pz;
        curr.z[0] = t_zx; curr.z[1] = t_zy; curr.z[2] = t_zz;
        curr.x[0] = t_xx; curr.x[1] = t_xy; curr.x[2] = t_xz;

        std::thread([this, arm_id, start, tgt, my_version]() {
            float max_angle_diff = 0.0f; 
            for(int i=0; i<6; i++) max_angle_diff = std::max(max_angle_diff, std::abs(tgt[i] - start[i]));

            float speed_deg_per_sec = 60.0f; 
            float loop_delay_sec = 0.02f;    
            int steps = std::max(1, (int)(max_angle_diff / (speed_deg_per_sec * loop_delay_sec)));
            std::vector<float> current_step_angles(6);

            for (int i = 1; i <= steps; ++i) {
                if (cmd_version_ != my_version) return; 
                float ratio = (float)i / steps;
                for(int j=0; j<6; j++) current_step_angles[j] = start[j] + (tgt[j] - start[j]) * ratio;
                
                setJointsDirect(arm_id, current_step_angles); 
                usleep(20000); 
            }
        }).detach(); 
    }

    ArmState& getState(int arm_id) { return states_[arm_id]; }

    // ==========================================================
    // 纯物理通道多轴同步平滑插值 (绕过 IK，专治交接扭曲姿态)
    // ==========================================================
    void moveRawChannelsSmooth(int arm_id, const std::vector<float>& target_raw_angles, float time_sec = 2.0f) {
        int my_version = ++cmd_version_;
        ArmState& curr = states_[arm_id];

        // 1. 反推当前真实的物理通道角度 (从 IK 记忆中提取当前位置)
        std::vector<float> start_raw(6, 0.0f);
        if (curr.initialized) {
            if (arm_id == 0) {
                start_raw[0] = curr.current_angles[0] + 110.0f;
                start_raw[1] = -curr.current_angles[1] + 120.0f;
                start_raw[2] = curr.current_angles[2] + 110.0f;
                start_raw[3] = -curr.current_angles[3] + 110.0f;
                start_raw[4] = (180.0f - curr.current_angles[4]) + 12.0f;
                start_raw[5] = curr.current_angles[5] + 99.0f;
            } else {
                start_raw[0] = curr.current_angles[0] + 108.0f;
                start_raw[1] = -curr.current_angles[1] + 108.0f;
                start_raw[2] = curr.current_angles[2] + 108.0f;
                start_raw[3] = -curr.current_angles[3] + 108.0f;
                start_raw[4] = (180.0f - curr.current_angles[4]) + 18.0f;
                start_raw[5] = curr.current_angles[5] + 41.0f;
            }
        } else {
            start_raw = target_raw_angles; // 未初始化则跳跃
        }

        // 2. 刷新内部 IK 记忆 (防止下一次收到 XYZ 指令时发生暴走跳变)
        if (arm_id == 0) {
            curr.current_angles[0] = target_raw_angles[0] - 110.0f;
            curr.current_angles[1] = -(target_raw_angles[1] - 120.0f);
            curr.current_angles[2] = target_raw_angles[2] - 110.0f;
            curr.current_angles[3] = -(target_raw_angles[3] - 110.0f);
            curr.current_angles[4] = 180.0f - (target_raw_angles[4] - 12.0f);
            curr.current_angles[5] = target_raw_angles[5] - 99.0f;
        } else {
            curr.current_angles[0] = target_raw_angles[0] - 108.0f;
            curr.current_angles[1] = -(target_raw_angles[1] - 108.0f);
            curr.current_angles[2] = target_raw_angles[2] - 108.0f;
            curr.current_angles[3] = -(target_raw_angles[3] - 108.0f);
            curr.current_angles[4] = 180.0f - (target_raw_angles[4] - 18.0f);
            curr.current_angles[5] = target_raw_angles[5] - 41.0f;
        }
        curr.initialized = true;

        // 3. 开启多线程同步匀速平滑插值
        std::thread([this, arm_id, start_raw, target_raw_angles, my_version, time_sec]() {
            float loop_delay_sec = 0.02f;    // 50Hz 刷新率
            int steps = std::max(1, (int)(time_sec / loop_delay_sec));
            int ch_offset = (arm_id == 0) ? 0 : 9; // ARM0 从 CH0开始，ARM1 从 CH9开始

            for (int i = 1; i <= steps; ++i) {
                if (cmd_version_ != my_version) return; // 随时响应打断
                float ratio = (float)i / steps;
                for (int j = 0; j < 6; j++) {
                    float cur_angle = start_raw[j] + (target_raw_angles[j] - start_raw[j]) * ratio;
                    setServoAngle(arm_id, ch_offset + j, cur_angle);
                }
                usleep(20000); 
            }
        }).detach();
    }
    // ==========================================================
    //  ARM2 (云台) 专属多线程平滑控制
    // ==========================================================
    void moveCameraSmooth(float target_pan, float target_tilt) {
        int my_version = ++cam_cmd_version_;
        float start_pan = curr_pan_.load();
        float start_tilt = curr_tilt_.load();

        std::thread([this, start_pan, start_tilt, target_pan, target_tilt, my_version]() {
            float diff_pan = target_pan - start_pan;
            float diff_tilt = target_tilt - start_tilt;
            float max_diff = std::max(std::abs(diff_pan), std::abs(diff_tilt));

            float speed_deg_per_sec = 60.0f; // 60 度/秒
            float loop_delay_sec = 0.02f;    // 20ms
            int steps = std::max(1, (int)(max_diff / (speed_deg_per_sec * loop_delay_sec)));

            for (int i = 1; i <= steps; ++i) {
                // 如果运行中途收到了新的 CAM 或 ARM2 指令，立刻打断并让出控制权
                if (cam_cmd_version_ != my_version) return; 
                
                float ratio = (float)i / steps;
                float cur_p = start_pan + diff_pan * ratio;
                float cur_t = start_tilt + diff_tilt * ratio;

                // 强制将命令指向 ARM1 板子上的 7 和 8 口
                setServoAngle(1, 7, cur_p);
                setServoAngle(1, 8, cur_t);
                
                // 实时更新当前角度记忆
                curr_pan_.store(cur_p);
                curr_tilt_.store(cur_t);

                usleep(20000); 
            }
        }).detach();
    }

    // 云台瞬间指向直控 (供 Monitor PID 自适应调用)
    void setCameraDirect(float target_pan, float target_tilt) {
        cam_cmd_version_++; // 发出打断信号，终止可能正在进行的平滑移动
        setServoAngle(1, 7, target_pan);
        setServoAngle(1, 8, target_tilt);
        curr_pan_.store(target_pan);
        curr_tilt_.store(target_tilt);
    }

    // 【新增】供单轴手动指令 CH 7/8 更新内置记忆
    void notifyCameraManualSet(int channel, float angle) {
        cam_cmd_version_++; // 只要人为接管了，就打断之前的自动平滑
        if (channel == 7) curr_pan_.store(angle);
        if (channel == 8) curr_tilt_.store(angle);
    }
};

RoboticArmController g_arm(SystemConfig::PCA_ADDR_ARM0);

// ============================================================================
// [3.5] 麦轮底盘全向闭环控制核心 
// ============================================================================
class ChassisController {
private:
    int fd_;
    std::string rx_buffer_;
    std::thread pid_thread_;
    std::thread rx_thread_;
    bool running_;

    const float Lx = 210.0f; 
    const float Ly = 135.0f; 
    const float K  = Lx + Ly; 
    
    // 放大单脉冲代表的距离后，小车只需较少的脉冲就能判定到达目标，1.20f可改===========//
    const float MM_PER_PULSE = 0.1953f * 1.10f;

    std::atomic<float> curr_x_{0}, curr_y_{0}, curr_yaw_{0};
    std::atomic<float> target_x_{0}, target_y_{0}, target_yaw_{0};
    
    long last_m1=0, last_m2=0, last_m3=0, last_m4=0;
    bool first_encoder = true;
    std::atomic<bool> is_testing_{false};

    int initPort(const char* portname) {
        int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) return -1;
        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, SystemConfig::BAUD_RATE);
        cfsetispeed(&tty, SystemConfig::BAUD_RATE);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK; tty.c_lflag = 0; tty.c_oflag = 0;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tty);
        return fd;
    }

    void sendCmd(const std::string& cmd) {
        if (fd_ >= 0) write(fd_, cmd.c_str(), cmd.length());
    }

    void parseEncoder(const std::string& msg) {
        size_t idx = msg.find("$MAll:");
        if (idx == std::string::npos) idx = msg.find("$MAII:");
        if (idx == std::string::npos) idx = msg.find("$MALL:");
        if (idx == std::string::npos) return;

        long m1, m2, m3, m4;
        std::string data = msg.substr(idx);
        if (sscanf(data.c_str() + 6, "%ld,%ld,%ld,%ld#", &m1, &m2, &m3, &m4) == 4) {
            
            if (first_encoder) {
                last_m1 = m1; last_m2 = m2; last_m3 = m3; last_m4 = m4;
                first_encoder = false;
                return;
            }
            
            float d1 = (m1 - last_m1) * MM_PER_PULSE;
            float d2 = (m2 - last_m2) * MM_PER_PULSE;
            float d3 = (m3 - last_m3) * MM_PER_PULSE;
            float d4 = (m4 - last_m4) * MM_PER_PULSE;
            last_m1 = m1; last_m2 = m2; last_m3 = m3; last_m4 = m4;

            float dx_local = (d1 + d2 + d3 + d4) / 4.0f;
            float dy_local = (d1 - d2 - d3 + d4) / 4.0f;
            float dyaw     = (d1 + d2 - d3 - d4) / (4.0f * K); 

            float cy = std::cos(curr_yaw_.load());
            float sy = std::sin(curr_yaw_.load());
            curr_x_ = curr_x_.load() + (dx_local * cy - dy_local * sy);
            curr_y_ = curr_y_.load() + (dx_local * sy + dy_local * cy);
            curr_yaw_ = curr_yaw_.load() + dyaw;
        }
    }

    void pidLoop() {
        float kp_x = 3.0f;     
        float kp_y = 3.0f;     
        float kp_yaw = 15.0f; 
        float ki_yaw = 0.2f;  
        float integral_yaw = 0.0f;

        float MIN_POWER = 120.0f; 
        float MAX_POWER = 400.0f; 

        int retry_counter = 0; 
        float last_vx = 0.0f, last_vy = 0.0f, last_vw = 0.0f;

        while (running_) {
            if (first_encoder) {
                if (++retry_counter >= 50) { 
                    sendCmd("$upload:1,0,0#");
                    retry_counter = 0;
                }
            }

            if (is_testing_.load()) {
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

            if (dist < 15.0f && abs_yaw < 0.05f) {
                integral_yaw = 0.0f; 
                last_vx = 0.0f; last_vy = 0.0f; last_vw = 0.0f; 
                
                target_x_ = curr_x_.load();
                target_y_ = curr_y_.load();
                target_yaw_ = curr_yaw_.load();
                sendCmd("$spd:0,0,0,0#");
            } 
            else {
                if (abs_yaw < 0.3f) {
                    integral_yaw += err_yaw;
                } else {
                    integral_yaw = 0.0f; 
                }
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

                float m1 = vx + vy + vw * K; 
                float m2 = vx - vy + vw * K; 
                float m3 = vx - vy - vw * K; 
                float m4 = vx + vy - vw * K; 

                float max_m = std::max({std::abs(m1), std::abs(m2), std::abs(m3), std::abs(m4)});
                if (max_m > 0.1f && max_m < MIN_POWER) {
                    float scale = MIN_POWER / max_m;
                    m1 *= scale; m2 *= scale; m3 *= scale; m4 *= scale;
                }

                auto clamp = [](float v) { return std::max(-800.0f, std::min(800.0f, v)); };
                m1 = clamp(m1); m2 = clamp(m2); m3 = clamp(m3); m4 = clamp(m4);

                char cmd[64];
                sprintf(cmd, "$spd:%d,%d,%d,%d#", (int)m1, (int)m2, (int)m3, (int)m4);
                sendCmd(cmd);
            }

            usleep(20000); 
        }
    }
    
public:
    ChassisController() : fd_(-1), running_(false) {}

    void blindTest() {
        std::thread([this]() {
            is_testing_ = true; 
            usleep(50000);      
            
            std::cout << "\n==================================================" << std::endl;
            std::cout << ">>> [物理层排查] 盲走测试开始 (纯开环 2 秒) <<<" << std::endl;
            std::cout << ">>> 正在强制下发底层指令: $spd:250,250,250,250#" << std::endl;
            std::cout << "==================================================\n" << std::endl;

            sendCmd("$spd:250,250,250,250#"); 
            usleep(2000000); 
            
            sendCmd("$spd:0,0,0,0#"); 
            usleep(10000);
            sendCmd("$mtype:1#"); 
            
            std::cout << "\n>>> [物理层排查] 盲走测试结束，已切断动力。 <<<\n" << std::endl;
            
            curr_x_ = 0; curr_y_ = 0; curr_yaw_ = 0;
            target_x_ = 0; target_y_ = 0; target_yaw_ = 0;
            is_testing_ = false; 
        }).detach();
    }

    bool init() {
        fd_ = initPort(SystemConfig::SERIAL_PORT_CHASSIS);
        if (fd_ < 0) return false;

        sendCmd("$mtype:1#");     usleep(100000);
        sendCmd("$mphase:30#");   usleep(100000);
        sendCmd("$mline:11#");    usleep(100000);
        sendCmd("$wdiameter:97#");usleep(100000);
        sendCmd("$deadzone:1900#");usleep(100000); 
        sendCmd("$upload:1,0,0#");usleep(100000); 

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

    void resetPosition() {
        curr_x_ = 0; curr_y_ = 0; curr_yaw_ = 0;
        target_x_ = 0; target_y_ = 0; target_yaw_ = 0;
        sendCmd("$spd:0,0,0,0#"); 
        usleep(10000);
        sendCmd("$mtype:1#");
    }
    void emergencyStop() {
        target_x_ = curr_x_.load();
        target_y_ = curr_y_.load();
        target_yaw_ = curr_yaw_.load();
        sendCmd("$spd:0,0,0,0#"); 
        usleep(10000);
        sendCmd("$mtype:1#");
    }
    void setAbsoluteTarget(float x_cm, float y_cm) {
        target_x_ = x_cm * 10.0f; 
        target_y_ = y_cm * 10.0f;
    }
    void moveRelative(float dx_cm, float dy_cm) {
        float cy = std::cos(curr_yaw_.load());
        float sy = std::sin(curr_yaw_.load());
        float dx_mm = dx_cm * 10.0f;
        float dy_mm = dy_cm * 10.0f;
        target_x_ = curr_x_.load() + (dx_mm * cy - dy_mm * sy);
        target_y_ = curr_y_.load() + (dx_mm * sy + dy_mm * cy);
    }
    void turnRelative(float deg) {
        target_yaw_ = target_yaw_.load() + (deg * M_PI / 180.0f);
    }
};

ChassisController g_car; 

int g_monitor_fd = -1; 
void sendToMonitor(const std::string& msg) {
    if (g_monitor_fd >= 0) {
        write(g_monitor_fd, msg.c_str(), msg.length());
    }
}

// ============================================================================
// [4] 动作组与宏指令库 (Demo Manager)
// ============================================================================
namespace DemoManager {

    float g_cache_091_px = 0.0f;
    float g_cache_091_py = 0.0f;
    float g_cache_091_pz = 0.0f;
    bool  g_has_cache_091 = false;
    // ==============小车===============//

    void executeChassisAutoMove(float px, float py) {
        std::thread([=]() {
            // 根据左臂 PnP 期望的最终抓取坐标
            float target_x = -15.6f;
            float target_y = 15.0f;
            float forward_cm = target_x - px;
            float right_cm = py - target_y;
            std::cout << "\n>>> [底盘追踪] 当前物体: X=" << px << ", Y=" << py << std::endl;
            std::cout << ">>> [底盘追踪] 期望到达: X=" << target_x << ", Y=" << target_y << std::endl;
            std::cout << ">>> [底盘追踪] 执行相对移动 -> 前进 " << forward_cm << " cm, 向右 " << right_cm << " cm" << std::endl;
            g_car.moveRelative(forward_cm, right_cm);
            // 估算行驶时间，让底盘有充足的时间跑完
            // 假设小车均速 15cm/s，外加 1.5 秒刹车缓冲时间
            float max_dist = std::max(std::abs(forward_cm), std::abs(right_cm));
            float move_time = max_dist / 15.0f + 1.5f; 
            usleep((int)(move_time * 1000000));

            std::cout << ">>> [底盘追踪] 移动结束，向大脑回传完成信号..." << std::endl;
            sendToMonitor("CHASSIS_DONE\r\n"); // 报告移动完毕，触发 Monitor 的下一步状态
        }).detach();
    }

    //===============机械臂================//
    void runDemoSequence() {
        std::cout << "\n>>> [DEMO]  <<<" << std::endl;
        g_arm.setServoAngle(0, 6, Arm0_open); usleep(100000); 
        g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000); 
        g_arm.setServoAngle(1, 15, Arm1_open); usleep(1000000); 
        g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000); 
        std::cout << ">>> 测试序列执行完毕！ <<<\n" << std::endl;
    }
    
    void executeDemo000(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); 

            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1500000); 
            //g_arm.moveSmooth(0, px-2 , py + 5.2f, -4.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.moveSmooth(0, px-0.5 , py + 5.2f, -1.0 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.moveSmooth(0, px-0.5 , py + 5.2f, -7.4 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000); 
            g_arm.setServoAngle(0, 6, Arm0_close); usleep(1000000);  
            //g_arm.moveSmooth(0, px-2 , py + 5.2f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            std::cout << ">>>  执行完毕！ <<<\n" << std::endl;
            g_arm.moveSmooth(0, -13 , 12 , 5, -0.1, 0, -1, -1, 0, 0); 

        }).detach();
    }

    
    void executeDemo111(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            // 实际要去的三维坐标点
            float arm_x = px - 2.0f;
            float arm_y = py - 10.0f;
            // 让手腕顺着从基座到目标点的向量方向延伸
            float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
            float f_xx = arm_x / length;
            float f_xy = arm_y / length;
            float f_xz = 0.0f;
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);   
            std::cout << "\n>>>开始执行demo111 <<<" << std::endl;
            g_arm.setServoAngle(1, 15, 110.0f); usleep(1500000); 
            // 手腕是自然歪斜姿态
            g_arm.moveSmooth(1, arm_x , arm_y , -2.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);  
            // 依据实测数据：Y=-3 -> 190, Y=0 -> 180, Y=5 -> 173
            // 这里的 arm_y 就是实际 Y 坐标 (py - 10.0)
            float ch14_angle = 0.242f * arm_y * arm_y - 2.608f * arm_y + 180.0f;
            if (ch14_angle > 220.0f) ch14_angle = 220.0f;
            if (ch14_angle < 90.0f) ch14_angle = 90.0f;
            float raw_physical_angle = ch14_angle - 55.0f;
            g_arm.setServoAngle(1, 14, raw_physical_angle); usleep(1500000);
            std::cout << ">>> 执行完毕 <<<\n" << std::endl;
            sendToMonitor("FIX_111\r\n");
            
        }).detach();
    }
    
    void executeDemo112(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            g_arm.moveSmooth(1, px , py - 9.0f , 1 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000);
            std::cout << "\n>>>开始执行demo111 <<<" << std::endl;
            g_arm.setServoAngle(1, 15, 130.0f); usleep(1500000); 
            g_arm.moveSmooth(1, px , py - 0.0f , 1 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.setServoAngle(1, 15, 50.0f); usleep(1000000);  
            std::cout << ">>> 执行完毕 <<<\n" << std::endl;
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);  
            
        }).detach();
    }
    
    
    void executeDemo021(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); 
            std::cout << "\n>>>开始执行demo000 <<<" << std::endl; 
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1500000); 
            g_arm.moveSmooth(0, px , py , 1.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.moveSmooth(0, px , py , -2.2 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            g_arm.setServoAngle(0, 6, Arm0_close); usleep(1500000);  
            g_arm.moveSmooth(0, px , py + 6.0f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            std::cout << ">>>  执行完毕！ <<<\n" << std::endl;
            g_arm.moveSmooth(0, -13 , 12 , 5, -0.1, 0, -1, -1, 0, 0); 

        }).detach();
    }
    

    void executeDemo031(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); 
            std::cout << "\n>>>开始执行demo000 <<<" << std::endl;
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1500000); 
            g_arm.moveSmooth(0, px , py+1.5 , 1.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.moveSmooth(0, px , py+1.5 , -1.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            g_arm.setServoAngle(0, 6, Arm0_close); usleep(1500000);  
            g_arm.moveSmooth(0, px , py + 6.0f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            std::cout << ">>>  执行完毕！ <<<\n" << std::endl;
            g_arm.moveSmooth(0, -13 , 12 , 5, -0.1, 0, -1, -1, 0, 0); 

        }).detach();
    }
    void executeDemo131(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            // 实际要去的三维坐标点
            float arm_x = px - 2.0f;
            float arm_y = py - 12.0f;
            // 让手腕顺着从基座到目标点的向量方向延伸
            float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
            float f_xx = arm_x / length;
            float f_xy = arm_y / length;
            float f_xz = 0.0f;
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);  
            std::cout << "\n>>>开始执行demo131 <<<" << std::endl;
            g_arm.setServoAngle(1, 15, 90.0f); usleep(1500000); 
            // 手腕是自然歪斜姿态
            g_arm.moveSmooth(1, arm_x , arm_y , -3.0 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);  
            // 依据实测数据：Y=-3 -> 190, Y=0 -> 180, Y=5 -> 173
            // 这里的 arm_y 就是实际 Y 坐标 (py - 10.0)
            float ch14_angle = 0.242f * arm_y * arm_y - 2.608f * arm_y + 180.0f;
            if (ch14_angle > 220.0f) ch14_angle = 220.0f;
            if (ch14_angle < 90.0f) ch14_angle = 90.0f;
            float raw_physical_angle = ch14_angle - 55.0f;
            g_arm.setServoAngle(1, 14, raw_physical_angle); usleep(1500000);
            std::cout << ">>> 执行完毕 <<<\n" << std::endl;
            sendToMonitor("FIX_131\r\n");
            
        }).detach();
    }
    void executeDemo132(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = 0.0f, f_zy = 0.3f, f_zz = -0.916f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            g_arm.moveSmooth(1, px-1 , py - 9.0f , -0.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000);
            std::cout << "\n>>>开始执行demo132 <<<" << std::endl;
            g_arm.setServoAngle(1, 15, Arm1_open); usleep(1500000); 
            g_arm.moveSmooth(1, px-1 , py - 3.8f , -0.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);  
            g_arm.moveSmooth(1, px-1 , py - 4.2f , 1.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000); 
            g_arm.moveSmooth(1, -10, -8 ,5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);  
            
        }).detach();
    }


    void executeDemo121(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            // 实际要去的三维坐标点
            float arm_x = px - 2.0f;
            float arm_y = py - 15.5f;
            // 让手腕顺着从基座到目标点的向量方向延伸
            float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
            float f_xx = arm_x / length;
            float f_xy = arm_y / length;
            float f_xz = 0.0f;
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);  
            g_arm.setServoAngle(1, 15, 90.0f); usleep(1500000); 
            // 手腕是自然歪斜姿态
            g_arm.moveSmooth(1, arm_x , arm_y , -2.0 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);  
            // 依据实测数据：Y=-3 -> 190, Y=0 -> 180, Y=5 -> 173
            // 这里的 arm_y 就是实际 Y 坐标 (py - 10.0)
            float ch14_angle = 0.242f * arm_y * arm_y - 2.608f * arm_y + 180.0f;
            if (ch14_angle > 220.0f) ch14_angle = 220.0f;
            if (ch14_angle < 90.0f) ch14_angle = 90.0f;
            float raw_physical_angle = ch14_angle - 55.0f;
            g_arm.setServoAngle(1, 14, raw_physical_angle); usleep(1500000);
            std::cout << ">>> 执行完毕 <<<\n" << std::endl;
            sendToMonitor("FIX_121\r\n");
            
        }).detach();
    }
    void executeDemo122(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = 0.0f, f_zy = 0.4f, f_zz = -0.916f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            g_arm.moveSmooth(1, px-1 , py - 10.0f , -1 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000);
            std::cout << "\n>>>开始执行demo132 <<<" << std::endl;
            g_arm.setServoAngle(1, 15, Arm1_open); usleep(1500000); 
            g_arm.moveSmooth(1, px-1 , py - 3.0f , -1 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);  
            g_arm.moveSmooth(1, px-1 , py - 6.0f , 2 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000); 
            g_arm.moveSmooth(1, -10, -8 ,5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);  
            
        }).detach();
    }
    void executeDemo041(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>>开始执行demo041 <<<" << std::endl;
            g_arm.setServoAngle(0, 6, 30.0f); usleep(1500000);
            g_arm.moveSmooth(0, px, py, pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            g_arm.moveSmooth(0, px, py, pz - 4.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000); 
            g_arm.setServoAngle(0, 6, 200.0f); usleep(1500000);  
            g_arm.moveSmooth(0, px, py, pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000); 
            std::cout << ">>>  执行完毕！ <<<\n" << std::endl;
            g_arm.moveSmooth(0, -15, 5 ,5, -0.1, 0, -1, -1, 0, 0); 

        }).detach();
    }

    //固定指令
    void executeDo001() {
        std::thread([]() {       
            g_arm.moveSmooth(0, -20, -3 , 0, -0.1, 0, -1, -1, 0, 0); usleep(2000000); 
            g_arm.moveSmooth(0, -20, -3 , -5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.moveSmooth(0, -20, -3 , -7, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1000000);
            g_arm.moveSmooth(0, -15, 5 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000); 
        }).detach();
    }

    //换手
    void executeDo031() {
        std::thread([]() {    
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.moveSmooth(1, -10, 0 ,8, -0.1, 0, -1, -1, 0, 0);usleep(1200000);
            // 顺序为: CH9=150, CH10=70, CH11=1, CH12=60, CH13=130, CH14=70
            std::vector<float> target_ch = {
                145.0f + 18.0f,  // CH9  
                80.0f + 18.0f,   // CH10 
                1.0f + 18.0f,    // CH11 
                70.0f + 18.0f,   // CH12 
                120.0f + 18.0f,  // CH13 
                70.0f - 55.0f    // CH14
            };
            // 调用同步平滑插值，设定 2.5 秒内缓慢移动到位
            g_arm.moveRawChannelsSmooth(1, target_ch, 1.5f); 
            usleep(500000); // 等待移动完成
            g_arm.moveSmooth(0, -12.1, -7.0 , 1.5, -0.1, 0, -1, -1, 0, 0); usleep(2300000); ////
            g_arm.setServoAngle(0, 6, Arm0_close); usleep(1000000); 
            g_arm.setServoAngle(1, 15, (Arm1_open + Arm1_close)/2); usleep(1000000);
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);
            g_arm.moveSmooth(1, -10, 0 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(2000000); 
            g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(1200000); 
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); 

            if (g_has_cache_091) {
                float px = g_cache_091_px;
                float py = g_cache_091_py;
                float pz = g_cache_091_pz;
                float f_zx = -0.20f, f_zy = 0.0f, f_zz = -1.0f;
                float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                float px_arm1 = px + 0.02f;  // X 轴极微小的标定误差补偿
                float py_arm1 = py + 18.2f;  // Y 轴平移 18.2 厘米，瞬间对齐物理空间！

                g_arm.moveSmooth(1, px_arm1 + 0.5 , py_arm1 - 3.2f , 8.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                g_arm.moveSmooth(1, px_arm1 + 0.5 , py_arm1 - 1.2f, 5.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                g_arm.setServoAngle(1, 15, Arm1_close); 
                
                g_arm.moveSmooth(0, px -0.7f, py + 12.5f , 5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                g_arm.moveSmooth(0, px -0.7f, py + 10.5f , 4.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                g_arm.moveSmooth(0, px -0.7f, py + 10.5f , 2.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
                g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000); // 组装压实后，松开爪子

                g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                g_arm.moveSmooth(1, px_arm1, py_arm1-1 , 9.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);

                g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);// 抬升并回归待命姿态
                g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);

            } else {
                std::cout << "\n>>> 没有发现 demo091 <<<" << std::endl;
            }
        }).detach();
    }

    //临时调试用
    void executeDo002() {
        std::thread([]() {    

            float f_zx = -0.37f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO091] ARM0 移动至组装目标 <<<" << std::endl;
            g_arm.moveSmooth(0, -20, 4 , 5, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, -22, -8 , 5, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.setServoAngle(0, 6, (Arm0_close + Arm0_open)/2 ); usleep(800000);
            g_arm.moveSmooth(0, -20, 4 , 5, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1200000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000);

            //换手
            g_arm.moveSmooth(1, -10, 0 ,8, -0.1, 0, -1, -1, 0, 0);usleep(1200000);
            std::vector<float> target_ch = {145.0f + 18.0f, 80.0f + 18.0f, 1.0f + 18.0f, 70.0f + 18.0f,  120.0f + 18.0f,  70.0f - 55.0f };
            g_arm.moveRawChannelsSmooth(1, target_ch, 1.5f); usleep(1500000); // 等待移动完成
            g_arm.moveSmooth(0, -11.8, -6 , 2.2, -0.1, 0, -1, -1, 0, 0); usleep(2300000); 
            g_arm.setServoAngle(0, 6, Arm0_close); usleep(1000000); 
            g_arm.setServoAngle(1, 15, 50); usleep(1000000);
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);
            g_arm.moveSmooth(1, -10, 0 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(2000000); 
            g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(200000); 
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(1200000); 

            //拼ID=2物体
            g_arm.moveSmooth(0, -17, 5 , 6, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, -17, 3 , 6, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, -17, 3 , 2, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000); // 组装压实后，松开爪子

            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);// 抬升并回归待命姿态
        }).detach();
    }

    //拼装动作（底）
    void executeDemo091(float px, float py, float pz) {
        std::thread([=]() {
            g_cache_091_px = px;
            g_cache_091_py = py;
            g_cache_091_pz = pz;
            g_has_cache_091 = true;
            float f_zx = -0.37f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO091] ARM0 移动至组装目标 <<<" << std::endl;
            g_arm.moveSmooth(0, px -2.2f, py + 18.5f , -1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px -2.0f, py + 17.6f , -3.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
            g_arm.moveSmooth(0, px -2.0f, py + 17.6f , -5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000);
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); 

        }).detach();
    }
    void executeDemo001(float px, float py, float pz) {
        std::thread([=]() {
            float px_arm1 = px + 0.02f;  
            float py_arm1 = py + 18.2f;  
            float f_zx = -0.15f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

         //   std::cout << "\n>>> [DEMO091] ARM0 移动至组装目标 <<<" << std::endl;
         //   g_arm.moveSmooth(0, px -5.0f, py+0.5 , 4.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
         //   g_arm.moveSmooth(0, px -5.0f, py+0.5 , 2.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(800000);
         //   g_arm.setServoAngle(0, 6, (Arm0_close + Arm0_open)/2 ); usleep(800000);
         //   g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1200000);
         //   g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000);
//
         //   //换手
         //   g_arm.moveSmooth(1, -10, 0 ,8, -0.1, 0, -1, -1, 0, 0);usleep(1200000);
         //   std::vector<float> target_ch = {145.0f + 18.0f, 80.0f + 18.0f, 1.0f + 18.0f, 70.0f + 18.0f,  120.0f + 18.0f,  70.0f - 55.0f };
         //   g_arm.moveRawChannelsSmooth(1, target_ch, 1.5f); usleep(1500000); // 等待移动完成
         //   g_arm.moveSmooth(0, -12.5, -6 , 2.2, -0.1, 0, -1, -1, 0, 0); usleep(2300000); 
         //   g_arm.setServoAngle(0, 6, Arm0_close); usleep(1000000); 
         //   g_arm.setServoAngle(1, 15, Arm1_open); usleep(1000000);
         //   g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
         //   g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);
         //   g_arm.moveSmooth(1, -10, 0 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(2000000); 
         //   g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(200000); 
         //   g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(1200000); 

            //拼ID=2物体
            g_arm.moveSmooth(0, px -0.0f, py + 7.5f , 5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px -0.0f, py + 6.5f , 4.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px -0.0f, py + 6.5f , 2.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000); // 组装压实后，松开爪子

            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);// 抬升并回归待命姿态

        }).detach();
    }


    void executeDemo102(float px, float py, float pz) {
        std::thread([=]() {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO102] 第二次拼装: ARM1 移动至目标 <<<" << std::endl;
            g_arm.moveSmooth(1, -13, -10 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(1000000); 
            g_arm.moveSmooth(1, px +2.2f, py + 0.8f , pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2500000);
            g_arm.moveSmooth(1, px -0.5f, py + 0.8f , pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(1, px -0.5f, py + 0.8f , pz - 0.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);

            g_arm.setServoAngle(1, 15, 150.0f); usleep(1000000); 
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);  
        }).detach();
    }

    void executeDemo002(float px, float py, float pz) {
        std::thread([=]() {
            float px_arm1 = px + 0.02f;  
            float py_arm1 = py + 18.2f;  
            float f_zx = -0.37f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            //拼ID=1物体
            g_arm.moveSmooth(1, px_arm1 + 2.0, py_arm1 + 1.0 , 6.5f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(2000000);
            g_arm.moveSmooth(1, px_arm1 + 0.5, py_arm1 + 1.0 , 5.5f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
            g_arm.moveSmooth(1, px_arm1 + 0.5, py_arm1 + 1.0 , 2.5f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
            g_arm.setServoAngle(1, 15, (Arm1_open + Arm1_close)/2 ); usleep(800000); // 组装压实后，松开爪子
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);

            //拼ID=2物体
            g_arm.moveSmooth(0, px -0.0f, py + 7.5f , 5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px -0.0f, py + 6.5f , 4.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px -0.0f, py + 6.5f , 2.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000); // 组装压实后，松开爪子

            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);// 抬升并回归待命姿态

        }).detach();
    }

    //arm2是摄像头云台
    void executeDemo220() {
        std::thread([]() {
            std::cout << "\n>>> [巡航搜索] 调整云台视角 1 (DEMO220: Pan 43, Tilt 45) <<<" << std::endl;
            g_arm.moveCameraSmooth(43.0f, 45.0f);
            usleep(2000000); // 预留 1 秒钟给云台平滑移动和摄像头画面稳定
            sendToMonitor("FIND_ACK_220\r\n"); // 动作完成，向上位机汇报！
        }).detach();
    }

    void executeDemo221() {
        std::thread([]() {
            std::cout << "\n>>> [巡航搜索] 调整云台视角 2 (DEMO221: Pan 43, Tilt 30) <<<" << std::endl;
            g_arm.moveCameraSmooth(43.0f, 20.0f);
            usleep(2000000); // 预留 1 秒钟
            sendToMonitor("FIND_ACK_221\r\n"); // 动作完成，向上位机汇报
        }).detach();
    }
}

// ============================================================================
// [5] 串口通信与指令路由网关 (Command Router)
// ============================================================================
class SerialRouter {
private:
    int fd_;
    std::string rx_buffer_;

    int initPort(const char* portname) {
        int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) return -1;
        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetospeed(&tty, SystemConfig::BAUD_RATE);
        cfsetispeed(&tty, SystemConfig::BAUD_RATE);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK; tty.c_lflag = 0; tty.c_oflag = 0;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
        tcsetattr(fd, TCSANOW, &tty);
        return fd;
    }

    void dispatchCommand(const std::string& cmd_str) {
        if (cmd_str.empty()) return;

        char cmd[16] = {0};
        float px=0, py=0, pz=0, zx=0, zy=0, zz=0, xx=0, xy=0, xz=0, claw=0;
        
        int num = sscanf(cmd_str.c_str(), "%15s %f %f %f %f %f %f %f %f %f %f",
                         cmd, &px, &py, &pz, &zx, &zy, &zz, &xx, &xy, &xz, &claw);

        std::string ack = "OK\r\n";

        for (int i = 0; cmd[i]; i++) {
            cmd[i] = toupper(cmd[i]);
        }

        // ==========================================
        // 云台专属动作指令
        // ==========================================
        if (strcmp(cmd, "ARM2") == 0) {
            std::cout << "\n>>> [Pilot] 接收云台平滑移动指令 (ARM2) | Pan=" << px << " Tilt=" << py << std::endl;
            // px 对应 CH7 (左右), py 对应 CH8 (俯仰)
            g_arm.moveCameraSmooth(px, py);
        }
        else if (strcmp(cmd, "CAM") == 0) {
            // Monitor 高频下发的指令，瞬间执行并打断任何由于 ARM2 正在进行的平滑移动
            g_arm.setCameraDirect(px, py);
        }

        // ==========================================
        // 原有机械臂与小车指令
        // ==========================================
        else if (strcmp(cmd, "DEMO") == 0) {
            DemoManager::runDemoSequence();
        }
        else if (strcmp(cmd, "DEMO000") == 0) {
            std::cout << "[Pilot] DEMO000" << std::endl;
            DemoManager::executeDemo000(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO001") == 0) {
            std::cout << "[Pilot] DEMO001" << std::endl;
            DemoManager::executeDemo001(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO111") == 0) {
            std::cout << "[Pilot] DEMO111" << std::endl;
            DemoManager::executeDemo111(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO112") == 0) {
            std::cout << "[Pilot] DEMO112" << std::endl;
            DemoManager::executeDemo112(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO021") == 0) {
            std::cout << "[Pilot] DEMO021" << std::endl;
            DemoManager::executeDemo021(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO031") == 0) {
            std::cout << "[Pilot] DEMO021" << std::endl;
            DemoManager::executeDemo021(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO131") == 0) {
            std::cout << "[Pilot] DEMO131" << std::endl;
            DemoManager::executeDemo131(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO132") == 0) {
            std::cout << "[Pilot] DEMO132" << std::endl;
            DemoManager::executeDemo132(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO121") == 0) {
            std::cout << "[Pilot] DEMO131" << std::endl;
            DemoManager::executeDemo131(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO122") == 0) {
            std::cout << "[Pilot] DEMO132" << std::endl;
            DemoManager::executeDemo132(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO041") == 0) {
            std::cout << "[Pilot] DEMO041" << std::endl;
            DemoManager::executeDemo041(px, py, pz);
        }
        else if (strcmp(cmd, "DO001") == 0) {
            std::cout << "[Pilot] DO001" << std::endl;
            DemoManager::executeDo001();
        }
        else if (strcmp(cmd, "DO031") == 0) {
            std::cout << "[Pilot] DO031" << std::endl;
            DemoManager::executeDo031();
        }
        else if (strcmp(cmd, "DO002") == 0) {
            std::cout << "[Pilot] DO002" << std::endl;
            DemoManager::executeDo002();
        }
        else if (strcmp(cmd, "DEMO001") == 0) {
            std::cout << "[Pilot] DEMO001" << std::endl;
            DemoManager::executeDemo001(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO102") == 0) {
            std::cout << "[Pilot] DEMO102" << std::endl;
            DemoManager::executeDemo102(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO002") == 0) {
            std::cout << "[Pilot] DEMO002" << std::endl;
            DemoManager::executeDemo002(px, py, pz);
        }
        else if (strcmp(cmd, "DEMO091") == 0) {
            std::cout << "[Pilot] DEMO091" << std::endl;
            DemoManager::executeDemo091(px, py, pz);
        }
        else if (strcmp(cmd, "CHASSIS_MOVE") == 0) {
            std::cout << "\n>>> [Pilot] 收到find移动指令 CHASSIS_MOVE" << std::endl;
            DemoManager::executeChassisAutoMove(px, py);
        }
        else if (num == 11 && (strcmp(cmd, "ARM0") == 0 || strcmp(cmd, "ARM1") == 0)) {
            int arm_id = (strcmp(cmd, "ARM0") == 0) ? 0 : 1;
            std::cout << "\n>>> [Pilot] 接收单步: " << cmd << " | X=" << px << " Y=" << py << " Z=" << pz << std::endl;
            g_arm.moveSmooth(arm_id, px, py, pz, zx, zy, zz, xx, xy, xz);
        } 
        else if (num >= 3 && strcmp(cmd, "CH") == 0) {
            int channel = (int)px;  
            if (channel >= 0 && channel <= 15) {
                int target_arm = (channel >= 7) ? 1 : 0;
                float calibrated_angle = py;
                
                if (target_arm == 0) {
                    if      (channel == 0) calibrated_angle = py + 20.0f; 
                    else if (channel == 1) calibrated_angle = py + 30.0f; 
                    else if (channel == 2) calibrated_angle = py + 20.0f; 
                    else if (channel == 3) calibrated_angle = py + 20.0f; 
                    else if (channel == 4) calibrated_angle = py + 12.0f; 
                    else if (channel == 5) calibrated_angle = py + 13.0f; 
                } 
                else if (target_arm == 1) {
                    if      (channel == 9) calibrated_angle = py + 18.0f; 
                    else if (channel == 10) calibrated_angle = py + 18.0f; 
                    else if (channel == 11) calibrated_angle = py + 18.0f; 
                    else if (channel == 12) calibrated_angle = py + 18.0f; 
                    else if (channel == 13) calibrated_angle = py + 18.0f; 
                    else if (channel == 14) calibrated_angle = py - 55.0f; 
                    
                    // ==========================================
                    // 摄像头云台舵机单步强制控制
                    // ==========================================
                    else if (channel == 7) {
                        calibrated_angle = py; 
                        g_arm.notifyCameraManualSet(7, calibrated_angle); // 同步状态给虚拟 ARM2
                    }
                    else if (channel == 8) {
                        calibrated_angle = py;
                        g_arm.notifyCameraManualSet(8, calibrated_angle); // 同步状态给虚拟 ARM2
                    }
                }
                g_arm.setServoAngle(target_arm, channel, calibrated_angle);
                
                std::cout << "[Pilot] 收到直控指令 -> " << cmd << " " << channel << " 角度:" << py 
                          << " (已映射为:" << calibrated_angle << ")" << std::endl;
            }
        }
        else if (strcmp(cmd, "MR") == 0) {
            g_car.resetPosition();
            std::cout << "[Pilot] 底盘里程计已归零。" << std::endl;
        }
        else if (strcmp(cmd, "0") == 0) {
            g_car.emergencyStop();
            std::cout << "[Pilot] 底盘急停锁死！" << std::endl;
        }
        else if (strcmp(cmd, "M") == 0) {
            g_car.setAbsoluteTarget(px, py);
            std::cout << "[Pilot] 绝对移动目标 -> X:" << px << "cm, Y:" << py << "cm" << std::endl;
        }
        else if (strcmp(cmd, "MW") == 0) {
            g_car.moveRelative(px, 0); 
            std::cout << "[Pilot] 相对移动 -> 前进 " << px << " cm" << std::endl;
        }
        else if (strcmp(cmd, "MS") == 0) {
            g_car.moveRelative(-px, 0); 
            std::cout << "[Pilot] 相对移动 -> 后退 " << px << " cm" << std::endl;
        }
        else if (strcmp(cmd, "MD") == 0) {
            g_car.moveRelative(0, px); 
            std::cout << "[Pilot] 相对移动 -> 向右平移 " << px << " cm" << std::endl;
        }
        else if (strcmp(cmd, "MA") == 0) {
            g_car.moveRelative(0, -px); 
            std::cout << "[Pilot] 相对移动 -> 向左平移 " << px << " cm" << std::endl;
        }
        else if (strcmp(cmd, "MQ") == 0) {
            float deg = (num >= 2) ? px : 90.0f;
            g_car.turnRelative(-deg); 
            std::cout << "[Pilot] 车身原地左转 " << deg << " 度" << std::endl;
        }
        else if (strcmp(cmd, "ME") == 0) {
            float deg = (num >= 2) ? px : 90.0f;
            g_car.turnRelative(deg);  
            std::cout << "[Pilot] 车身原地右转 " << deg << " 度" << std::endl;
        }
        else if (strcmp(cmd, "TEST") == 0) {
            std::cout << "[Pilot] 收到 TEST 指令，准备执行物理层盲走验证..." << std::endl;
            g_car.blindTest();
        }
        else {
            std::cout << "[Pilot 警告] 收到未知或格式错误的指令: [" << cmd_str << "] 解析出CMD:[" << cmd << "]" << std::endl;
        }

        write(fd_, ack.c_str(), ack.length());
        std::cout << std::flush; 
    }

public:
    SerialRouter() : fd_(-1) {}
    
    bool start() {
        fd_ = initPort(SystemConfig::SERIAL_PORT_MONITOR);
        g_monitor_fd = fd_; 
        return fd_ >= 0;
    }

    void spinOnce() {
        char buffer[256];
        int valread = read(fd_, buffer, sizeof(buffer) - 1);
        if (valread > 0) {
            buffer[valread] = '\0';
            rx_buffer_ += buffer;
            
            size_t pos;
            while ((pos = rx_buffer_.find_first_of("\r\n")) != std::string::npos) {
                std::string line = rx_buffer_.substr(0, pos);
                rx_buffer_.erase(0, pos + 1);
                dispatchCommand(line);
            }
        } else {
            usleep(10000); 
        }
    }
};

// ============================================================================
// [6] 主程序入口
// ============================================================================
int main() {
    std::cout << "[Pilot] 启动运动控制核心下位机 " << std::endl;

    g_arm.init();
    
    if (!g_car.init()) {
        std::cerr << "[Pilot] 警告：无法连接 STM32 底盘串口，小车功能已禁用。" << std::endl;
    }

    SerialRouter router;
    if (!router.start()) {
        std::cerr << "[Pilot] 致命错误：串口打开失败: " << SystemConfig::SERIAL_PORT_MONITOR  << std::endl;
        return -1;
    }
    std::cout << "[Pilot] 串口指令网关启动 (115200)。\n[Pilot] 等待上位机 (Monitor) 接入指令...\n" << std::endl;

    while (true) {
        router.spinOnce();
    }

    return 0;
}