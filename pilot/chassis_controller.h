/**
 * @file chassis_controller.h
 * @brief 麦轮底盘闭环控制
 */
#pragma once
#include <string>
#include <thread>
#include <atomic>

class ChassisController {
private:
    int fd_;
    std::string rx_buffer_;
    std::thread pid_thread_;
    std::thread rx_thread_;
    bool running_;

    const float Lx = 210.0f;
    const float Ly = 135.0f;
    const float K = Lx + Ly;
    const float MM_PER_PULSE = 0.1953f * 1.10f;

    std::atomic<float> curr_x_{0}, curr_y_{0}, curr_yaw_{0};
    std::atomic<float> target_x_{0}, target_y_{0}, target_yaw_{0};

    // 新增：全局导航坐标记录仪
    float nav_x_{0.0f};
    float nav_y_{0.0f};
    float nav_yaw_{0.0f};

    long last_m1 = 0, last_m2 = 0, last_m3 = 0, last_m4 = 0;
    bool first_encoder = true;
    std::atomic<bool> is_testing_{false};

    std::atomic<bool> vel_mode_{false};
    std::atomic<float> vel_vx_{0}, vel_vy_{0}, vel_vw_{0};

    int initPort(const char *portname);
    void sendCmd(const std::string &cmd);
    void parseEncoder(const std::string &msg);
    void pidLoop();

public:
    ChassisController();
    bool init();
    void blindTest();
    void resetPosition();
    void emergencyStop();
    void setAbsoluteTarget(float x_cm, float y_cm);
    void moveRelative(float dx_cm, float dy_cm);
    void turnRelative(float deg);
    void setVelocity(float vx, float vy, float vw);
    void stopVelocity();

    // 新增：路径规划核心方法
    void planPath(float tx, float ty, float tyaw);
    void applyNavAdjustment(float real_fwd, float real_right);
    void executeAlignManeuver(float dx, float dy, float dyaw);
};

extern ChassisController g_car;