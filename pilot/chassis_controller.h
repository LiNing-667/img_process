/**
 * @file chassis_controller.h
 * @brief 麦轮底盘闭环控制
 */
#pragma once
#include <string>
#include <thread>
#include <atomic>

// ==============================================================
// 底盘动力与物理标定全局配置
// 集中管理所有和重量、摩擦力、惯性相关的“魔数”，方便换配重后统一调参
// ==============================================================
struct ChassisDynamicsConfig
{
    // 1. 底层 PID 动力参数限制
    float PID_MIN_POWER = 319.0f; // 克服静摩擦的最小 PWM/速度 (起步死区)
    float PID_MAX_POWER = 450.0f; // 输出上限
    float RAMP_STEP_X = 15.0f;    // 前后加速度斜率
    float RAMP_STEP_Y = 8.0f;     // 左右加速度斜率 (防侧滑)
    float RAMP_STEP_YAW = 15.0f;  // 旋转加速度斜率

    // 2. 开环基础动力指令值 (对应发送给下位机的速度设定)
    float CMD_SPEED_BASE = 320.0f; // 主要平移动力 (Align和PlanPath使用)
    float CMD_KICK_SPEED = 320.0f; // 短距离微动补偿动力 (moveRelative使用)

    // 3. 物理速度映射标定 (时间估算核心！)
    // 公式：行驶时间 = 距离 / (对应的_CM_PER_SEC)
    float CM_PER_SEC_FWD = 25.0f;   // 在CMD_SPEED_BASE下，前后每秒跑几厘米
    float CM_PER_SEC_LAT = 20.0f;   // 在CMD_SPEED_BASE下，左右每秒跑几厘米
    float DEG_PER_SEC_TURN = 30.0f; // 旋转每秒估算度数
    float KICK_CM_PER_SEC = 25.0f;  // 在CMD_KICK_SPEED下，微动每秒跑几厘米

    // PlanPath 中单独调用的移动/旋转指令速度参数
    float PLAN_MOVE_SPEED = 25.0f;
    float PLAN_TURN_SPEED = 45.0f;

    // 4. 起步/刹车物理延时补偿 (秒/毫秒)
    float T_COMP_STARTUP_SEC = 0.5f;       // 标准起步防滑延时 (0.5秒)
    float T_COMP_SHORT_STARTUP_SEC = 0.1f; // 极短程微调延时 (0.1秒)
    int WAIT_MS_OFFSET = 500;              // 各种动作后默认的停顿缓冲 (500毫秒)

    // 横向位移削弱系数 (AlignManeuver 中的避让比例)
    float LAT_DIAG_RATIO = 0.4f;
};

// 实例化为全局配置
extern ChassisDynamicsConfig g_dynamics;

class ChassisController
{
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

    // 路径规划
    void planPath(float tx, float ty, float tyaw);
    void applyNavAdjustment(float real_fwd, float real_right);
    void executeAlignManeuver(float dx, float dy, float dyaw);

    // 全局位置估计（命令式，不依赖底层里程计）：
    // 相对移动指令(MW/MS/MD/MA/MQ/ME)同步累加 nav_* 全局导航坐标，
    // 供 MW 移动完成后上报上位机实时显示。
    void noteForwardMove(float fwd_cm); // 前进(+) / 后退(-)
    void noteRightMove(float right_cm); // 右移(+) / 左移(-)
    void noteTurn(float deg);           // 右转(+) / 左转(-)
    void reportPosition();              // 上报当前坐标 "POS x y yaw"
};

extern ChassisController g_car;