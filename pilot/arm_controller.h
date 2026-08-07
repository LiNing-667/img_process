/**
 * @file arm_controller.h
 * @brief 机械臂控制中枢
 */
#pragma once
#include <vector>
#include <atomic>
#include <stdint.h>

extern float g_arm_spatial_speed; // 空间直线移动速度 (cm/s)
extern float g_arm_angle_speed;// 姿态旋转角速度 (deg/s)

class RoboticArmController {
private:
    struct ArmState {
        bool initialized = false;
        float p[3]; float z[3]; float x[3];
        float current_angles[6];
        // 【新增】：记录末端上一次的三维坐标
        float last_px = 0.0f; 
        float last_py = 0.0f; 
        float last_pz = 0.0f; 
    };
    int i2c_addr_;
    ArmState states_[2];
    std::atomic<int> cmd_version_{0};

    std::atomic<float> curr_pan_{40.0f};  
    std::atomic<float> curr_tilt_{43.0f}; 
    std::atomic<int> cam_cmd_version_{0};

    void normalizeVec(float v[3]);

public:
    RoboticArmController(int addr);
    bool init();
    void setServoAngle(int arm_id, uint8_t channel, float angle);
    void setChannelDirect(int channel, float angle);   // 单通道直控（含 CH 标定补偿 / 云台状态同步）
    void setJointsDirect(int arm_id, const std::vector<float> &angles);
    void moveSmooth(int arm_id, float t_px, float t_py, float t_pz, float t_zx, float t_zy, float t_zz, float t_xx, float t_xy, float t_xz);
    
    void moveRawChannelsSmooth(int arm_id, const std::vector<float> &target_raw_angles, float time_sec = 2.0f);
    void moveCameraSmooth(float target_pan, float target_tilt);
    void setCameraDirect(float target_pan, float target_tilt);
    void notifyCameraManualSet(int channel, float angle);
};

extern RoboticArmController g_arm;