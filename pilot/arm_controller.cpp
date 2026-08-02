/**
 * @file arm_controller.cpp
 * @brief 机械臂控制中枢
 */
#include "arm_controller.h"
#include "hw_i2c.h"
#include "pilot_config.h"
#include "ik_solver.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <thread>
#include <algorithm>

void RoboticArmController::normalizeVec(float v[3]) {
    float len = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (len > 1e-6f) { v[0]/=len; v[1]/=len; v[2]/=len; }
}

RoboticArmController::RoboticArmController(int addr) : i2c_addr_(addr) {}

bool RoboticArmController::init() {
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
    initBoard(SystemConfig::PCA_ADDR_ARM0);
    initBoard(SystemConfig::PCA_ADDR_ARM1);
    std::cout << "[Arm] 双臂 PCA9685 驱动模块已就绪" << std::endl;
    return true;
}

void RoboticArmController::setServoAngle(int arm_id, uint8_t channel, float angle) {
    int addr = (arm_id == 0) ? SystemConfig::PCA_ADDR_ARM0 : SystemConfig::PCA_ADDR_ARM1;
    if (angle < 0) angle = 0; if (angle > 250) angle = 250;
    uint16_t off = 102 + (uint16_t)((angle / 180.0f) * (512 - 102));
    uint8_t reg_base = 0x06 + 4 * channel;
    g_i2c_arm.writeReg(addr, reg_base, 0);
    g_i2c_arm.writeReg(addr, reg_base + 1, 0);
    g_i2c_arm.writeReg(addr, reg_base + 2, off & 0xFF);
    g_i2c_arm.writeReg(addr, reg_base + 3, off >> 8);
}

void RoboticArmController::setJointsDirect(int arm_id, const std::vector<float> &angles) {
    int ch_offset = (arm_id == 0) ? 0 : 9;
    if (arm_id == 0) {
        setServoAngle(arm_id, ch_offset + 0, angles[0] + 95.0f);
        setServoAngle(arm_id, ch_offset + 1, angles[1] + 120.0f);
        setServoAngle(arm_id, ch_offset + 2, angles[2] + 110.0f);
        setServoAngle(arm_id, ch_offset + 3, -angles[3] + 110.0f);
        setServoAngle(arm_id, ch_offset + 4, (180.0f - angles[4]) + 12.0f);
        setServoAngle(arm_id, ch_offset + 5, angles[5] + 102.0f);
    } else {
        setServoAngle(arm_id, ch_offset + 0, angles[0] + 108.0f);
        setServoAngle(arm_id, ch_offset + 1, angles[1] + 108.0f);
        setServoAngle(arm_id, ch_offset + 2, angles[2] + 108.0f);
        setServoAngle(arm_id, ch_offset + 3, -angles[3] + 108.0f);
        setServoAngle(arm_id, ch_offset + 4, (180.0f - angles[4]) + 18.0f);
        setServoAngle(arm_id, ch_offset + 5, angles[5] + 51.0f);
    }
}

void RoboticArmController::moveSmooth(int arm_id, float t_px, float t_py, float t_pz, float t_zx, float t_zy, float t_zz, float t_xx, float t_xy, float t_xz) {
    int my_version = ++cmd_version_;
    float target_angles[6];
    float t_z_axis[3] = {t_zx, t_zy, t_zz};
    float t_x_axis[3] = {t_xx, t_xy, t_xz};

    int status = ik6_pose(t_px, t_py, t_pz, t_z_axis, t_x_axis, target_angles);
    if (status != 0) { std::cerr << "[Arm" << arm_id << "] 警告：目标超出物理极限！" << std::endl; return; }

    ArmState &curr = states_[arm_id];
    std::vector<float> tgt(target_angles, target_angles + 6);

    if (!curr.initialized) {
        setJointsDirect(arm_id, tgt);
        for (int i=0; i<6; i++) curr.current_angles[i] = tgt[i];
        curr.initialized = true; return;
    }
    std::vector<float> start(curr.current_angles, curr.current_angles + 6);
    for (int i=0; i<6; i++) curr.current_angles[i] = tgt[i];

    std::thread([this, arm_id, start, tgt, my_version]() {
        float max_angle_diff = 0.0f; 
        for(int i=0; i<6; i++) max_angle_diff = std::max(max_angle_diff, std::abs(tgt[i] - start[i]));
        float speed_deg_per_sec = 60.0f; float loop_delay_sec = 0.02f;    
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

void RoboticArmController::moveRawChannelsSmooth(int arm_id, const std::vector<float> &target_raw_angles, float time_sec) {
    int my_version = ++cmd_version_;
    ArmState &curr = states_[arm_id];
    std::vector<float> start_raw(6, 0.0f);
    
    if (curr.initialized) {
        if (arm_id == 0) {
            start_raw[0] = curr.current_angles[0] + 95.0f;
            start_raw[1] = curr.current_angles[1] + 120.0f;
            start_raw[2] = curr.current_angles[2] + 110.0f;
            start_raw[3] = -curr.current_angles[3] + 110.0f;
            start_raw[4] = (180.0f - curr.current_angles[4]) + 12.0f;
            start_raw[5] = curr.current_angles[5] + 102.0f; // 99
        } else {
            start_raw[0] = curr.current_angles[0] + 108.0f;
            start_raw[1] = curr.current_angles[1] + 108.0f;
            start_raw[2] = curr.current_angles[2] + 108.0f;
            start_raw[3] = -curr.current_angles[3] + 108.0f;
            start_raw[4] = (180.0f - curr.current_angles[4]) + 18.0f;
            start_raw[5] = curr.current_angles[5] + 51.0f;
        }
    } else { start_raw = target_raw_angles; }

    if (arm_id == 0) {
        curr.current_angles[0] = target_raw_angles[0] - 95.0f;
        curr.current_angles[1] = (target_raw_angles[1] - 120.0f);
        curr.current_angles[2] = target_raw_angles[2] - 110.0f;
        curr.current_angles[3] = -(target_raw_angles[3] - 110.0f);
        curr.current_angles[4] = 180.0f - (target_raw_angles[4] - 12.0f);
        curr.current_angles[5] = target_raw_angles[5] - 102.0f;
    } else {
        curr.current_angles[0] = target_raw_angles[0] - 108.0f;
        curr.current_angles[1] = (target_raw_angles[1] - 108.0f);
        curr.current_angles[2] = target_raw_angles[2] - 108.0f;
        curr.current_angles[3] = -(target_raw_angles[3] - 108.0f);
        curr.current_angles[4] = 180.0f - (target_raw_angles[4] - 18.0f);
        curr.current_angles[5] = target_raw_angles[5] - 51.0f;
    }
    curr.initialized = true;

    std::thread([this, arm_id, start_raw, target_raw_angles, my_version, time_sec]() {
        int steps = std::max(1, (int)(time_sec / 0.02f));
        int ch_offset = (arm_id == 0) ? 0 : 9;
        for (int i = 1; i <= steps; ++i) {
            if (cmd_version_ != my_version) return;
            float ratio = (float)i / steps;
            for (int j = 0; j < 6; j++) {
                float cur_angle = start_raw[j] + (target_raw_angles[j] - start_raw[j]) * ratio;
                setServoAngle(arm_id, ch_offset + j, cur_angle);
            }
            usleep(20000); 
        } 
    }).detach();
}

void RoboticArmController::moveCameraSmooth(float target_pan, float target_tilt) {
    int my_version = ++cam_cmd_version_;
    float start_pan = curr_pan_.load();
    float start_tilt = curr_tilt_.load();
    std::thread([this, start_pan, start_tilt, target_pan, target_tilt, my_version]() {
        float diff_pan = target_pan - start_pan;
        float diff_tilt = target_tilt - start_tilt;
        float max_diff = std::max(std::abs(diff_pan), std::abs(diff_tilt));
        int steps = std::max(1, (int)(max_diff / (60.0f * 0.02f)));

        for (int i = 1; i <= steps; ++i) {
            if (cam_cmd_version_ != my_version) return; 
            float ratio = (float)i / steps;
            float cur_p = start_pan + diff_pan * ratio;
            float cur_t = start_tilt + diff_tilt * ratio;
            setServoAngle(1, 7, cur_p);
            setServoAngle(1, 8, cur_t);
            curr_pan_.store(cur_p);
            curr_tilt_.store(cur_t);
            usleep(20000); 
        } 
    }).detach();
}

void RoboticArmController::setCameraDirect(float target_pan, float target_tilt) {
    cam_cmd_version_++;
    setServoAngle(1, 7, target_pan);
    setServoAngle(1, 8, target_tilt);
    curr_pan_.store(target_pan);
    curr_tilt_.store(target_tilt);
}

void RoboticArmController::notifyCameraManualSet(int channel, float angle) {
    cam_cmd_version_++; 
    if (channel == 7) curr_pan_.store(angle);
    if (channel == 8) curr_tilt_.store(angle);
}

RoboticArmController g_arm(SystemConfig::PCA_ADDR_ARM0);