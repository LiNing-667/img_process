/**
 * @file global_state.h
 * @brief 全局变量声明 (Extern)
 */
#pragma once

#include "types.h"
#include <atomic>
#include <mutex>

extern cv::VideoCapture *g_cap_ptr;
extern int g_serial_fd;

extern std::mutex g_task_mtx;
extern DemoTask g_demo_task;

extern ClosedLoopState g_cl_state;

extern std::atomic<bool> g_trigger_aruco_fix;
extern cv::Point2f g_fixed_aruco_center;

extern std::atomic<bool> g_auto_cam_running;
extern float g_cam_pan;
extern float g_cam_tilt;

extern std::atomic<bool> g_wf_chassis_done;
extern std::atomic<bool> g_wf_find_failed;
extern float g_arm_x_offset_cm[2];// 拆分为数组：0对应ARM0(右臂)，1对应ARM1(左臂)

// --- 新增：HSV 视觉伺服与云台记忆 ---
extern float g_calibrated_pan;
extern float g_calibrated_tilt;
extern std::atomic<bool> g_hsv_find_running;

// --- DEMO091 闭环验证缓存 ---
extern cv::Rect g_cache_091_bbox;
extern float g_cache_091_px;
extern float g_cache_091_py;
extern float g_cache_091_pz;

// --- DEMO001 和 DEMO002 闭环验证缓存 ---
extern cv::Point2f g_cache_pt1; // 锚点：1号点
extern float g_cache_001_px, g_cache_001_py, g_cache_001_pz;
extern float g_cache_002_px, g_cache_002_py, g_cache_002_pz;