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
extern float g_global_x_offset_cm;