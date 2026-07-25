/**
 * @file global_state.cpp
 * @brief 全局变量定义与初始化
 */
#include "global_state.h"

cv::VideoCapture *g_cap_ptr = nullptr;
int g_serial_fd = -1;

std::mutex g_task_mtx;
DemoTask g_demo_task;

ClosedLoopState g_cl_state;

std::atomic<bool> g_trigger_aruco_fix{false};
cv::Point2f g_fixed_aruco_center(-1.0f, -1.0f);

std::atomic<bool> g_auto_cam_running{false};
float g_cam_pan = 113.0f;
float g_cam_tilt = 50.0f;

std::atomic<bool> g_wf_chassis_done{false};
std::atomic<bool> g_wf_find_failed{false};
float g_global_x_offset_cm = 0.0f;