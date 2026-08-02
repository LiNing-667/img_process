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
std::atomic<bool> g_wf_demo_done{false};
std::atomic<bool> g_wf_cmd_done{false};
std::atomic<bool> g_wf_find_failed{false};
float g_arm_x_offset_cm[2] = {0.0f, 0.0f};

float g_calibrated_pan = -1.0f;
float g_calibrated_tilt = -1.0f;
std::atomic<bool> g_hsv_find_running{false};

cv::Rect g_cache_091_bbox(0, 0, 0, 0);
float g_cache_091_px = 0.0f;
float g_cache_091_py = 0.0f;
float g_cache_091_pz = 0.0f;

cv::Point2f g_cache_pt1(-1.0f, -1.0f);
float g_cache_001_px = 0.0f, g_cache_001_py = 0.0f, g_cache_001_pz = 0.0f;
float g_cache_002_px = 0.0f, g_cache_002_py = 0.0f, g_cache_002_pz = 0.0f;

cv::Vec4i g_align91_ref_line(0, 0, 0, 0);
bool g_align91_ref_line_valid = false;