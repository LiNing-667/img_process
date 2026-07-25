/**
 * @file types.h
 * @brief 系统核心数据结构定义
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

// 宏指令任务结构
struct DemoTask
{
    bool pending = false;
    int arm_id = -1;
    int class_id = -1;
    int action_id = -1;
    std::string raw_cmd = "";
};

// 6D 空间位姿
struct Pose6D
{
    float x, y, z;
    float rx, ry, rz;
};

// 视觉闭环复核状态
struct ClosedLoopState
{
    std::vector<cv::Point2f> base_corners_2d;
    Pose6D last_pose;
    int retry_count = 0;
    cv::Mat last_rvec;
    cv::Mat last_tvec;
    cv::Point2f last_obj_center;
};

// 目标物体元数据
struct ObjectMeta
{
    cv::Rect bbox;
    cv::Point2f center;
    int class_id;
    float confidence;
    bool has_refined_center;
    cv::Point2f refined_center;
    cv::Mat roi_mask;
    cv::Mat ai_mask;
    std::vector<cv::Point2f> corners_2d;
    double tx, ty, tz;
    double rx, ry, rz;
    std::vector<cv::Point2f> sub_centers;
};

// YOLO 整体检测结果
struct YoloResult
{
    bool detected;
    std::vector<ObjectMeta> objects;
};

// 跨线程图像共享帧
struct SharedFrame
{
    cv::Mat frame;
    std::mutex mtx;
    std::condition_variable cv;
    bool ready = false;
};

// 闭环过渡动作图
struct CLTransition
{
    int target_id;
    std::vector<int> required_points;
    std::string retry_cmd;
    std::string success_cmd;
};