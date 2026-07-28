/**
 * @file vision_engine.h
 * @brief 核心业务状态机与任务管线
 */
#pragma once
#include "types.h"
#include "pilot_comm.h"
#include "geometry_utils.h"
#include <opencv2/opencv.hpp>

namespace TaskManager {
    DemoTask fetchTask();
}

class VisionEngine {
private:
    PilotCommunicator &pilot_comm;
    HandEyeCalibrator calibrator;
    YoloResult current_yolo_res;
    bool has_detection = false;

    // 内部私有处理流
    void handleClosedLoopCheck(const DemoTask &current_task, cv::Mat &raw_frame);
    void handleSingleAxisServo(const DemoTask &current_task, cv::Mat &raw_frame);
    bool handleBlindOperations(const DemoTask &current_task);
    bool handleYoloAndPnP(const DemoTask &current_task, cv::Mat &raw_frame);
    void handleHsvFindOneshot(const DemoTask &task, cv::Mat &raw_frame);
    void handleCheck091(cv::Mat &raw_frame);

public:
    VisionEngine(PilotCommunicator &comm);

    // 公开的周期调用接口
    void processTask(const DemoTask &task, cv::Mat &raw_frame);
    void processAutoCamera(cv::Mat &raw_frame);
    void processArucoFix(cv::Mat &raw_frame);
    void renderOsd(cv::Mat &raw_frame);
};