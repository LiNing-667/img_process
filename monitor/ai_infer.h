#pragma once
#include "types.h"
#include <opencv2/opencv.hpp>
#include <vector>

// 暴露 AI 推理接口
YoloResult runYoloInference(const cv::Mat &frame, int target_class_id);
std::vector<cv::Point2f> runNextYoloInferenceRaw(const cv::Mat &roi_frame);