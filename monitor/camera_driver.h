#pragma once
#include "types.h"
#include <opencv2/opencv.hpp>

namespace CameraManager {
    cv::VideoCapture probeAndInit();
    void startCaptureThread(cv::VideoCapture &cap, SharedFrame &shared);
    bool getLatestFrame(SharedFrame &shared, cv::Mat &frame, int timeout_ms);
}