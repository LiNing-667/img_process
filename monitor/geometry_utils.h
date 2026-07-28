/**
 * @file geometry_utils.h
 * @brief 数学矩阵与手眼标定辅助算法
 */
#pragma once

#include "types.h"
#include <opencv2/opencv.hpp>
#include <vector>

// 相机内参声明
extern const cv::Mat CAMERA_MATRIX;
extern const cv::Mat DIST_COEFFS;

// 几何工具函数
cv::Point2f getBasePoint(int index, const std::vector<cv::Point2f> &corners);
std::vector<cv::Point3f> get3DModelPoints(int class_id);
std::vector<cv::Point2f> clusterPoints(const std::vector<cv::Point2f> &raw_pts, float dist_thresh = 10.0f);

// 3D手眼标定转换器
class HandEyeCalibrator
{
private:
    double tx_[2] = {-40.00, -52.60}; // {-2.40, -2.60}; 偏前就减
    double ty_[2] = {78.0, -120.0}; // {91.0, -91.0}; 偏外就放大
    double tz_[2] = {205.0, 205.0};
    double rx_[2] = {0.0, 0.0};
    double ry_[2] = {0.0, 0.0};
    double rz_[2] = {0.0, 0.0};

    cv::Mat getTransformationMatrix(int arm_id);

public:
    Pose6D transform(const cv::Mat &rvec_cam, const cv::Mat &tvec_cam, int arm_id);
};