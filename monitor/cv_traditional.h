#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// 传统视觉辅助算法提取
bool findOrderedCorners(const cv::Mat &roi_frame, int class_id, std::vector<cv::Point2f> &ordered_corners, cv::Mat &out_mask);
bool findWallCorners(const cv::Mat &roi_frame, std::vector<cv::Point2f> &ordered_corners, cv::Mat &out_mask, int class_id);