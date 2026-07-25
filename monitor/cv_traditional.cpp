/**
 * @file cv_traditional.cpp
 * @brief 传统 OpenCV 图像处理与极值特征提取
 */
#include "cv_traditional.h"
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;

bool findOrderedCorners(const Mat &roi_frame, int class_id, std::vector<Point2f> &ordered_corners, Mat &out_mask)
{
    if (roi_frame.empty() || roi_frame.cols < 15 || roi_frame.rows < 15)
        return false;
    Mat gray, blurred;
    cvtColor(roi_frame, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blurred, Size(9, 9), 0);
    Mat sample_patch;
    if (class_id == 3)
    {
        int p_size = 20;
        Point2f pt_left(roi_frame.cols * 0.25f, roi_frame.rows / 2.0f);
        Point2f pt_right(roi_frame.cols * 0.75f, roi_frame.rows / 2.0f);
        Point2f pt_bottom(roi_frame.cols / 2.0f, roi_frame.rows * 0.75f);
        Rect r_left(pt_left.x - p_size / 2, pt_left.y - p_size / 2, p_size, p_size);
        Rect r_right(pt_right.x - p_size / 2, pt_right.y - p_size / 2, p_size, p_size);
        Rect r_bottom(pt_bottom.x - p_size / 2, pt_bottom.y - p_size / 2, p_size, p_size);
        r_left &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
        r_right &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
        r_bottom &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
        std::vector<Mat> patches;
        if (r_left.area() > 0) patches.push_back(blurred(r_left));
        if (r_right.area() > 0) patches.push_back(blurred(r_right));
        if (r_bottom.area() > 0) patches.push_back(blurred(r_bottom));
        if (patches.empty()) return false;
        cv::vconcat(patches, sample_patch);
    }
    else
    {
        Point2f sample_pt;
        int p_size = 30;
        if (class_id == 4)
        {
            sample_pt = Point2f(roi_frame.cols / 2.0f, roi_frame.rows * 0.3f);
            p_size = 20;
        }
        else
        {
            sample_pt = Point2f(roi_frame.cols / 2.0f, roi_frame.rows / 2.0f);
        }
        Rect center_patch_rect(sample_pt.x - p_size / 2, sample_pt.y - p_size / 2, p_size, p_size);
        center_patch_rect &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
        if (center_patch_rect.area() <= 0) return false;
        sample_patch = blurred(center_patch_rect);
    }
    Scalar mean_val, stddev_val;
    meanStdDev(sample_patch, mean_val, stddev_val);
    double tolerance = max(stddev_val[0] * 3.0, 30.0);
    Mat mask;
    inRange(blurred, mean_val[0] - tolerance, mean_val[0] + tolerance, mask);
    Mat open_kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    Mat close_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, open_kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, close_kernel);
    mask.copyTo(out_mask);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    double max_area = 0;
    vector<Point> best_contour;
    for (const auto &c : contours)
    {
        double area = contourArea(c);
        if (area > max_area)
        {
            max_area = area;
            best_contour = c;
        }
    }
    if (max_area < roi_frame.cols * roi_frame.rows * 0.05) return false;
    RotatedRect rect = minAreaRect(best_contour);
    Point2f rect_pts[4];
    rect.points(rect_pts);
    Point2f center = rect.center;

    std::vector<Point2f> corners;
    for (int i = 0; i < 4; i++)
    {
        Point2f v_dir = rect_pts[i] - center;
        double max_dot = -1e9;
        Point2f best_pt;
        for (const auto &cp : best_contour)
        {
            Point2f pt(cp.x, cp.y);
            Point2f v_pt = pt - center;
            double dot_prod = v_pt.x * v_dir.x + v_pt.y * v_dir.y;
            if (dot_prod > max_dot)
            {
                max_dot = dot_prod;
                best_pt = pt;
            }
        }
        corners.push_back(best_pt);
    }
    std::vector<Point2f> top, bot;
    std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b) { return a.y < b.y; });
    top.push_back(corners[0]);
    top.push_back(corners[1]);
    bot.push_back(corners[2]);
    bot.push_back(corners[3]);
    if (top[0].x > top[1].x) std::swap(top[0], top[1]);
    if (bot[0].x > bot[1].x) std::swap(bot[0], bot[1]);
    ordered_corners = {top[0], top[1], bot[1], bot[0]};
    return true;
}

bool findWallCorners(const Mat &roi_frame, std::vector<Point2f> &ordered_corners, Mat &out_mask, int class_id)
{
    if (roi_frame.empty() || roi_frame.cols < 15 || roi_frame.rows < 15)
        return false;

    Mat gray, blurred, edges;
    cvtColor(roi_frame, gray, COLOR_BGR2GRAY);

    // 如果是 ID=9 (demo091)，保留直方图均衡化以增强亮暗对比
    if (class_id == 9)
    {
        equalizeHist(gray, gray);
    }

    GaussianBlur(gray, blurred, Size(3, 3), 0);

    // ============================================================================
    // 【核心重构】：分离 ID 1~3 (如 demo131) 与 ID 9 (demo091) 的边缘提取逻辑
    // ============================================================================
    if (class_id >= 1 && class_id <= 3)
    {
        // --- [分支 A]：保持 demo131 等原有的“中心采样与双轨边缘融合”算法（含内部横线提取） ---
        Mat hsv, mask;
        Mat blurred_bgr;
        GaussianBlur(roi_frame, blurred_bgr, Size(3, 3), 0);
        cvtColor(blurred_bgr, hsv, COLOR_BGR2HSV);

        int p_size = 20;
        Point center_pt(hsv.cols / 2, hsv.rows / 2);
        Rect sample_rect(center_pt.x - p_size / 2, center_pt.y - p_size / 2, p_size, p_size);
        sample_rect &= Rect(0, 0, hsv.cols, hsv.rows);

        Scalar lower_bound, upper_bound;
        if (sample_rect.area() > 0)
        {
            Scalar mean_hsv = mean(hsv(sample_rect));
            float h_tol = 15.0f;
            float s_tol = 50.0f;
            float v_tol = 50.0f;
            lower_bound = Scalar(max(0.0, mean_hsv[0] - h_tol), max(40.0, mean_hsv[1] - s_tol), max(40.0, mean_hsv[2] - v_tol));
            upper_bound = Scalar(min(180.0, mean_hsv[0] + h_tol), min(255.0, mean_hsv[1] + s_tol), min(255.0, mean_hsv[2] + v_tol));
            inRange(hsv, lower_bound, upper_bound, mask);
        }
        else
        {
            inRange(hsv, Scalar(90, 100, 130), Scalar(124, 255, 255), mask);
        }

        Mat open_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 4));
        Mat close_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(mask, mask, MORPH_OPEN, open_kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, close_kernel);

        Mat outer_edges;
        Canny(mask, outer_edges, 30, 120);

        Mat raw_edges;
        Canny(blurred, raw_edges, 30, 100);

        Mat internal_zone;
        Mat erode_kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        erode(mask, internal_zone, erode_kernel);

        Mat internal_edges = Mat::zeros(blurred.size(), CV_8UC1);
        raw_edges.copyTo(internal_edges, internal_zone);

        edges = outer_edges | internal_edges; // 融合内部横线

        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(edges, edges, kernel);
    }
    else if (class_id == 9)
    {
        // --- [分支 B]：🚀 demo091 专属无噪点优化分支 ---
        Mat hsv, mask;
        Mat blurred_bgr;
        GaussianBlur(roi_frame, blurred_bgr, Size(3, 3), 0);
        cvtColor(blurred_bgr, hsv, COLOR_BGR2HSV);

        int p_size = 20;
        Point center_pt(hsv.cols / 2, hsv.rows / 2);
        Rect sample_rect(center_pt.x - p_size / 2, center_pt.y - p_size / 2, p_size, p_size);
        sample_rect &= Rect(0, 0, hsv.cols, hsv.rows);

        Scalar lower_bound, upper_bound;
        if (sample_rect.area() > 0)
        {
            Scalar mean_hsv = mean(hsv(sample_rect));
            float h_tol = 15.0f;
            float s_tol = 50.0f;
            float v_tol = 50.0f;
            lower_bound = Scalar(max(0.0, mean_hsv[0] - h_tol), max(40.0, mean_hsv[1] - s_tol), max(40.0, mean_hsv[2] - v_tol));
            upper_bound = Scalar(min(180.0, mean_hsv[0] + h_tol), min(255.0, mean_hsv[1] + s_tol), min(255.0, mean_hsv[2] + v_tol));
            inRange(hsv, lower_bound, upper_bound, mask);
        }
        else
        {
            inRange(hsv, Scalar(90, 100, 130), Scalar(124, 255, 255), mask);
        }

        Mat open_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 4));
        Mat close_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        morphologyEx(mask, mask, MORPH_OPEN, open_kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, close_kernel);

        // 🔥【关键改动】：只对纯净的Mask计算Canny，彻底切断原图内部杂色线条的引入
        Canny(mask, edges, 30, 120);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(edges, edges, kernel);
    }
    else
    {
        // --- [分支 C]：传统兜底分支 ---
        Canny(blurred, edges, 30, 100);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(edges, edges, kernel);
    }

    out_mask = edges.clone(); // 输出给调试窗口

    // ============================================================================
    // 以下后续处理流程（轮廓树遍历、找最大框、极值四角锁定）保持原样不变，确保工作流一致
    // ============================================================================
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return false;

    double max_size = 0;
    vector<Point> target_contour;

    for (size_t i = 0; i < contours.size(); i++)
    {
        bool has_inner_box = false;

        if ((class_id >= 1 && class_id <= 3) || class_id == 9)
        {
            int child_idx = hierarchy[i][2];
            while (child_idx != -1)
            {
                if (boundingRect(contours[child_idx]).area() > 50)
                {
                    has_inner_box = true;
                    break;
                }
                child_idx = hierarchy[child_idx][0];
            }
        }

        if (has_inner_box)
            continue;

        double size = boundingRect(contours[i]).area();
        if (size > max_size)
        {
            max_size = size;
            target_contour = contours[i];
        }
    }

    if (target_contour.empty() || max_size < roi_frame.cols * roi_frame.rows * 0.1)
        return false;

    Point2f tl = target_contour[0], tr = target_contour[0], br = target_contour[0], bl = target_contour[0];
    float min_x_plus_y = 1e9, max_x_minus_y = -1e9, max_x_plus_y = -1e9, min_x_minus_y = 1e9;

    for (auto p : target_contour)
    {
        float x = p.x;
        float y = p.y;
        if (x + y < min_x_plus_y)
        {
            min_x_plus_y = x + y;
            tl = p;
        }
        if (x - y > max_x_minus_y)
        {
            max_x_minus_y = x - y;
            tr = p;
        }
        if (x + y > max_x_plus_y)
        {
            max_x_plus_y = x + y;
            br = p;
        }
        if (x - y < min_x_minus_y)
        {
            min_x_minus_y = x - y;
            bl = p;
        }
    }

    ordered_corners = {tl, tr, br, bl};
    return true;
}