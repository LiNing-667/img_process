/**
 * @file cv_traditional.cpp
 * @brief 传统 OpenCV 图像处理与极值特征提取
 */
#include "cv_traditional.h"
#include "global_state.h"
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
        if (r_left.area() > 0)
            patches.push_back(blurred(r_left));
        if (r_right.area() > 0)
            patches.push_back(blurred(r_right));
        if (r_bottom.area() > 0)
            patches.push_back(blurred(r_bottom));
        if (patches.empty())
            return false;
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
        if (center_patch_rect.area() <= 0)
            return false;
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

    // ==========================================================
    // 在角点提取前，强制保留最大的白色连通域，其余全部涂黑
    // ==========================================================
    if (class_id == 9)
    {
        vector<vector<Point>> temp_contours;
        findContours(mask, temp_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (!temp_contours.empty())
        {
            int max_idx = 0;
            double max_area = 0;
            for (size_t i = 0; i < temp_contours.size(); i++)
            {
                double area = contourArea(temp_contours[i]);
                if (area > max_area)
                {
                    max_area = area;
                    max_idx = i;
                }
            }
            // 创建全黑画布，仅把面积最大的连通域画上去（涂白）
            Mat clean_mask = Mat::zeros(mask.size(), CV_8UC1);
            drawContours(clean_mask, temp_contours, max_idx, Scalar(255), FILLED);
            mask = clean_mask; // 覆盖回原 mask，彻底消灭孤立噪点
        }
    }
    
    mask.copyTo(out_mask); // 将纯净的图形输出给调试窗口

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty())
        return false;
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
    if (max_area < roi_frame.cols * roi_frame.rows * 0.05)
        return false;
    RotatedRect rect = minAreaRect(best_contour);
    Point2f rect_pts[4];
    rect.points(rect_pts);
    Point2f center = rect.center;

    std::vector<Point2f> corners;

    // ==========================================================
    // 【新增】：基于霍夫直线交点的高鲁棒性四边形提取 (无视外挂杂斑与突起)
    // ==========================================================
    if (class_id == 9)
    {
        Mat edge_mask;
        Canny(mask, edge_mask, 50, 150);
        vector<Vec4i> lines;
        HoughLinesP(edge_mask, lines, 1, CV_PI / 180, 20, 20, 10);

        Vec4i l_t(0,0,0,0), l_b(0,0,0,0), l_l(0,0,0,0), l_r(0,0,0,0);
        float len_t = 0, len_b = 0, len_l = 0, len_r = 0;
        int cx = mask.cols / 2;
        int cy = mask.rows / 2;

        // 1. 在四个象限中分别寻找最长的边缘直线
        for (const auto& l : lines) {
            float dx = l[2] - l[0], dy = l[3] - l[1];
            float len = std::sqrt(dx*dx + dy*dy);
            
            // 计算线段中点
            int mx = std::round((l[0] + l[2]) / 2.0f);
            int my = std::round((l[1] + l[3]) / 2.0f);

            // 防止中点越界
            mx = std::max(0, std::min(mask.cols - 1, mx));
            my = std::max(0, std::min(mask.rows - 1, my));

            if (std::abs(dx) > std::abs(dy)) { // 横向线
                if (my < cy && len > len_t) { 
                    len_t = len; l_t = l; 
                }
                else if (my >= cy && len > len_b) { 
                    // 【极性校验】：下边缘直线的上方必须是白色(主体)，下方必须是黑色(背景)
                    int check_dist = 5; // 探测距离设为 5 像素
                    
                    // 检查上方 (y 减小)
                    bool is_white_above = (my - check_dist >= 0) ? (mask.at<uchar>(my - check_dist, mx) > 0) : true;
                    // 检查下方 (y 增大)
                    bool is_black_below = (my + check_dist < mask.rows) ? (mask.at<uchar>(my + check_dist, mx) == 0) : true;

                    if (is_white_above && is_black_below) {
                        len_b = len; 
                        l_b = l; 
                    }
                }
            } else { // 纵向线
                if (mx < cx && len > len_l) { len_l = len; l_l = l; }
                else if (mx >= cx && len > len_r) { len_r = len; l_r = l; }
            }
        }

        // 2. 如果四条边界线都找到了，利用克莱姆法则求两两交点
        if (len_t > 0 && len_b > 0 && len_l > 0 && len_r > 0) {
            auto intersect = [](Vec4i l1, Vec4i l2) -> Point2f {
                float A1 = l1[3] - l1[1], B1 = l1[0] - l1[2], C1 = A1 * l1[0] + B1 * l1[1];
                float A2 = l2[3] - l2[1], B2 = l2[0] - l2[2], C2 = A2 * l2[0] + B2 * l2[1];
                float det = A1 * B2 - A2 * B1;
                if (std::abs(det) < 1e-5) return Point2f(-1, -1); // 平行无交点
                return Point2f((C1 * B2 - C2 * B1) / det, (A1 * C2 - A2 * C1) / det);
            };

            Point2f tl = intersect(l_t, l_l); // 上与左交于左上
            Point2f tr = intersect(l_t, l_r); // 上与右交于右上
            Point2f bl = intersect(l_b, l_l); // 下与左交于左下
            Point2f br = intersect(l_b, l_r); // 下与右交于右下

            // 校验数学交点是否有效
            if (tl.x != -1 && tr.x != -1 && bl.x != -1 && br.x != -1) {
                corners = {tl, tr, br, bl};
                std::cout << ">>> [霍夫四边形] ID=9 | 成功利用 4 条最长边界线延长相交获取完美四角！" << std::endl;
            }
        }
    }
    // ==========================================================

    // 如果交点提取失败（比如某条边破损太严重没提取到长线），退回原始的点乘极值法兜底
    if (corners.empty())
    {
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
    }

    std::vector<Point2f> top, bot;
    std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b)
              { return a.y < b.y; });
    top.push_back(corners[0]);
    top.push_back(corners[1]);
    bot.push_back(corners[2]);
    bot.push_back(corners[3]);
    if (top[0].x > top[1].x)
        std::swap(top[0], top[1]);
    if (bot[0].x > bot[1].x)
        std::swap(bot[0], bot[1]);
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