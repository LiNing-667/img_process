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
    // 【终极重构】：极值初筛 + 混合几何雕刻算法 (穿透防压扁版)
    // ==========================================================
    if (class_id == 9)
    {
        // 1. 先用点乘极值法获取四个外围极端点
        RotatedRect r_rect = minAreaRect(best_contour);
        Point2f r_pts[4];
        r_rect.points(r_pts);
        Point2f r_center = r_rect.center;

        std::vector<Point2f> raw_corners;
        for (int i = 0; i < 4; i++) {
            Point2f v_dir = r_pts[i] - r_center;
            double max_dot = -1e9;
            Point2f best_pt;
            for (const auto &cp : best_contour) {
                Point2f pt(cp.x, cp.y);
                Point2f v_pt = pt - r_center;
                double dot_prod = v_pt.x * v_dir.x + v_pt.y * v_dir.y;
                if (dot_prod > max_dot) {
                    max_dot = dot_prod;
                    best_pt = pt;
                }
            }
            raw_corners.push_back(best_pt);
        }

        // 排序得到初筛的 左上、右上、左下、右下
        std::vector<Point2f> top_pts, bot_pts;
        std::sort(raw_corners.begin(), raw_corners.end(), [](Point2f a, Point2f b) { return a.y < b.y; });
        top_pts.push_back(raw_corners[0]); top_pts.push_back(raw_corners[1]);
        bot_pts.push_back(raw_corners[2]); bot_pts.push_back(raw_corners[3]);
        if (top_pts[0].x > top_pts[1].x) std::swap(top_pts[0], top_pts[1]);
        if (bot_pts[0].x > bot_pts[1].x) std::swap(bot_pts[0], bot_pts[1]);

        Point2f tl = top_pts[0], tr = top_pts[1];
        Point2f bl = bot_pts[0], br = bot_pts[1];

        // 2. 上侧线：绝对信任 tl 和 tr
        float k_top = (tr.y - tl.y) / (tr.x - tl.x + 1e-5f);
        float b_top = tl.y - k_top * tl.x;

        // 3. 下侧线：穿透式扫描法！
        int best_d = -1;
        int margin = (tr.x - tl.x) * 0.15f; 
        int scan_x_start = std::max(0, (int)tl.x + margin);
        int scan_x_end = std::min(mask.cols - 1, (int)tr.x - margin);

        if (scan_x_end > scan_x_start) {
            bool has_entered_white = false; // 穿透状态标志位
            for (int d = 5; d < mask.rows; d++) {
                int white_count = 0, total_count = 0;
                for (int x = scan_x_start; x <= scan_x_end; x++) {
                    int y = std::round(k_top * x + b_top + d);
                    if (y >= 0 && y < mask.rows) {
                        if (mask.at<uchar>(y, x) > 128) white_count++;
                        total_count++;
                    }
                }
                
                // 越界保护，如果底边直接插到底
                if (total_count == 0) {
                    if (has_entered_white) best_d = std::max(0, d - 2); 
                    break;
                }
                
                float white_ratio = (float)white_count / total_count;
                if (white_ratio > 0.90f) {
                    // 确认扫描线已经完全进入了纯白的主体内部
                    has_entered_white = true; 
                } else if (has_entered_white && white_ratio < 0.85f) {
                    // 只有在进入主体后，白点率再次跌破 85%，才说明切出了下边界！
                    best_d = std::max(0, d - 2); 
                    break;
                }
            }
        }
        if (best_d == -1) best_d = std::max(10, (int)(bl.y - tl.y)); // 极度异常兜底

        // 定义检测某条线段是否主体纯白的闭包函数 
        auto isLineWhite = [&](Point2f p1, Point2f p2) -> bool {
            int steps = std::max(10, (int)std::max(std::abs(p1.x - p2.x), std::abs(p1.y - p2.y)));
            int white_hits = 0, valid_pts = 0;
            // 【核心防御】：掐头去尾，跳过线段两端 15% 的圆角和毛刺，只测最直的中段！
            int start_i = steps * 0.15f;
            int end_i = steps * 0.85f;
            if (start_i >= end_i) return false;

            for (int i = start_i; i <= end_i; i++) {
                float r = (float)i / steps;
                int px = std::round(p1.x + r * (p2.x - p1.x));
                int py = std::round(p1.y + r * (p2.y - p1.y));
                if (px >= 0 && px < mask.cols && py >= 0 && py < mask.rows) {
                    if (mask.at<uchar>(py, px) > 128) white_hits++;
                    valid_pts++;
                }
            }
            if (valid_pts == 0) return false;
            return (float)white_hits / valid_pts >= 0.85f; 
        };

        // 4. 左侧线修正：以 tl 为圆心向内逆时针旋转
        Point2f new_bl(bl.x, k_top * bl.x + b_top + best_d); 
        for (int i = 0; i < 200; i++) { 
            if (isLineWhite(tl, new_bl)) break;
            new_bl.x += 1.0f; // 向右移
            new_bl.y = k_top * new_bl.x + b_top + best_d; // 强制落在底线上
        }

        // 5. 右侧线修正：整体向内平移
        Point2f new_tr = tr;
        Point2f new_br(br.x, k_top * br.x + b_top + best_d); 
        for (int i = 0; i < 200; i++) { 
            if (isLineWhite(new_tr, new_br)) break;
            new_tr.x -= 1.0f; // 向左移
            new_tr.y = k_top * new_tr.x + b_top;          
            new_br.x -= 1.0f;
            new_br.y = k_top * new_br.x + b_top + best_d; 
        }

        // 组装最终完全切合纯白实体的四个点
        corners = {tl, new_tr, new_br, new_bl};
        std::cout << ">>> [无敌雕刻] ID=9 | 成功执行穿透顶线下移、防圆角侧边收缩修正！" << std::endl;
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