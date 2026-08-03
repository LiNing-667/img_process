/**
 * @file vision_engine.cpp
 * @brief 核心业务状态机与任务管线
 */
#include "vision_engine.h"
#include "global_state.h"
#include "config.h"
#include "ai_infer.h"
#include "cv_traditional.h"
#include <opencv2/aruco.hpp>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <map>

using namespace cv;
using namespace std;

// 【新增】：用于宏动作链的 A/B 状态机标志位
bool g_wf_align_done = false;    // A 信号：动作执行完成
bool g_wf_align_success = false; // B 信号：精度已完全达标
bool g_reset_align_memory = false;

namespace TaskManager
{
    DemoTask fetchTask()
    {
        std::lock_guard<std::mutex> lock(g_task_mtx);
        DemoTask task;
        if (g_demo_task.pending)
        {
            task = g_demo_task;
            g_demo_task.pending = false;
        }
        return task;
    }
}

VisionEngine::VisionEngine(PilotCommunicator &comm) : pilot_comm(comm) {}

void VisionEngine::handleClosedLoopCheck(const DemoTask &current_task, Mat &raw_frame)
{
    std::map<std::string, CLTransition> cl_map = {
        {"CHECK_H11", {1, {1}, "DEMO101", "DEMO001"}},
        {"CHECK_H21", {2, {2, 3}, "DEMO001", "DEMO021"}},
        {"CHECK_H12", {1, {1}, "DEMO102", "DEMO002"}},
        {"CHECK_H22", {2, {2, 3}, "DEMO002", "DEMO022"}}};
    if (cl_map.find(current_task.raw_cmd) == cl_map.end())
    {
        cout << "[视觉闭环] 未知的闭环复核指令: " << current_task.raw_cmd << endl;
        return;
    }
    CLTransition trans = cl_map[current_task.raw_cmd];
    cout << "[视觉闭环] 正在扫描 ID=" << trans.target_id << " 进行拼装位重复核..." << endl;

    current_yolo_res = runYoloInference(raw_frame, trans.target_id);
    has_detection = true;
    bool check_passed = false;
    bool found_target_obj = false;
    Point2f active_obj_center, active_target_pt;

    if (current_yolo_res.detected)
    {
        for (auto &obj : current_yolo_res.objects)
        {
            if (obj.class_id == trans.target_id)
            {
                found_target_obj = true;
                active_obj_center = obj.center;
                active_target_pt = getBasePoint(trans.required_points[0], g_cl_state.base_corners_2d);
                bool all_points_in = true;
                float margin = 10.0f;
                for (int pt_idx : trans.required_points)
                {
                    Point2f pt = getBasePoint(pt_idx, g_cl_state.base_corners_2d);
                    if (!(pt.x >= obj.bbox.x && pt.x <= obj.bbox.x + obj.bbox.width &&
                          pt.y >= obj.bbox.y && pt.y <= obj.bbox.y + obj.bbox.height - margin))
                    {
                        all_points_in = false;
                        break;
                    }
                }
                if (all_points_in)
                    check_passed = true;
                break;
            }
        }
    }
    if (check_passed)
    {
        cout << ">>> [闭环成功] 指定特征点被完美覆盖，装配精准！下发 " << trans.success_cmd << "..." << endl;
        g_cl_state.retry_count = 0;
        int next_arm = trans.success_cmd[4] - '0';
        Pose6D next_pose = calibrator.transform(g_cl_state.last_rvec, g_cl_state.last_tvec, next_arm);
        next_pose.x /= -10.0;
        next_pose.y /= -10.0;
        next_pose.z /= -10.0;
        next_pose.x += g_arm_x_offset_cm[next_arm];
        g_cl_state.last_pose = next_pose;
        pilot_comm.sendDemoCommand(trans.success_cmd, next_pose);
    }
    else
    {
        g_cl_state.retry_count++;
        cout << ">>> [闭环失败] 残差未达标。计算图像伺服反馈... (重试:" << g_cl_state.retry_count << ")" << endl;
        if (found_target_obj)
        {
            const float STEP_CM = 0.5f;
            float dx_pixel = active_obj_center.x - active_target_pt.x;
            float dy_pixel = active_obj_center.y - active_target_pt.y;
            cout << "    [残差分析] 像素偏差 -> DX: " << dx_pixel << " px | DY: " << dy_pixel << " px" << endl;
            if (dx_pixel > 0.0f)
            {
                g_cl_state.last_pose.y -= STEP_CM;
                cout << "    [伺服决策] 目标框偏右，机械臂 Y 轴减小" << endl;
            }
            else
            {
                g_cl_state.last_pose.y += STEP_CM;
                cout << "    [伺服决策] 目标框偏左，机械臂 Y 轴增大" << endl;
            }

            if (dy_pixel > 0.0f)
            {
                g_cl_state.last_pose.x -= STEP_CM;
                cout << "    [伺服决策] 目标框偏下，机械臂 X 轴减小" << endl;
            }
            else
            {
                g_cl_state.last_pose.x += STEP_CM;
                cout << "    [伺服决策] 目标框偏上，机械臂 X 轴增大" << endl;
            }
        }
        else
        {
            cout << "    [伺服警告] 视野内丢失目标类目标框，保持原位坐标重试。" << endl;
        }
        pilot_comm.sendDemoCommand(trans.retry_cmd, g_cl_state.last_pose);
    }
}

void VisionEngine::handleSingleAxisServo(const DemoTask &current_task, Mat &raw_frame)
{
    Mat clean_gray;
    cvtColor(raw_frame, clean_gray, COLOR_BGR2GRAY);
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->minMarkerPerimeterRate = 0.01;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(clean_gray, dictionary, marker_corners, marker_ids, parameters);
    Point2f aruco_center(-1, -1);
    if (!marker_ids.empty())
    {
        Point2f p1 = marker_corners[0][0];
        Point2f p2 = marker_corners[0][1];
        Point2f p3 = marker_corners[0][2];
        Point2f p4 = marker_corners[0][3];
        aruco_center = Point2f((p1.x + p2.x + p3.x + p4.x) / 4.0f, (p1.y + p2.y + p3.y + p4.y) / 4.0f);
    }
    Point2f obj_center = g_cl_state.last_obj_center;
    if (aruco_center.y >= 0 && obj_center.y >= 0 && !g_cl_state.last_tvec.empty())
    {
        double tz_mm = g_cl_state.last_tvec.at<double>(2);
        float dynamic_scale_cm = (float)(tz_mm / 7053.0);
        float delta_x_cm = (obj_center.y - aruco_center.y) * dynamic_scale_cm;
        // g_global_x_offset_cm += delta_x_cm; // 将本次算出的误差，累加到全局补偿系统中

        // 从指令如 "FIX_131" 的第 4 位提取机械臂 ID，默认 fallback 到 ARM1
        int target_arm = (current_task.raw_cmd.length() > 4) ? (current_task.raw_cmd[4] - '0') : 1;
        if (target_arm != 0 && target_arm != 1)
            target_arm = 1;

        g_arm_x_offset_cm[target_arm] += delta_x_cm; // 独立累加到各自的补偿系统中

        float calibrated_px = g_cl_state.last_pose.x + delta_x_cm;
        std::cout << "\n>>> [视觉对齐成功] PnP 深度提取: Tz=" << tz_mm << "mm | 动态比例尺: " << dynamic_scale_cm << " cm/px" << std::endl;
        std::cout << ">>> ArUco纵向: " << aruco_center.y << " | 物体纵向: " << obj_center.y << std::endl;
        std::cout << ">>> X轴(前后) 校准量: " << delta_x_cm << " cm" << std::endl;
        Pose6D fix_pose = g_cl_state.last_pose;
        fix_pose.x = calibrated_px;
        // 根据接收到的 FIX 信号，动态决定下一步该发什么抓取指令
        std::string next_cmd = "DEMO112";
        if (current_task.raw_cmd == "FIX_131")
        {
            next_cmd = "DEMO132";
        }
        pilot_comm.sendDemoCommand(next_cmd, fix_pose);
    }
    else
    {
        std::cout << "\n>>> [伺服失败] 未找齐 ArUco 与目标，或深度矩阵丢失！1秒后重试..." << std::endl;
        usleep(1000000);
        std::lock_guard<std::mutex> lock(g_task_mtx);
        g_demo_task.pending = true;
    }
}

bool VisionEngine::handleBlindOperations(const DemoTask &current_task)
{
    if (current_task.raw_cmd == "DEMO004")
    {
        if (!g_cl_state.last_rvec.empty() && !g_cl_state.last_tvec.empty())
        {
            cout << ">>> [视觉记忆跳跃] 检测到单独下发 " << current_task.raw_cmd << "，调用物理矩阵跳过 YOLO！" << endl;
            Pose6D arm0_pose = calibrator.transform(g_cl_state.last_rvec, g_cl_state.last_tvec, 0);
            arm0_pose.x /= -10.0;
            arm0_pose.y /= -10.0;
            arm0_pose.z /= -10.0;
            arm0_pose.x += g_arm_x_offset_cm[0]; // DEMO004 专属 ARM0
            g_cl_state.last_pose = arm0_pose;
            pilot_comm.sendDemoCommand(current_task.raw_cmd, arm0_pose);
            return true;
        }
        else
        {
            cout << ">>> [警告] 内存中暂无底座矩阵(请先执行101/102)，降级为重新视觉扫描..." << endl;
        }
    }
    return false;
}

bool VisionEngine::handleYoloAndPnP(const DemoTask &current_task, Mat &raw_frame)
{
    cout << "[Monitor] 正在执行神经网络与 6D 位姿解算 (锁定ID=" << current_task.class_id << ")..." << endl;

    // 针对 ID=9，使用动态 HSV 提取最大蓝色区域，并裁切上下 10% 后送入 NEXT 模型
    if (current_task.class_id == 9)
    {
        Mat hsv, mask;
        cvtColor(raw_frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(95, 80, 40), Scalar(140, 255, 255), mask);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        Rect best_rect(0, 0, 0, 0);
        for (const auto &contour : contours)
        {
            double area = contourArea(contour);
            if (area > max_area)
            {
                max_area = area;
                best_rect = boundingRect(contour);
            }
        }

        // 面积阈值同样设为 10000 过滤噪点
        if (max_area > 10000.0)
        {
            // 左右各扩60，上下各缩50
            best_rect.x -= 60;
            best_rect.y += 50;
            best_rect.width += 120;
            best_rect.height -= 100; // 上侧下侧各缩50

            // 防止拓展后越界导致程序崩溃
            best_rect &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

            // 【UI 可视化】：在主画面中直接用醒目的青色加粗画出拓展 50px 后的框
            rectangle(raw_frame, best_rect, Scalar(255, 255, 0), 3);
            putText(raw_frame, "Expanded ROI (+50px)", Point(best_rect.x, max(best_rect.y - 10, 10)), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);

            current_yolo_res.detected = true;
            current_yolo_res.objects.clear();

            ObjectMeta obj9;
            obj9.bbox = best_rect;
            obj9.center = Point2f(obj9.bbox.x + obj9.bbox.width / 2.0f, obj9.bbox.y + obj9.bbox.height / 2.0f);
            obj9.class_id = 9;
            obj9.confidence = 1.0f; // 虚拟置信度 100%
            obj9.has_refined_center = false;

            current_yolo_res.objects.push_back(obj9);
            cout << ">>> [特殊模式] 截获 ID=9，提取最大 HSV 蓝色区域，并向外拓展 50 像素作为 YOLO 选区！" << endl;
        }
        else
        {
            current_yolo_res.detected = false;
            cout << ">>> [特殊模式] 截获 ID=9，但视野内未发现足够大的蓝色区域！" << endl;
        }
    }
    // ==========================================================
    // 【新增】：仅针对 DEMO001，直接使用画面中心偏移划定绝对安全框！
    // 其他 ID=0 的任务（如 DEMO000）依然会正常执行 YOLO 搜索
    // ==========================================================
    else if (current_task.raw_cmd == "DEMO001" || current_task.raw_cmd == "align92")
    {
        current_yolo_res.detected = true;
        current_yolo_res.objects.clear();

        ObjectMeta obj0;
        // 提取画面绝对中点
        int cx = raw_frame.cols / 2;
        int cy = raw_frame.rows / 2;

        // 左侧360，右侧420，下侧240，上侧330
        int left = cx - 360;
        int right = cx + 420;
        int top = cy - 330;
        int bottom = cy + 240;

        obj0.bbox = Rect(left, top, right - left, bottom - top);
        obj0.center = Point2f(obj0.bbox.x + obj0.bbox.width / 2.0f, obj0.bbox.y + obj0.bbox.height / 2.0f);
        obj0.class_id = 0;
        obj0.confidence = 1.0f; // 虚拟置信度 100%
        obj0.has_refined_center = false;

        current_yolo_res.objects.push_back(obj0);
        cout << ">>> [特殊模式] 截获 DEMO001，使用基于画面中点的硬编码选区代替 best.pt！" << endl;
    }
    else
    {
        // 对于其他 ID (包括 DEMO000, DEMO002 等)，走正常的 YOLO 推理流程
        current_yolo_res = runYoloInference(raw_frame, current_task.class_id);
    }

    has_detection = true;
    bool target_found = false;

    if (current_yolo_res.detected)
    {
        for (auto &obj : current_yolo_res.objects)
        {
            if (obj.class_id == 0 || obj.class_id == 9)
            {
                Rect safe_crop = obj.bbox & Rect(0, 0, raw_frame.cols, raw_frame.rows);
                if (safe_crop.area() > 0)
                {
                    Mat roi_frame = raw_frame(safe_crop);
                    std::vector<Point2f> raw_centers = runNextYoloInferenceRaw(roi_frame);
                    std::vector<Point2f> global_raw;
                    int dropped_points = 0;
                    for (const auto &pt : raw_centers)
                    {
                        Point2f global_pt(pt.x + safe_crop.x, pt.y + safe_crop.y);

                        // 确保坐标不越界
                        int px = std::max(0, std::min(raw_frame.cols - 1, (int)std::round(global_pt.x)));
                        int py = std::max(0, std::min(raw_frame.rows - 1, (int)std::round(global_pt.y)));

                        // ==========================================================
                        // 【修正】：给 ID=9 开绿灯，只有 ID=0 才需要经过掩码严格校验
                        // ==========================================================
                        if (obj.class_id == 9 || current_task.raw_cmd == "DEMO001" || current_task.raw_cmd == "align92" || (!obj.ai_mask.empty() && obj.ai_mask.at<uchar>(py, px) > 0))
                        {
                            global_raw.push_back(global_pt);
                        }
                        else
                        {
                            dropped_points++;
                        }
                    }

                    obj.sub_centers = clusterPoints(global_raw, 12.0f);
                    cout << ">>> [二次级联] ID=" << obj.class_id << " | next.pt 原始点:" << raw_centers.size()
                         << " | 掩码过滤掉噪点:" << dropped_points
                         << " | 最终聚类有效特征点:" << obj.sub_centers.size() << endl;

                    // 注意：只有 ID=0 才去做透视矩阵的角点修复
                    if (obj.class_id == 0 && obj.sub_centers.size() >= 4)
                    {
                        bool perspective_fixed = false;
                        std::vector<Point2f> pts = obj.sub_centers;

                        // 定义核心游走引擎
                        auto pushPoint = [](Point2f cur, const std::vector<Point2f> &pts_list, char dir) -> Point2f
                        {
                            std::vector<Point2f> valid_pts;
                            for (auto p : pts_list)
                            {
                                if (norm(p - cur) < 5.0f)
                                    continue;
                                if (dir == 'R' && p.x > cur.x + 5.0f)
                                    valid_pts.push_back(p);
                                if (dir == 'L' && p.x < cur.x - 5.0f)
                                    valid_pts.push_back(p);
                                if (dir == 'U' && p.y < cur.y - 5.0f)
                                    valid_pts.push_back(p);
                            }
                            if (valid_pts.empty())
                                return cur;

                            std::sort(valid_pts.begin(), valid_pts.end(), [cur](Point2f a, Point2f b)
                                      { return (abs(a.x - cur.x) + abs(a.y - cur.y)) < (abs(b.x - cur.x) + abs(b.y - cur.y)); });

                            int k = std::min(3, (int)valid_pts.size());
                            Point2f best_pt = cur;
                            float min_diff = 1e9;
                            for (int i = 0; i < k; ++i)
                            {
                                Point2f p = valid_pts[i];
                                float diff = (dir == 'U') ? abs(p.x - cur.x) : abs(p.y - cur.y);
                                if (diff < min_diff)
                                {
                                    min_diff = diff;
                                    best_pt = p;
                                }
                            }
                            return best_pt;
                        };

                        std::vector<Point2f> final_corners;

                        // ==============================================================
                        // 动态场景分支：根据当前 Demo 的进度，应用不同的拓扑推导策略
                        // ==============================================================
                        // ==============================================================
                        // 动态场景分支：根据当前 Demo 的进度，应用不同的拓扑推导策略
                        // ==============================================================
                        if (current_task.action_id == 0)
                        {
                            if (pts.size() < 12)
                            {
                                // 【DEMO000 遮挡/缺点分支】：点数少于12个
                                // 根据前提条件：1, 4, 5, 12 号点必然存活
                                Point2f P1 = pts[0], P4 = pts[0];
                                float min_x_minus_y = 1e9, max_x_plus_y = -1e9;
                                for (auto p : pts)
                                {
                                    if (p.x - p.y < min_x_minus_y)
                                    {
                                        min_x_minus_y = p.x - p.y;
                                        P1 = p;
                                    } // 1号点：X最小Y最大
                                    if (p.x + p.y > max_x_plus_y)
                                    {
                                        max_x_plus_y = p.x + p.y;
                                        P4 = p;
                                    } // 4号点：X最大Y最大
                                }

                                // 利用 pushPoint 获取两侧的第二层点
                                Point2f P5 = pushPoint(P4, pts, 'U');
                                Point2f P12 = pushPoint(P1, pts, 'U');

                                // 【核心杀招】：构建单应性透视矩阵 (Homography)
                                // 我们把底座看作一个 3x3 的标准网格，建立物理坐标与像素坐标的映射关系
                                std::vector<Point2f> src_pts = {
                                    Point2f(0, 0), // 对应 1号点 (左下)
                                    Point2f(3, 0), // 对应 4号点 (右下)
                                    Point2f(3, 1), // 对应 5号点 (右侧上一层)
                                    Point2f(0, 1)  // 对应 12号点 (左侧上一层)
                                };
                                std::vector<Point2f> dst_pts = {P1, P4, P5, P12};
                                Mat H = getPerspectiveTransform(src_pts, dst_pts);

                                // 利用透视矩阵，精准推算出 10号点 和 7号点 的像素坐标
                                // 在理想网格中，它们位于最顶层 (Y = 3)
                                std::vector<Point2f> target_src = {Point2f(0, 3), Point2f(3, 3)};
                                std::vector<Point2f> target_dst;
                                perspectiveTransform(target_src, target_dst, H);

                                Point2f P10 = target_dst[0]; // 左上角
                                Point2f P7 = target_dst[1];  // 右上角

                                final_corners = {P10, P7, P4, P1};
                                perspective_fixed = true;
                                cout << ">>> [DEMO000 缺点修补] 激活单应性透视推演：完美预测出具备真实透视畸变的 7/10 号角点！" << endl;
                            }
                            else
                            {
                                // 【DEMO000 完美状态】：点数充足，直接利用极值锁定四大外围角点
                                Point2f P1 = pts[0], P4 = pts[0], P7 = pts[0], P10 = pts[0];
                                float min_x_minus_y = 1e9;
                                float max_x_plus_y = -1e9;
                                float max_x_minus_y = -1e9;
                                float min_x_plus_y = 1e9;

                                for (auto p : pts)
                                {
                                    if (p.x - p.y < min_x_minus_y)
                                    {
                                        min_x_minus_y = p.x - p.y;
                                        P1 = p;
                                    }
                                    if (p.x + p.y > max_x_plus_y)
                                    {
                                        max_x_plus_y = p.x + p.y;
                                        P4 = p;
                                    }
                                    if (p.x - p.y > max_x_minus_y)
                                    {
                                        max_x_minus_y = p.x - p.y;
                                        P7 = p;
                                    }
                                    if (p.x + p.y < min_x_plus_y)
                                    {
                                        min_x_plus_y = p.x + p.y;
                                        P10 = p;
                                    }
                                }

                                final_corners = {P10, P7, P4, P1};
                                perspective_fixed = true;
                                cout << ">>> [DEMO000 四角锁定] 点数充足，成功直接提取 1/4/7/10 号角点！" << endl;
                            }
                        }
                        else if (current_task.action_id == 1)
                        {
                            // 【DEMO001】：1号点被挡，找右下角的点为4号点，逆向推导
                            Point2f P4 = pts[0];
                            float max_x_plus_y = -1e9;
                            for (auto p : pts)
                            {
                                // 右下角 (4号点)：X最大，Y最大 -> x + y 最大
                                if (p.x + p.y > max_x_plus_y)
                                {
                                    max_x_plus_y = p.x + p.y;
                                    P4 = p;
                                }
                            }
                            Point2f P3 = pushPoint(P4, pts, 'L');
                            Point2f P2 = pushPoint(P3, pts, 'L');

                            // 向左补一个点变为 1号点 (利用 2和3 的间距)
                            Point2f P1 = P2 + (P2 - P3);
                            // 兜底：如果推点失败导致点重合，给定一个经验横移量
                            if (norm(P2 - P3) < 5.0f)
                            {
                                P1 = Point2f(P2.x - 30.0f, P2.y);
                            }
                            Point2f P5 = pushPoint(P4, pts, 'U');
                            Point2f P6 = pushPoint(P5, pts, 'U');
                            Point2f P7 = pushPoint(P6, pts, 'U');
                            Point2f P8 = pushPoint(P7, pts, 'L');
                            Point2f P9 = pushPoint(P8, pts, 'L');
                            Point2f P10 = P9 + (P9 - P8);
                            // 兜底：根据矩形对角线向量补齐
                            if (norm(P9 - P8) < 5.0f)
                            {
                                P10 = P1 + P7 - P4;
                            }

                            final_corners = {P10, P7, P4, P1};
                            perspective_fixed = true;
                            cout << ">>> [DEMO001 右侧起手] 成功锁定右下角 4号点，逆向向左游走完美推演出 1/10 号角点！" << endl;
                        }
                        else if (current_task.action_id == 3)
                        {
                            // 【DEMO003】：1号点被挡住，从2号点起手推导
                            Point2f P2 = pts[0];
                            float min_x_minus_y = 1e9;
                            for (auto p : pts)
                            {
                                // 因为 1 号点没了，X-Y 最小的自然就是最左下的 2 号点
                                if (p.x - p.y < min_x_minus_y)
                                {
                                    min_x_minus_y = p.x - p.y;
                                    P2 = p;
                                }
                            }
                            Point2f P3 = pushPoint(P2, pts, 'R');
                            Point2f P4 = pushPoint(P3, pts, 'R'); // 向右推两次到 4
                            Point2f P5 = pushPoint(P4, pts, 'U');
                            Point2f P6 = pushPoint(P5, pts, 'U');
                            Point2f P7 = pushPoint(P6, pts, 'U'); // 向上推三次到 7
                            Point2f P8 = pushPoint(P7, pts, 'L');
                            Point2f P9 = pushPoint(P8, pts, 'L'); // 向左推两次到 9

                            // 向左推演补齐 10号点
                            Point2f P10 = P9 + (P9 - P8);
                            // 向左推演补齐 1号点 (利用 2和3 的间距)
                            Point2f P1 = P2 - (P3 - P2);

                            final_corners = {P10, P7, P4, P1};
                            perspective_fixed = true;
                            cout << ">>> [DEMO003 严重遮挡] 1号点丢失，从 2号点游走推导，双向完美补齐 1/10 号角点！" << endl;
                        }
                        else
                        {
                            // 【DEMO002】(及默认状态)：1号点起手，推导1~9并补10
                            Point2f P1 = pts[0];
                            float min_x_minus_y = 1e9;
                            for (auto p : pts)
                            {
                                if (p.x - p.y < min_x_minus_y)
                                {
                                    min_x_minus_y = p.x - p.y;
                                    P1 = p;
                                }
                            }
                            Point2f P2 = pushPoint(P1, pts, 'R');
                            Point2f P3 = pushPoint(P2, pts, 'R');
                            Point2f P4 = pushPoint(P3, pts, 'R'); // 向右推三次到 4
                            Point2f P5 = pushPoint(P4, pts, 'U');
                            Point2f P6 = pushPoint(P5, pts, 'U');
                            Point2f P7 = pushPoint(P6, pts, 'U'); // 向上推三次到 7
                            Point2f P8 = pushPoint(P7, pts, 'L');
                            Point2f P9 = pushPoint(P8, pts, 'L'); // 向左推两次到 9

                            // 补齐 10号点
                            Point2f P10 = P9 + (P9 - P8);

                            final_corners = {P10, P7, P4, P1};
                            perspective_fixed = true;
                            cout << ">>> [DEMO002 标准遮挡] 从 1号点发起游走，成功利用向量法补全 10 号角点！" << endl;
                        }

                        if (perspective_fixed)
                        {
                            std::vector<Point2f> top, bot;
                            std::sort(final_corners.begin(), final_corners.end(), [](Point2f a, Point2f b)
                                      { return a.y < b.y; });
                            top.push_back(final_corners[0]);
                            top.push_back(final_corners[1]);
                            bot.push_back(final_corners[2]);
                            bot.push_back(final_corners[3]);
                            if (top[0].x > top[1].x)
                                std::swap(top[0], top[1]);
                            if (bot[0].x > bot[1].x)
                                std::swap(bot[0], bot[1]);
                            obj.corners_2d = {top[0], top[1], bot[1], bot[0]};
                        }

                        // ==============================================================
                        // 万能兜底逻辑（如果点数不足 9 个，退回标准 4 点包裹提取）
                        // ==============================================================
                        if (!perspective_fixed)
                        {
                            RotatedRect min_rect = minAreaRect(obj.sub_centers);
                            Point2f rect_pts[4];
                            min_rect.points(rect_pts);
                            std::vector<Point2f> corners;
                            std::vector<Point2f> available_pts = obj.sub_centers;
                            for (int i = 0; i < 4; i++)
                            {
                                int best_idx = -1;
                                float min_dist = 1e9;
                                for (size_t j = 0; j < available_pts.size(); j++)
                                {
                                    float d = norm(rect_pts[i] - available_pts[j]);
                                    if (d < min_dist)
                                    {
                                        min_dist = d;
                                        best_idx = j;
                                    }
                                }
                                if (best_idx != -1)
                                {
                                    corners.push_back(available_pts[best_idx]);
                                    available_pts.erase(available_pts.begin() + best_idx);
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
                            obj.corners_2d = {top[0], top[1], bot[1], bot[0]};
                        }
                    }

                    // 利用全量几何极值锁定 P1(左上) 和 P4(右下)
                    if (obj.class_id == 9 && obj.sub_centers.size() >= 4)
                    {
                        std::vector<Point2f> pts = obj.sub_centers;
                        if (pts.size() > 40)
                            pts.resize(40);

                        // ≥10个点时只保留画面最下侧的10个，滤掉顶部噪点
                        if (pts.size() >= 10)
                        {
                            std::sort(pts.begin(), pts.end(),
                                      [](Point2f a, Point2f b)
                                      { return a.y > b.y; });
                            pts.resize(10); 
                            cout << ">>> [P1智能] ID=9 | 点数≥10，仅保留最下侧10个点" << endl;
                        }

                        // ==========================================================
                        // 【重构 P4 智能算法】：提取最靠右下角的两个点，利用夹角过滤假点
                        // ==========================================================
                        std::vector<Point2f> pts_sorted_br = pts;
                        // 按 (X+Y) 降序排列，最前面的就是最靠右下角的点
                        std::sort(pts_sorted_br.begin(), pts_sorted_br.end(),
                                  [](Point2f a, Point2f b) { return (a.x + a.y) > (b.x + b.y); });

                        Point2f P4 = pts_sorted_br[0]; // 默认取 max(X+Y)

                        if (pts_sorted_br.size() >= 2)
                        {
                            Point2f pt1 = pts_sorted_br[0];
                            Point2f pt2 = pts_sorted_br[1];

                            // 在这个作用域内，安全框的变量名叫 safe_crop
                            float cx = safe_crop.x + safe_crop.width / 2.0f;
                            float cy = safe_crop.y + safe_crop.height / 2.0f;

                            // 判定点是否都在第 4 象限 (右下角：X大于中点 且 Y大于中点)
                            bool pt1_in_q4 = (pt1.x > cx && pt1.y > cy);
                            bool pt2_in_q4 = (pt2.x > cx && pt2.y > cy);

                            if (pt1_in_q4 && pt2_in_q4)
                            {
                                // 计算两点连线与水平方向的夹角
                                float dx = std::abs(pt1.x - pt2.x);
                                float dy = std::abs(pt1.y - pt2.y);
                                float angle = std::atan2(dy, dx + 1e-5f) * 180.0f / CV_PI;

                                // OpenCV 坐标系 Y 轴向下增大，Y 值越大越靠下方
                                Point2f pt_lower = (pt1.y > pt2.y) ? pt1 : pt2;
                                Point2f pt_higher = (pt1.y > pt2.y) ? pt2 : pt1;

                                cout << ">>> [P4智能] ID=9 | 右下两备选点齐聚第四象限，水平夹角: " << angle << " 度" << endl;

                                if (angle < 65.0f)
                                {
                                    P4 = pt_higher;
                                    cout << ">>> [P4智能] 夹角 < 65度，判定更靠下的点为杂散假点！修正 P4 为上方真实点。" << endl;
                                }
                                else
                                {
                                    P4 = pt_lower;
                                    cout << ">>> [P4智能] 夹角 >= 65度，边缘陡峭正常，锁定下方为真实 P4 点。" << endl;
                                }
                            }
                            else
                            {
                                cout << ">>> [P4智能] ID=9 | 右下角两点未齐聚第四象限，使用标准 max(X+Y) 兜底" << endl;
                            }
                        }

                        


                        // ---- P1 新算法：左侧两点连线法 ----
                        // 1. 找最左侧的两个点 C 和 D
                        std::vector<Point2f> sorted = pts;
                        std::sort(sorted.begin(), sorted.end(),
                                  [](Point2f a, Point2f b)
                                  { return a.x < b.x; });
                        Point2f C = sorted[0], D = sorted[1];

                        // 2. 计算 CD 与水平方向夹角
                        float dx_cd = D.x - C.x;
                        float dy_cd = D.y - C.y;
                        float angle_cd = std::abs(std::atan2(dy_cd, dx_cd) * 180.0f / CV_PI);

                        cout << ">>> [P1智能] ID=9 | C(" << C.x << "," << C.y
                             << ") D(" << D.x << "," << D.y
                             << ") 夹角=" << angle_cd << "度" << endl;

                        Point2f P1;
                        if (angle_cd < 60.0f)
                        {
                            // 左侧边缘大致水平（夹角<60度）：直接取那条线上最左侧的点（即点C）
                            P1 = C;
                            cout << ">>> [P1智能] 夹角<60°, 直接锁定最左侧点 C 作为 P1" << endl;
                        }
                        else
                        {
                            // 左侧边缘陡峭 → 向下延长 CD，找最后一个靠近的点
                            // 方向向量 (沿 CD 向下，即 dy > 0)
                            Point2f dir(dx_cd, dy_cd);
                            if (dir.y < 0)
                                dir = -dir; // 确保向下
                            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                            if (len > 0)
                            {
                                dir.x /= len;
                                dir.y /= len;
                            }

                            // 从 C 沿 CD 向下找最后一个 ≤20px 的点
                            Point2f best = C.y > D.y ? C : D; // 起手选较下方的
                            float best_t = 0;
                            for (auto p : pts)
                            {
                                // 点到直线 CD 的距离
                                float cross = std::abs((p.x - C.x) * dy_cd - (p.y - C.y) * dx_cd);
                                float dist = cross / std::sqrt(dx_cd * dx_cd + dy_cd * dy_cd);
                                if (dist < 20.0f)
                                {
                                    // 投影到 CD 方向上的 t 值（越大越靠下）
                                    float t = (p.x - C.x) * dir.x + (p.y - C.y) * dir.y;
                                    if (t > best_t)
                                    {
                                        best_t = t;
                                        best = p;
                                    }
                                }
                            }

                            if (best_t > 0)
                            {
                                P1 = best;
                                cout << ">>> [P1智能] 夹角≥60°, 沿CD向下延伸找到 P1" << endl;
                            }
                            else
                            {
                                P1 = (C.y > D.y) ? C : D;
                                cout << ">>> [P1智能] 未找到沿线点, 取C/D中更下方者" << endl;
                            }
                        }

                        // 防呆：如果 P1 和 P4 太近，用 X极值兜底
                        if (norm(P1 - P4) < 30.0f)
                        {
                            float min_x = 1e9, max_x = -1e9;
                            for (auto p : pts)
                            {
                                if (p.x < min_x)
                                {
                                    min_x = p.x;
                                    P1 = p;
                                }
                                if (p.x > max_x)
                                {
                                    max_x = p.x;
                                    P4 = p;
                                }
                            }
                        }

                        cout << ">>> [几何极值] ID=9 | P1(" << P1.x << "," << P1.y
                             << ") P4(" << P4.x << "," << P4.y << ")" << endl;

                        // 生成虚拟紧凑框并执行四向膨胀
                        int new_x = P1.x - 290;
                        int new_y = P1.y; // 上侧不扩
                        int new_w = (P4.x - P1.x) + 310;
                        int new_h = (P4.y - P1.y) + 70; // 下侧往下扩70

                        obj.bbox = Rect(new_x, new_y, new_w, new_h);
                        cout << ">>> [几何极值] ID=9 已锁定对角点，生成虚拟选区" << endl;
                    }
                    else if (obj.class_id == 9)
                    {
                        obj.bbox = Rect(0, 0, 0, 0);
                    }
                    // -------------------------------------------------------------
                }
            }

            obj.has_refined_center = false;

            // ==========================================================
            // 【新增】：针对 ID=1~3，将 YOLO 框向四周放大 30 像素，防止切掉角点
            // ==========================================================
            if (obj.class_id >= 1 && obj.class_id <= 3)
            {
                int expand_px = 30;
                obj.bbox.x -= expand_px;
                obj.bbox.y -= expand_px;
                // 注意：因为左右各宽了30，所以 width 增加了 60；上下同理
                obj.bbox.width += expand_px * 2;
                obj.bbox.height += expand_px * 2;
            }

            // 这一步与原画幅进行求交集 ( & Rect )，完美防止放大后越界导致的 Crash
            Rect safe_bbox = obj.bbox & Rect(0, 0, raw_frame.cols, raw_frame.rows);
            // 【调试放行】：ID=9 即使 bbox 为空，也用原始虚拟框跑二值化，方便图传排查
            if (safe_bbox.area() <= 0)
            {
                if (obj.class_id == 9)
                    safe_bbox = Rect(20, 0, 800, 460) & Rect(0, 0, raw_frame.cols, raw_frame.rows);
                if (safe_bbox.area() <= 0)
                    continue;
            }

            // ==========================================================
            // 【新增】：二值化之后，找面积最大的白色，删掉右10%、左5%，用最左上和最右上点连线！
            // ==========================================================
            float custom_tilt_angle = -999.0f;
            Vec4i best_line(0, 0, 0, 0);

            if (current_task.raw_cmd == "align91")
            {
                Mat roi_hsv;
                cvtColor(raw_frame(safe_bbox), roi_hsv, COLOR_BGR2HSV);

                Mat blue_mask;
                inRange(roi_hsv, Scalar(95, 80, 40), Scalar(140, 255, 255), blue_mask);
                Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
                morphologyEx(blue_mask, blue_mask, MORPH_OPEN, kernel);
                morphologyEx(blue_mask, blue_mask, MORPH_CLOSE, kernel);

                vector<vector<Point>> contours;
                findContours(blue_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                if (!contours.empty())
                {
                    // 1. 找二值化图中面积最大的白色
                    int max_idx = 0;
                    double max_area = 0;
                    for (size_t i = 0; i < contours.size(); i++)
                    {
                        double area = contourArea(contours[i]);
                        if (area > max_area)
                        {
                            max_area = area;
                            max_idx = i;
                        }
                    }

                    if (max_area > 100.0) // 确保有有效区域
                    {
                        // 把最大轮廓单独画在一个干净的 mask 上，彻底无视其他杂点
                        Mat clean_mask = Mat::zeros(blue_mask.size(), CV_8UC1);
                        drawContours(clean_mask, contours, max_idx, Scalar(255), FILLED);

                        Rect bbox = boundingRect(contours[max_idx]);

                        // 2. 删掉右侧的10%，左侧5%
                        int valid_min_x = bbox.x + bbox.width * 0.05f;
                        int valid_max_x = bbox.x + bbox.width * 0.90f;

                        // ==========================================================
                        // 3. 提取边缘并使用霍夫变换寻找最长的上侧直线 (无视局部小突起)
                        // ==========================================================
                        Mat edges;
                        Canny(clean_mask, edges, 50, 150);
                        vector<Vec4i> lines;
                        // 允许线段断裂10像素，最短线段要求大于图像宽度的20%
                        HoughLinesP(edges, lines, 1, CV_PI / 180, 20, clean_mask.cols * 0.2, 10);

                        float max_len = 0;
                        int cy = clean_mask.rows / 2;

                        for (const auto &l : lines)
                        {
                            float dx = l[2] - l[0];
                            float dy = l[3] - l[1];
                            float len = std::sqrt(dx * dx + dy * dy);
                            float mid_y = (l[1] + l[3]) / 2.0f;

                            // 筛选条件：1. 走势为横向(dx > dy)  2. 位于图像上半部分  3. 长度最长
                            if (std::abs(dx) > std::abs(dy) && mid_y < cy && len > max_len)
                            {
                                max_len = len;
                                best_line = l;
                            }
                        }

                        // 4. 将最长线赋值为基准线，并计算角度
                        if (max_len > 0)
                        {
                            // 统一左右顺序：保证 l[0] 放左边，l[2] 放右边，确保角度符号正确
                            if (best_line[0] > best_line[2])
                            {
                                std::swap(best_line[0], best_line[2]);
                                std::swap(best_line[1], best_line[3]);
                            }

                            float angle = atan2(best_line[3] - best_line[1], best_line[2] - best_line[0]) * 180.0f / CV_PI;
                            if (angle > 90.0f)
                                angle -= 180.0f;
                            else if (angle < -90.0f)
                                angle += 180.0f;

                            if (abs(angle) < 45.0f)
                            {
                                custom_tilt_angle = angle;
                            }
                        }
                    }
                }

                if (custom_tilt_angle != -999.0f)
                {
                    g_align91_ref_line = Vec4i(
                        safe_bbox.x + best_line[0], safe_bbox.y + best_line[1],
                        safe_bbox.x + best_line[2], safe_bbox.y + best_line[3]);
                    g_align91_ref_line_valid = true;

                    cout << ">>> [自定义特征] align91 纯净两点连线法: 【成功找到】" << endl;
                    cout << "    -> 位置坐标: 点(" << g_align91_ref_line[0] << ", " << g_align91_ref_line[1]
                         << ") 连 点(" << g_align91_ref_line[2] << ", " << g_align91_ref_line[3] << ")" << endl;
                    cout << "    -> 旋转倾角: " << custom_tilt_angle << " 度" << endl;
                }
                else
                {
                    g_align91_ref_line_valid = false;
                    cout << ">>> [自定义特征] align91 纯净两点连线法: 【未找到】" << endl;
                }
            }

            bool feature_extracted = false;
            if (obj.class_id == 0)
            {
                if (obj.corners_2d.size() == 4)
                    feature_extracted = true;
            }
            else
            {
                Mat roi_frame = raw_frame(safe_bbox);
                std::vector<Point2f> local_corners;
                feature_extracted = (obj.class_id >= 1 && obj.class_id <= 3) ? findWallCorners(roi_frame, local_corners, obj.roi_mask, obj.class_id) : findOrderedCorners(roi_frame, obj.class_id, local_corners, obj.roi_mask);
                if (feature_extracted)
                {
                    std::vector<Point2f> global_corners(4);
                    for (int i = 0; i < 4; i++)
                        global_corners[i] = Point2f(safe_bbox.x + local_corners[i].x, safe_bbox.y + local_corners[i].y);
                    obj.corners_2d = global_corners;
                }
            }

            if (feature_extracted)
            {
                std::vector<Point3f> obj_pts_3d = get3DModelPoints(obj.class_id);
                Mat rvec, tvec;
                if (solvePnP(obj_pts_3d, obj.corners_2d, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
                {
                    // if (obj.class_id == 1) {
                    //     Mat R_obj; cv::Rodrigues(rvec, R_obj);
                    //     double theta_rad = -14.036 * CV_PI / 180.0;
                    //     Mat R_corr = (Mat_<double>(3,3) << cos(theta_rad), -sin(theta_rad), 0, sin(theta_rad),  cos(theta_rad), 0, 0, 0, 1);
                    //     Mat t_corr = (Mat_<double>(3,1) << -2.1, 4.0, 0.0);
                    //     Mat R_new = R_obj * R_corr; Mat t_new = R_obj * t_corr + tvec;
                    //     cv::Rodrigues(R_new, rvec); tvec = t_new;
                    // }
                    obj.tx = tvec.at<double>(0);
                    obj.ty = tvec.at<double>(1);
                    obj.tz = tvec.at<double>(2);
                    Mat R_temp;
                    cv::Rodrigues(rvec, R_temp);
                    double sy_temp = sqrt(R_temp.at<double>(0, 0) * R_temp.at<double>(0, 0) + R_temp.at<double>(1, 0) * R_temp.at<double>(1, 0));
                    obj.rx = atan2(R_temp.at<double>(2, 1), R_temp.at<double>(2, 2)) * 180 / CV_PI;
                    obj.ry = atan2(-R_temp.at<double>(2, 0), sy_temp) * 180 / CV_PI;
                    obj.rz = atan2(R_temp.at<double>(1, 0), R_temp.at<double>(0, 0)) * 180 / CV_PI;

                    obj.has_refined_center = true;
                    std::vector<Point3f> center_3d = {Point3f(0, 0, 0)};
                    std::vector<Point2f> center_2d;
                    projectPoints(center_3d, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS, center_2d);
                    obj.refined_center = center_2d[0];

                    Pose6D arm_target_pose = calibrator.transform(rvec, tvec, current_task.arm_id);
                    arm_target_pose.x /= -10.0;
                    arm_target_pose.y /= -10.0;
                    arm_target_pose.z /= -10.0;
                    arm_target_pose.x += g_arm_x_offset_cm[current_task.arm_id];

                    // ==========================================================
                    // 如果当前是 align91/2/3，则拦截并执行闭环对齐逻辑（这是偷懒，把align对齐功能插到这里来了）
                    // (支持 align91, align92, align93)
                    // ==========================================================
                    if (current_task.raw_cmd == "align91" || current_task.raw_cmd == "align92" || current_task.raw_cmd == "align93")
                    {
                        float target_x = -13.0f; // ★ 你可以按需修改 align91 对齐的X坐标
                        float target_y = -11.0f; // ★ 你可以按需修改 align91 对齐的Y坐标

                        if (current_task.raw_cmd == "align92")
                        {
                            target_x = -14.0f; // ★ 你可以按需修改 align92 对齐的X坐标
                            target_y = -11.0f; // ★ 你可以按需修改 align92 对齐的Y坐标
                        }
                        else if (current_task.raw_cmd == "align93")
                        {
                            target_x = -13.0f; // ★ 你可以按需修改 align93 对齐的X坐标
                            target_y = -14.0f; // ★ 你可以按需修改 align93 对齐的Y坐标
                        }

                        float dx = arm_target_pose.x - target_x;
                        float dy = arm_target_pose.y - target_y;
                        float tilt_angle = 0.0f;

                        if (current_task.raw_cmd == "align91")
                        {
                            tilt_angle = (custom_tilt_angle != -999.0f) ? custom_tilt_angle : obj.rz;
                        }
                        else
                        {
                            // align92 和 align93：直接使用 PNP 四边形下底边 (点3 -> 点2) 的物理斜率
                            // obj.corners_2d 顺序: 0:左上, 1:右上, 2:右下, 3:左下
                            Point2f pt_left = obj.corners_2d[3];
                            Point2f pt_right = obj.corners_2d[2];
                            tilt_angle = atan2(pt_right.y - pt_left.y, pt_right.x - pt_left.x) * 180.0f / CV_PI;
                        }

                        if (tilt_angle > 90.0f)
                            tilt_angle -= 180.0f;
                        else if (tilt_angle < -90.0f)
                            tilt_angle += 180.0f;

                        static float s_align_first_x = 0.0f;
                        static float s_align_first_y = 0.0f;
                        extern bool g_reset_align_memory;

                        if (g_reset_align_memory)
                        {
                            if (std::abs(dx) > 30.0f || std::abs(dy) > 30.0f)
                            {
                                cout << ">>> [视觉闭环防爆] 警告：初始PnP深度异常，拒绝锁定锚点！" << endl;
                            }
                            else
                            {
                                s_align_first_x = arm_target_pose.x;
                                s_align_first_y = arm_target_pose.y;
                                g_reset_align_memory = false;
                                cout << ">>> [视觉闭环] 已锁定 " << current_task.raw_cmd << " 初始物理锚点 -> X:" << s_align_first_x << " Y:" << s_align_first_y << endl;
                            }
                        }

                        cout << "\n>>> [视觉对齐] 触发任务: " << current_task.raw_cmd << " | 边缘倾角:" << tilt_angle << "度" << endl;
                        cout << ">>> [视觉对齐] 当前X:" << arm_target_pose.x << " Y:" << arm_target_pose.y << " | 原始偏差 dX:" << dx << " dY:" << dy << endl;

                        float move_fwd = -dx;
                        float move_right = dy;
                        float turn_a = tilt_angle;

                        std::cout << ">>> [视觉对齐] 解析到底盘动作 -> 需" << (move_fwd >= 0 ? "前进 " : "后退 ") << std::abs(move_fwd)
                                  << " cm | 需" << (move_right >= 0 ? "右移 " : "左移 ") << std::abs(move_right)
                                  << " cm | 需" << (turn_a >= 0 ? "右转 " : "左转 ") << std::abs(turn_a) << " 度" << std::endl;

                        // 复用相同的 2.0 精度阈值
                        if (std::abs(dx) < 1.5f && std::abs(dy) < 1.5f && std::abs(tilt_angle) < 2.0f)
                        {
                            cout << ">>> [视觉对齐] 精度已达标！无需进行底盘调整。" << endl;

                            // 【已禁用】NAV_ADJ 位置更新：align 达标后不再补偿路径规划坐标
                            // if (!g_reset_align_memory) { ... NAV_ADJ ... }

                            extern bool g_wf_align_success;
                            g_wf_align_success = true;
                        }
                        else
                        {
                            cout << ">>> [视觉对齐] 误差超限，下发 ALIGN_MOVE 动作..." << endl;
                            extern int g_serial_fd;
                            if (g_serial_fd >= 0)
                            {
                                char buf[128];
                                sprintf(buf, "ALIGN_MOVE %.1f %.1f %.1f\r\n", dx, dy, tilt_angle);
                                write(g_serial_fd, buf, strlen(buf));
                            }
                        }
                        target_found = true;
                        break; // 彻底拦截，绝对不往下走发 DEMO 指令的代码！
                    }

                    if ((current_task.raw_cmd == "DEMO101" || current_task.raw_cmd == "DEMO102") && obj.corners_2d.size() == 4)
                    {
                        g_cl_state.base_corners_2d = obj.corners_2d;
                        g_cl_state.retry_count = 0;
                    }
                    g_cl_state.last_pose = arm_target_pose;
                    rvec.copyTo(g_cl_state.last_rvec);
                    tvec.copyTo(g_cl_state.last_tvec);
                    g_cl_state.last_obj_center = obj.center;

                    cout << ">>> [坐标转化完成] 发现目标物体 ID: " << obj.class_id << "\n    下发串口指令 -> " << current_task.raw_cmd
                         << " 移动至: X=" << arm_target_pose.x << " Y=" << arm_target_pose.y << " Z=" << arm_target_pose.z << " (厘米)" << endl;

                    // 【新增】：如果是 DEMO091，把框和坐标存下来供闭环使用

                    // 1号点在 sorted corners_2d 中是左下角，索引为 3

                    if (current_task.raw_cmd == "DEMO001")
                    {
                        g_cache_pt1 = obj.corners_2d[3];
                        g_cache_001_px = arm_target_pose.x;
                        g_cache_001_py = arm_target_pose.y;
                        g_cache_001_pz = arm_target_pose.z;
                    }
                    else if (current_task.raw_cmd == "DEMO002")
                    {
                        g_cache_pt1 = obj.corners_2d[3];
                        g_cache_002_px = arm_target_pose.x;
                        g_cache_002_py = arm_target_pose.y;
                        g_cache_002_pz = arm_target_pose.z;
                    }
                    else if (current_task.raw_cmd == "DEMO091")
                    {
                        g_cache_091_bbox = safe_bbox;
                        g_cache_091_px = arm_target_pose.x;
                        g_cache_091_py = arm_target_pose.y;
                        g_cache_091_pz = arm_target_pose.z;
                    }

                    pilot_comm.sendDemoCommand(current_task.raw_cmd, arm_target_pose);
                    target_found = true;
                    break;
                }
            }
        }
    }
    if (!target_found)
    {
        cout << "[Monitor] 视觉检测结束。视野中未找到满足要求的目标 (ID=" << current_task.class_id << ")" << endl;
        
        // 【新增】：如果它是对齐任务且没找到目标，触发恢复机制
        if (current_task.raw_cmd == "align91" || current_task.raw_cmd == "align92" || current_task.raw_cmd == "align93")
        {
            cout << ">>> [视觉对齐] 视野内未发现目标，触发寻找恢复机制！" << endl;
            std::thread([]() {
                extern int g_serial_fd;
                if (g_serial_fd >= 0) {
                    char buf[64];
                    float pan = (g_calibrated_pan > 0) ? g_calibrated_pan : 113.0f;
                    sprintf(buf, "CAM %.1f 30.0\r\n", pan); // 抬起摄像头到30度
                    write(g_serial_fd, buf, strlen(buf));
                }
                usleep(1500000); // 闭眼等待 1.5 秒让摄像头到位
                
                std::lock_guard<std::mutex> lock(g_task_mtx);
                g_demo_task.pending = true;
                g_demo_task.raw_cmd = "ALIGN_RECOVERY"; // 推送特殊单帧任务
            }).detach();
        }
    }
    return target_found; // 告诉调用者有没有找到
}

void VisionEngine::processTask(const DemoTask &task, Mat &raw_frame)
{
    // ================== 新增路由 ==================
    // 【新增】：对齐失败恢复机制
    if (task.raw_cmd == "ALIGN_RECOVERY")
    {
        cout << "\n>>> [对齐恢复] 图像就绪，开始扫描蓝色区域分布..." << endl;
        Mat hsv, mask;
        cvtColor(raw_frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(95, 80, 40), Scalar(140, 255, 255), mask);
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(mask, mask, MORPH_OPEN, kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, kernel);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        float dx = -5.0f; // 默认向前 5cm (根据 ALIGN_MOVE 映射：dx=-5 对应前进 5cm)
        float dy = 0.0f;

        if (!contours.empty())
        {
            double max_area = 0;
            Rect best_rect;
            for (const auto &c : contours)
            {
                double area = contourArea(c);
                if (area > max_area)
                {
                    max_area = area;
                    best_rect = boundingRect(c);
                }
            }

            if (max_area > 500)
            {
                float cx = best_rect.x + best_rect.width / 2.0f;
                if (cx < raw_frame.cols / 3.0f)
                {
                    dx = 0.0f; dy = -5.0f; // 偏左 -> 向左移
                    cout << ">>> [对齐恢复] 蓝色区域在画面左侧，修正：向左移动 5cm" << endl;
                }
                else if (cx > raw_frame.cols * 2.0f / 3.0f)
                {
                    dx = 0.0f; dy = 5.0f;  // 偏右 -> 向右移
                    cout << ">>> [对齐恢复] 蓝色区域在画面右侧，修正：向右移动 5cm" << endl;
                }
                else
                {
                    dx = -5.0f; dy = 0.0f; // 居中 -> 向前移
                    cout << ">>> [对齐恢复] 蓝色区域在画面中央，修正：向前移动 5cm" << endl;
                }
            }
            else
            {
                cout << ">>> [对齐恢复] 蓝色区域太小，默认向前移动 5cm" << endl;
            }
        }
        else
        {
            cout << ">>> [对齐恢复] 画面中未发现蓝色区域，默认向前移动 5cm" << endl;
        }

        extern int g_serial_fd;
        if (g_serial_fd >= 0)
        {
            char buf[128];
            // 恢复摄像头角度为装配记忆角度
            float pan = (g_calibrated_pan > 0) ? g_calibrated_pan : 113.0f;
            float tilt = (g_calibrated_tilt > 0) ? g_calibrated_tilt : 45.0f;
            sprintf(buf, "CAM %.1f %.1f\r\n", pan, tilt);
            write(g_serial_fd, buf, strlen(buf));
            
            // 下发调整位移 (让小车移动，移动完会自动触发 ALIGN_DONE 进行下一轮 auto_align_loop)
            sprintf(buf, "ALIGN_MOVE %.1f %.1f 0.0\r\n", dx, dy);
            write(g_serial_fd, buf, strlen(buf));
        }
        return;
    }

    if (task.raw_cmd == "HSV_FIND_ONESHOT")
    {
        handleHsvFindOneshot(task, raw_frame);
        return;
    }
    if (task.raw_cmd == "CHECK_091")
    {
        handleCheck091(raw_frame);
        return;
    }
    if (task.raw_cmd == "CHECK_001")
    {
        handleCheck001(raw_frame);
        return;
    }
    if (task.raw_cmd == "CHECK_002")
    {
        handleCheck002(raw_frame);
        return;
    }
    if (task.raw_cmd == "CHECK_003")
    {
        handleCheck003(raw_frame);
        return;
    }
    if (task.raw_cmd.rfind("align", 0) == 0)
    {
        if (task.raw_cmd == "align91" || task.raw_cmd == "align92" || task.raw_cmd == "align93")
        {
            DemoTask modified_task = task;
            if (task.raw_cmd == "align91")
            {
                modified_task.class_id = 9;
                modified_task.arm_id = 0;
            }
            else if (task.raw_cmd == "align92")
            {
                modified_task.class_id = 0;
                modified_task.action_id = 1; // 强制复用 DEMO001 的“右侧起手”拓扑推导逻辑
                modified_task.arm_id = 0;
            }
            else if (task.raw_cmd == "align93")
            {
                modified_task.class_id = 0;
                modified_task.action_id = 2; // 强制复用 DEMO002 的“标准起手”拓扑推导逻辑
                modified_task.arm_id = 0;
            }
            handleYoloAndPnP(modified_task, raw_frame);
        }
        else
        {
            handleAlign(task, raw_frame);
        }
        return;
    }
    // 处理巡航寻找逻辑
    if (task.raw_cmd.rfind("FIND_ACK_", 0) == 0)
    {
        cout << "\n>>> [巡航搜索] 云台已就位，开始扫描 ID=2..." << endl;
        // 构造一个虚拟任务，用 YOLO+PnP 流水线
        DemoTask search_task;
        search_task.class_id = 2;             // 锁定寻找 ID=2
        search_task.arm_id = 1;               // 解算基准 ARM1
        search_task.raw_cmd = "CHASSIS_MOVE"; // 占位指令：一旦算好坐标，就把坐标连同此指令发给Pilot
        // 流水线内部会自动把 "CHASSIS_MOVE X Y Z..." 发给 Pilot
        bool found = handleYoloAndPnP(search_task, raw_frame);
        if (!found)
        {
            if (task.raw_cmd == "FIND_ACK_220")
            {
                cout << ">>> [巡航搜索] 视角1 未发现目标 继续搜索..." << endl;
                Pose6D empty_pose{0, 0, 0, 0, 0, 0};
                pilot_comm.sendDemoCommand("DEMO221", empty_pose);
            }
            else if (task.raw_cmd == "FIND_ACK_221")
            {
                cout << ">>> [巡航搜索] 两个视角均未发现 ID=2 搜索终止。" << endl;
                g_wf_find_failed = true;
            }
        }
        return;
    }
    // 拦截小车完成信号，通知线程
    if (task.raw_cmd == "CHASSIS_DONE")
    {
        g_wf_chassis_done = true;
        return;
    }
    if (task.raw_cmd.rfind("CHECK_H", 0) == 0)
    {
        handleClosedLoopCheck(task, raw_frame);
        return;
    }
    if (task.raw_cmd == "FIX_111" || task.raw_cmd == "FIX_131")
    {
        handleSingleAxisServo(task, raw_frame);
        return;
    }
    if (handleBlindOperations(task))
    {
        return;
    }
    handleYoloAndPnP(task, raw_frame);
}

void VisionEngine::processAutoCamera(Mat &raw_frame)
{
    if (!g_auto_cam_running)
        return;
    // 每 8 帧提取一次画面并下发一次指令
    static int throttle_counter = 0;
    if (throttle_counter++ % 8 != 0)
    {
        // 在等待舵机运动的冷却期内，跳过繁重的视觉计算
        putText(raw_frame, "AUTO_CAM WAITING FOR SERVO...", Point(15, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 255), 3);
        return;
    }

    int roi_h = 170;
    Rect roi_rect(0, raw_frame.rows - roi_h, raw_frame.cols, roi_h);
    Mat roi = raw_frame(roi_rect);

    // 灰度与高斯模糊去噪
    Mat gray, blurred, binary;
    cvtColor(roi, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blurred, Size(9, 9), 0);

    // 这个值(60)代表对黑色的敏感度，如果画面里胶带偏灰，可以调大；如果环境很暗，可以调小(如40)
    int black_thresh = 120;
    threshold(blurred, binary, black_thresh, 255, THRESH_BINARY_INV);

    // 填补胶带上可能存在的细小灰尘点或缝隙
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(binary, binary, MORPH_CLOSE, kernel);

    int left_y_sum = 0, left_count = 0, right_y_sum = 0, right_count = 0, center_y_sum = 0, center_count = 0;
    for (int x = 0; x < binary.cols; x += 10)
    {
        for (int y = 0; y < binary.rows; y++)
        {
            if (binary.at<uchar>(y, x) == 255)
            {
                if (x < binary.cols * 0.2)
                {
                    left_y_sum += y;
                    left_count++;
                }
                else if (x > binary.cols * 0.8)
                {
                    right_y_sum += y;
                    right_count++;
                }
                else if (x > binary.cols * 0.4 && x < binary.cols * 0.6)
                {
                    center_y_sum += y;
                    center_count++;
                }
                circle(raw_frame, Point(x, y + roi_rect.y), 2, Scalar(0, 255, 0), -1);
                break;
            }
        }
    }

    if (center_count > 5 && left_count > 5 && right_count > 5)
    {
        float center_y = (float)center_y_sum / center_count;
        float left_y = (float)left_y_sum / left_count;
        float right_y = (float)right_y_sum / right_count;

        float target_dist_from_bottom = 50.0f; // 如果想让车板在画面里更靠下（镜头抬高），把这个值改小，单位：像素
        float target_y = roi_h - target_dist_from_bottom;

        float err_tilt = center_y - target_y;
        float err_pan = left_y - right_y;
        float Kp_tilt = 0.02f, Kp_pan = 0.02f;
        bool tilt_ok = abs(err_tilt) < 18.0f, pan_ok = abs(err_pan) < 18.0f;

        if (tilt_ok && pan_ok)
        {
            g_auto_cam_running = false;
            // 记录下当前完美的基准角度
            g_calibrated_pan = g_cam_pan;
            g_calibrated_tilt = g_cam_tilt;
            std::cout << "\n>>> [自适应云台] 校准完美并已记忆！落点 -> Pan: " << g_calibrated_pan << " Tilt: " << g_calibrated_tilt << std::endl;
        }
        else
        {
            if (!tilt_ok)
                g_cam_tilt += Kp_tilt * err_tilt;
            if (!pan_ok)
                g_cam_pan += Kp_pan * err_pan;
            if (g_cam_pan < 20)
                g_cam_pan = 20;
            if (g_cam_pan > 170)
                g_cam_pan = 170;
            if (g_cam_tilt < 20.0f)
                g_cam_tilt = 20.0f;
            if (g_cam_tilt > 70.0f)
                g_cam_tilt = 70.0f;

            // static int frame_counter = 0;
            // if (frame_counter++ % 4 == 0) {
            //     if (g_serial_fd >= 0) {
            //         char buf[64]; sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt); write(g_serial_fd, buf, strlen(buf));
            //     }
            //     std::cout << "[伺服动态] 俯仰残差: " << err_tilt << " | 偏航残差: " << err_pan << " | 下发 CAM: " << g_cam_pan << " " << g_cam_tilt << std::endl;
            // }

            if (g_serial_fd >= 0)
            {
                char buf[64];
                sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
                write(g_serial_fd, buf, strlen(buf));
            }
            std::cout << "[伺服动态] 俯仰残差: " << err_tilt << " | 偏航残差: " << err_pan
                      << " | 下发 CAM: " << g_cam_pan << " " << g_cam_tilt << std::endl;
        }
    }
    else
    {
        std::cout << "[云台伺服警告] 视野内丢失车板！正在自动向下低头寻找..." << std::endl;
        // 每次找不到时，让俯仰角向下低头 1.0 度 (度数越大越往下)
        g_cam_tilt += 1.0f;
        if (g_cam_tilt > 70.0f)
            g_cam_tilt = 70.0f; // 物理限位保护
        // 同样需要降帧发送，避免指令发得太快导致舵机卡死或过度震荡
        static int search_frame_counter = 0;
        if (search_frame_counter++ % 4 == 0)
        {
            if (g_serial_fd >= 0)
            {
                char buf[64];
                sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
                write(g_serial_fd, buf, strlen(buf));
            }
        }
    }
    putText(raw_frame, "AUTO_CAM ALIGNING (EDGE)...", Point(15, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 165, 255), 3);
    rectangle(raw_frame, roi_rect, Scalar(255, 0, 0), 2);
}

void VisionEngine::processArucoFix(Mat &raw_frame)
{
    if (g_trigger_aruco_fix)
    {
        g_trigger_aruco_fix = false;
        Mat clean_gray_for_aruco;
        cvtColor(raw_frame, clean_gray_for_aruco, COLOR_BGR2GRAY);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        parameters->minMarkerPerimeterRate = 0.01;
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(clean_gray_for_aruco, dictionary, marker_corners, marker_ids, parameters);

        if (!marker_ids.empty())
        {
            std::cout << "\n>>> [ArUco 标定成功] 成功锁定机械爪基准码 (ID: " << marker_ids[0] << ")" << std::endl;
            Point2f p1 = marker_corners[0][0], p2 = marker_corners[0][1], p3 = marker_corners[0][2], p4 = marker_corners[0][3];
            g_fixed_aruco_center = Point2f((p1.x + p2.x + p3.x + p4.x) / 4.0f, (p1.y + p2.y + p3.y + p4.y) / 4.0f);
        }
        else
        {
            std::cout << "\n>>> [ArUco 标定失败] 未找到基准码，请检查画面清晰度或是否被遮挡！" << std::endl;
        }
    }

    if (g_fixed_aruco_center.x >= 0 && g_fixed_aruco_center.y >= 0)
    {
        circle(raw_frame, g_fixed_aruco_center, 5, Scalar(255, 0, 255), -1);
        line(raw_frame, Point(g_fixed_aruco_center.x - 20, g_fixed_aruco_center.y), Point(g_fixed_aruco_center.x + 20, g_fixed_aruco_center.y), Scalar(255, 0, 255), 2);
        line(raw_frame, Point(g_fixed_aruco_center.x, g_fixed_aruco_center.y - 20), Point(g_fixed_aruco_center.x, g_fixed_aruco_center.y + 20), Scalar(255, 0, 255), 2);
    }
}

void VisionEngine::renderOsd(Mat &raw_frame)
{
    if (has_detection && current_yolo_res.detected)
    {
        int bottom_text_y = raw_frame.rows - 20;
        for (auto &obj : current_yolo_res.objects)
        {
            obj.bbox &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

            // 【关键修改】：无视外接框的大小，只要算出了红点（特征点），无条件先画在屏幕上！
            for (const auto &pt : obj.sub_centers)
            {
                circle(raw_frame, pt, 4, Scalar(0, 0, 255), -1);
            }

            // 然后再判断如果目标框本身不成立 (比如降维成了 0x0)，就跳过画外框和坐标文字
            if (obj.bbox.width < 15 || obj.bbox.height < 15)
                continue;

            Scalar color((obj.class_id * 80) % 255, (obj.class_id * 150) % 255, (obj.class_id * 200 + 100) % 255);
            // =======================================================
            // 【新增】：将 AI 预测出的 Mask 以 50% 透明度彩色叠加在主画面上！
            // =======================================================
            if (!obj.ai_mask.empty())
            {
                Mat color_mask = Mat::zeros(raw_frame.size(), raw_frame.type());
                color_mask.setTo(color, obj.ai_mask);
                addWeighted(raw_frame, 1.0, color_mask, 0.5, 0.0, raw_frame);
            }
            rectangle(raw_frame, obj.bbox, color, 2);

            if (obj.has_refined_center && obj.corners_2d.size() == 4)
            {
                Scalar pnp_box_color = Scalar(255, 0, 255);
                for (int i = 0; i < 4; i++)
                    line(raw_frame, obj.corners_2d[i], obj.corners_2d[(i + 1) % 4], pnp_box_color, 2);
                circle(raw_frame, obj.refined_center, 6, Scalar(0, 255, 0), 2);
                line(raw_frame, obj.center, obj.refined_center, Scalar(0, 255, 255), 1);
                char pose_text[256];
                snprintf(pose_text, sizeof(pose_text), "ID:%d P(X:%.1f Y:%.1f D:%.1f)mm | R(Rx:%.1f Ry:%.1f Rz:%.1f)deg", obj.class_id, obj.tx, obj.ty, obj.tz, obj.rx, obj.ry, obj.rz);
                putText(raw_frame, pose_text, Point(15, bottom_text_y), FONT_HERSHEY_SIMPLEX, 0.65, color, 2);
                bottom_text_y -= 30;
            }
            string label = "ID:" + to_string(obj.class_id) + " " + to_string(obj.confidence).substr(0, 4);
            if (obj.class_id != 8) // 放行所有ID标签
            {
                putText(raw_frame, label, Point(obj.bbox.x, max(obj.bbox.y - 5, 10)), FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }

        int pip_offset_y = 10;
        for (const auto &obj : current_yolo_res.objects)
        {
            if (!obj.roi_mask.empty()) // 放行所有ID的二值化图，包括ID=9
            {
                Mat mask_bgr;
                cvtColor(obj.roi_mask, mask_bgr, COLOR_GRAY2BGR);
                Rect pip_rect(10, pip_offset_y, mask_bgr.cols, mask_bgr.rows);
                pip_rect &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

                if (pip_rect.area() > 0)
                {
                    mask_bgr(Rect(0, 0, pip_rect.width, pip_rect.height)).copyTo(raw_frame(pip_rect));
                    rectangle(raw_frame, pip_rect, Scalar(0, 255, 0), 2);
                    string pip_text = "ID:" + to_string(obj.class_id) + " Mask";
                    putText(raw_frame, pip_text, Point(10, pip_rect.y + pip_rect.height + 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
                }
                pip_offset_y += pip_rect.height + 25;

                // ==========================================================
                // 【新增】：每帧持久渲染 align91 找到的最上方旋转基准线
                // ==========================================================
                if (g_align91_ref_line_valid)
                {
                    Point pt1(g_align91_ref_line[0], g_align91_ref_line[1]);
                    Point pt2(g_align91_ref_line[2], g_align91_ref_line[3]);
                    line(raw_frame, pt1, pt2, Scalar(0, 255, 255), 4);
                    putText(raw_frame, "Ref Line", Point(pt1.x, pt1.y - 10), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
                }
            }
        }
    }
}

void VisionEngine::handleHsvFindOneshot(const DemoTask &task, Mat &raw_frame)
{
    cout << ">>> [单帧寻物] 图像就绪，开始 HSV 提取与 PNP (ID=" << task.class_id << ")..." << endl;

    Mat hsv, mask;
    cvtColor(raw_frame, hsv, COLOR_BGR2HSV);

    // 提取蓝色 HSV 范围
    inRange(hsv, Scalar(95, 80, 40), Scalar(140, 255, 255), mask);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int best_idx = -1;
    float min_dist = 1e9;
    Point2f img_center(raw_frame.cols / 2.0f, raw_frame.rows / 2.0f);

    for (size_t i = 0; i < contours.size(); i++)
    {
        if (contourArea(contours[i]) < 400)
            continue; // 过滤极小噪点
        Rect rect = boundingRect(contours[i]);
        Point2f center(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f);

        // 在图传画面中用红点点出中心
        circle(raw_frame, center, 6, Scalar(0, 0, 255), -1);

        // 选中最接近画面中心的那一个
        float dist = norm(center - img_center);
        if (dist < min_dist)
        {
            min_dist = dist;
            best_idx = i;
        }
    }

    if (best_idx != -1)
    {
        // 回退到使用最小外接矩形 (RotatedRect)，抗远距离模糊能力更强
        RotatedRect rrect = minAreaRect(contours[best_idx]);
        Point2f pts[4];
        rrect.points(pts);

        // 矩形四角排序以适配 3D 模型的物理点位顺序 (左上、右上、右下、左下)
        std::vector<Point2f> corners(pts, pts + 4);
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

        // ==========================================================
        // 【核心修正】：削平 ID=1~4 顶部的“犄角”，下压 10% 还原真实物理高度
        // ==========================================================
        if (task.class_id >= 1 && task.class_id <= 4)
        {
            float height_left = bot[0].y - top[0].y;
            float height_right = bot[1].y - top[1].y;

            // 将顶部的两个角点向下平移自身高度的 10%
            top[0].y += height_left * 0.10f;
            top[1].y += height_right * 0.10f;

            cout << ">>> [特征修正] ID=" << task.class_id << " 发现顶部突起，角点已下压 10% 还原主体尺寸！" << endl;
        }

        corners = {top[0], top[1], bot[1], bot[0]};

        // 【UI】画出高精度提取的真实角点连线 (特意换成紫色 255,0,255，便于你在图传里核对)
        for (int i = 0; i < 4; i++)
        {
            line(raw_frame, corners[i], corners[(i + 1) % 4], Scalar(255, 0, 255), 2);
        }

        // 获取对应的 3D std::vector<Point2f> corners(pts物理尺寸并进行 PnP 位姿解算
        std::vector<Point3f> obj_pts_3d = get3DModelPoints(task.class_id);
        Mat rvec, tvec;
        if (solvePnP(obj_pts_3d, corners, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
        {

            // --- 补偿镜头抬头 15 度带来的坐标系偏差 ---
            double theta = 15.0 * CV_PI / 180.0;
            Mat R_pitch_comp = (Mat_<double>(3, 3) << 1, 0, 0,
                                0, cos(theta), -sin(theta),
                                0, sin(theta), cos(theta));

            Mat R_cam;
            cv::Rodrigues(rvec, R_cam);
            R_cam = R_pitch_comp * R_cam;
            tvec = R_pitch_comp * tvec;
            cv::Rodrigues(R_cam, rvec);
            // ------------------------------------------

            // 空间转换到 ARM1 坐标系下
            Pose6D arm1_pose = calibrator.transform(rvec, tvec, 1);
            arm1_pose.x /= -10.0;
            arm1_pose.y /= -10.0;
            arm1_pose.z /= -10.0;
            arm1_pose.x += g_arm_x_offset_cm[1]; // 专属 ARM1

            float px = arm1_pose.x;
            float py = arm1_pose.y;
            cout << ">>> [单帧寻物] HSV 外框 PnP 解析成功 | 位于 ARM1_X: " << px << " cm, ARM1_Y: " << py << " cm" << endl;

            // ===== 底盘移动核心解算逻辑 =====
            float target_x = -16.0f;
            float target_y = 8.0f;
            float forward_cm = target_x - px;
            float right_cm = py - target_y;

            cout << ">>> [底盘调度] 需前进 " << forward_cm << " cm, 需向右 " << right_cm << " cm" << endl;

            if (g_serial_fd >= 0)
            {
                char buf[128];
                // Y轴补偿：前后移动
                if (abs(forward_cm) > 0.5f)
                {
                    if (forward_cm > 0)
                        sprintf(buf, "MW %.1f\r\n", forward_cm);
                    else
                        sprintf(buf, "MS %.1f\r\n", -forward_cm);
                    write(g_serial_fd, buf, strlen(buf));
                    usleep(30000); // 防串口粘包
                }
                // X轴补偿：左右平移
                if (abs(right_cm) > 0.5f)
                {
                    if (right_cm > 0)
                        sprintf(buf, "MD %.1f\r\n", right_cm);
                    else
                        sprintf(buf, "MA %.1f\r\n", -right_cm);
                    write(g_serial_fd, buf, strlen(buf));
                }
            }
        }
        else
        {
            cout << ">>> [单帧寻物] PnP 矩阵收敛失败！" << endl;
        }
    }
    else
    {
        cout << ">>> [单帧寻物] 致命：画面中心周围完全没有蓝色目标！" << endl;
    }

    // 无论最终结果如何，看完最后一眼立刻把云台归位到 Nod 的位置
    if (g_serial_fd >= 0 && g_calibrated_pan >= 0)
    {
        char buf[64];
        sprintf(buf, "CAM %.1f %.1f\r\n", g_calibrated_pan, g_calibrated_tilt);
        write(g_serial_fd, buf, strlen(buf));
        cout << ">>> [单帧寻物] 流程收尾：云台已复位至 Nod (Pan:" << g_calibrated_pan << ", Tilt:" << g_calibrated_tilt << ")" << endl;
    }
}

void VisionEngine::handleCheck091(Mat &raw_frame)
{
    if (g_cache_091_bbox.area() <= 0)
    {
        cout << ">>> [视觉闭环] 无效的 DEMO091 缓存框！" << endl;
        return;
    }

    // ==========================================================
    // 【修改 1】：截取最右侧 1/3 区域，并在宽度上额外向右侧外扩 80 个像素
    // ==========================================================
    Rect right_roi = g_cache_091_bbox;
    right_roi.x = right_roi.x + right_roi.width * 2 / 3;
    right_roi.width = (right_roi.width / 3) + 80;

    // 依然保留安全裁剪，防止这多出来的 80 像素超出了图像真实边界导致程序崩溃
    right_roi &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

    if (right_roi.area() <= 0)
        return;

    Mat roi = raw_frame(right_roi);

    // ==========================================================
    // 2. 进阶版：HSV 饱和度与明度双通道融合边缘提取
    // ==========================================================
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);

    // 将 HSV 图像分离为 H, S, V 三个独立的单通道图像
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);

    Mat edges_s, edges_v, edges;

    // 对饱和度 (S - hsv_channels[1]) 提取边缘：对颜色边界极度敏感
    Canny(hsv_channels[1], edges_s, 10, 30);

    // 对明度 (V - hsv_channels[2]) 提取边缘：对物理阴影和缝隙极度敏感
    Canny(hsv_channels[2], edges_v, 10, 30);

    // 融合两种边缘：只要颜色突变或亮度突变，统统作为有效边缘！
    bitwise_or(edges_s, edges_v, edges);

    // 3. 霍夫直线变换
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 20, right_roi.height * 0.6, 10);

    // 4. 筛选并聚类竖直长线 (防止同一条粗边被识别成好几条线)
    std::vector<int> valid_x_centers;

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        float angle = atan2(abs(l[3] - l[1]), abs(l[2] - l[0])) * 180.0 / CV_PI;
        float length = norm(Point(l[0], l[1]) - Point(l[2], l[3]));

        // 判定条件：角度在 75~105 度之间，长度 >= ROI 高度的 60%
        if (angle > 75.0 && angle < 105.0 && length >= right_roi.height * 0.6)
        {
            int cx = (l[0] + l[2]) / 2;
            bool merged = false;
            // X轴聚类：相距 10 像素以内的线段认为是同一条边
            for (int &existing_x : valid_x_centers)
            {
                if (abs(cx - existing_x) < 10)
                {
                    merged = true;
                    break;
                }
            }
            if (!merged)
            {
                valid_x_centers.push_back(cx);
                // 【UI】在画面上画出识别到的长红线
                line(raw_frame, Point(right_roi.x + l[0], right_roi.y + l[1]),
                     Point(right_roi.x + l[2], right_roi.y + l[3]), Scalar(0, 0, 255), 3);
            }
        }
    }

    int vertical_line_count = valid_x_centers.size();
    cout << ">>> [视觉闭环] 扫描右侧边缘，发现独立竖直长线数量: " << vertical_line_count << endl;

    // 【UI】画出扫描区域的蓝框 (可以看到这比原来的 1/3 框多出了 50 像素)
    rectangle(raw_frame, right_roi, Scalar(255, 255, 0), 2);

    // 5. 闭环决策分发
    if (vertical_line_count >= 2)
    {
        g_cache_091_px += 0.5f; // 参数X加0.5
        Pose6D adj_pose = {g_cache_091_px, g_cache_091_py, g_cache_091_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] 未卡平 (竖线≥2)！下发微调指令 DEMO092 (X=" << g_cache_091_px << ")" << endl;
        pilot_comm.sendDemoCommand("DEMO092", adj_pose);
    }
    else
    {
        Pose6D adj_pose = {g_cache_091_px, g_cache_091_py, g_cache_091_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] 卡紧完毕 (竖线<2)！触发装配收尾指令 DEMO093" << endl;
        pilot_comm.sendDemoCommand("DEMO093", adj_pose);
    }
}

void VisionEngine::handleCheck001(Mat &raw_frame)
{
    if (g_cache_pt1.x < 0)
    {
        cout << ">>> [视觉闭环] 缺少 DEMO001 的有效1号锚点！" << endl;
        return;
    }

    // 注意：OpenCV 坐标系 Y 轴向下，向上推就是减法
    // 以 1号点 为基准，向左 50，向右 150（总宽200），向上 320
    Rect roi_rect(g_cache_pt1.x - 50, g_cache_pt1.y - 320, 200, 320);
    roi_rect &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

    if (roi_rect.area() <= 0)
        return;

    Mat roi = raw_frame(roi_rect);
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);

    Mat edges_s, edges_v, edges;
    Canny(hsv_channels[1], edges_s, 12, 33);
    Canny(hsv_channels[2], edges_v, 12, 33);
    bitwise_or(edges_s, edges_v, edges);

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 20, roi_rect.height * 0.6, 10);
    std::vector<int> valid_x_centers;

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        float angle = atan2(abs(l[3] - l[1]), abs(l[2] - l[0])) * 180.0 / CV_PI;
        float length = norm(Point(l[0], l[1]) - Point(l[2], l[3]));

        if (angle > 75.0 && angle < 105.0 && length >= roi_rect.height * 0.6)
        {
            int cx = (l[0] + l[2]) / 2;
            bool merged = false;
            for (int &existing_x : valid_x_centers)
            {
                if (abs(cx - existing_x) < 25)
                {
                    merged = true;
                    break;
                }
            }
            if (!merged)
            {
                valid_x_centers.push_back(cx);
                line(raw_frame, Point(roi_rect.x + l[0], roi_rect.y + l[1]),
                     Point(roi_rect.x + l[2], roi_rect.y + l[3]), Scalar(0, 0, 255), 3);
            }
        }
    }

    int vertical_line_count = valid_x_centers.size();
    rectangle(raw_frame, roi_rect, Scalar(255, 255, 0), 2);
    cout << ">>> [视觉闭环] DEMO001 侧边扫描，发现独立竖直长线数量: " << vertical_line_count << endl;

    // 临界值为 2
    if (vertical_line_count >= 2)
    {

        g_cache_001_px -= 0.5f;

        Pose6D adj_pose = {g_cache_001_px, g_cache_001_py, g_cache_001_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] DEMO001 未卡平！下发微调指令 DEMO001_ADJ (X=" << g_cache_001_px << ")" << endl;
        pilot_comm.sendDemoCommand("DEMO001_ADJ", adj_pose);
    }
    else
    {
        Pose6D adj_pose = {g_cache_001_px, g_cache_001_py, g_cache_001_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] DEMO001 卡紧完毕！触发收尾指令 DEMO001_DONE" << endl;
        pilot_comm.sendDemoCommand("DEMO001_DONE", adj_pose);
    }
}

void VisionEngine::handleCheck002(Mat &raw_frame)
{
    if (g_cache_pt1.x < 0)
    {
        cout << ">>> [视觉闭环] 缺少 DEMO002 的有效1号锚点！" << endl;
        return;
    }

    // 以 1号点 为下侧中点，向左右各 75，向上 200 像素
    Rect roi_rect(g_cache_pt1.x - 30, g_cache_pt1.y - 250, 90, 200);
    roi_rect &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

    if (roi_rect.area() <= 0)
        return;

    Mat roi = raw_frame(roi_rect);
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);

    Mat edges_s, edges_v, edges;
    Canny(hsv_channels[1], edges_s, 10, 31);
    Canny(hsv_channels[2], edges_v, 10, 31);
    bitwise_or(edges_s, edges_v, edges);

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 20, roi_rect.height * 0.6, 10);
    std::vector<int> valid_x_centers;

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        float angle = atan2(abs(l[3] - l[1]), abs(l[2] - l[0])) * 180.0 / CV_PI;
        float length = norm(Point(l[0], l[1]) - Point(l[2], l[3]));

        if (angle > 75.0 && angle < 105.0 && length >= roi_rect.height * 0.6)
        {
            int cx = (l[0] + l[2]) / 2;
            bool merged = false;
            for (int &existing_x : valid_x_centers)
            {
                if (abs(cx - existing_x) < 25)
                {
                    merged = true;
                    break;
                }
            }
            if (!merged)
            {
                valid_x_centers.push_back(cx);
                line(raw_frame, Point(roi_rect.x + l[0], roi_rect.y + l[1]),
                     Point(roi_rect.x + l[2], roi_rect.y + l[3]), Scalar(0, 0, 255), 3);
            }
        }
    }

    int vertical_line_count = valid_x_centers.size();
    rectangle(raw_frame, roi_rect, Scalar(255, 255, 0), 2);
    cout << ">>> [视觉闭环] DEMO002 侧边扫描，发现独立竖直长线数量: " << vertical_line_count << endl;

    // 临界值为 3 (即允许出现0、1、2条，达到3条即判为未卡平) // 还是2吧
    if (vertical_line_count >= 2)
    {

        g_cache_002_py += 0.5f;

        Pose6D adj_pose = {g_cache_002_px, g_cache_002_py, g_cache_002_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] DEMO002 未卡平 (竖线≥3)！下发微调指令 DEMO002_ADJ (X=" << g_cache_002_px << ")" << endl;
        pilot_comm.sendDemoCommand("DEMO002_ADJ", adj_pose);
    }
    else
    {
        Pose6D adj_pose = {g_cache_002_px, g_cache_002_py, g_cache_002_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] DEMO002 卡紧完毕！触发收尾指令 DEMO002_DONE" << endl;
        pilot_comm.sendDemoCommand("DEMO002_DONE", adj_pose);
    }
}
void VisionEngine::handleCheck003(Mat &raw_frame)
{
    if (g_cache_pt1.x < 0)
    {
        cout << ">>> [视觉闭环] 缺少 DEMO003 的有效1号锚点！" << endl;
        return;
    }

    // 和 CHECK_001 相同的框选逻辑：向左50，向右150，向上320
    Rect roi_rect(g_cache_pt1.x - 50, g_cache_pt1.y - 230, 100, 230);
    roi_rect &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

    if (roi_rect.area() <= 0)
        return;

    Mat roi = raw_frame(roi_rect);
    Mat hsv;
    cvtColor(roi, hsv, COLOR_BGR2HSV);
    vector<Mat> hsv_channels;
    split(hsv, hsv_channels);

    Mat edges_s, edges_v, edges;
    Canny(hsv_channels[1], edges_s, 8, 27);
    Canny(hsv_channels[2], edges_v, 8, 27);
    bitwise_or(edges_s, edges_v, edges);

    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 20, roi_rect.height * 0.6, 10);
    std::vector<int> valid_x_centers;

    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        float angle = atan2(abs(l[3] - l[1]), abs(l[2] - l[0])) * 180.0 / CV_PI;
        float length = norm(Point(l[0], l[1]) - Point(l[2], l[3]));

        if (angle > 75.0 && angle < 105.0 && length >= roi_rect.height * 0.6)
        {
            int cx = (l[0] + l[2]) / 2;
            bool merged = false;
            for (int &existing_x : valid_x_centers)
            {
                if (abs(cx - existing_x) < 10)
                {
                    merged = true;
                    break;
                }
            }
            if (!merged)
            {
                valid_x_centers.push_back(cx);
                line(raw_frame, Point(roi_rect.x + l[0], roi_rect.y + l[1]),
                     Point(roi_rect.x + l[2], roi_rect.y + l[3]), Scalar(0, 0, 255), 3);
            }
        }
    }

    int vertical_line_count = valid_x_centers.size();
    rectangle(raw_frame, roi_rect, Scalar(255, 255, 0), 2);
    cout << ">>> [视觉闭环] DEMO003 侧边扫描，发现独立竖直长线数量: " << vertical_line_count << endl;

    // 临界值为 2。
    // 注意：复用 g_cache_002 坐标进行微调，因为 003 检测的物体实际上和 002 阶段是同一个位置。
    if (vertical_line_count >= 2)
    {
        g_cache_002_px -= 0.5f;
        Pose6D adj_pose = {g_cache_002_px, g_cache_002_py, g_cache_002_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] DEMO003 未卡平！下发微调指令 DEMO003_ADJ (X=" << g_cache_002_px << ")" << endl;
        pilot_comm.sendDemoCommand("DEMO003_ADJ", adj_pose);
    }
    else
    {
        Pose6D adj_pose = {g_cache_002_px, g_cache_002_py, g_cache_002_pz, 0, 0, 0};
        cout << ">>> [视觉闭环] DEMO003 卡紧完毕！触发收尾指令 DEMO003_DONE" << endl;
        pilot_comm.sendDemoCommand("DEMO003_DONE", adj_pose);
    }
}

void VisionEngine::handleAlign(const DemoTask &task, Mat &raw_frame)
{
    cout << "\n>>> [视觉对齐] 触发闭环 Align 引擎: " << task.raw_cmd << endl;

    // 1. 解析目标参数
    int arm_id = 0, class_id = 0;
    float target_x = 0.0f, target_y = 0.0f;

    if (task.raw_cmd == "align01")
    {
        arm_id = 1;
        class_id = 3;
        target_x = -15.0f;
        target_y = 10.0f;
    }
    else if (task.raw_cmd == "align02")
    {
        arm_id = 0;
        class_id = 0;
        target_x = -15.0f;
        target_y = -8.0f;
    }
    else if (task.raw_cmd == "align03")
    {
        arm_id = 0;
        class_id = 2;
        target_x = -18.0f;
        target_y = -4.0f;
    }
    else if (task.raw_cmd == "align04")
    {
        arm_id = 1;
        class_id = 1; // 对应 demo111 的 ID=1
        target_x = -15.0f;
        target_y = 8.0f;
    }
    else
    {
        cout << ">>> [错误] 未知的 align 指令！" << endl;
        return;
    }

    // 2. 纯 HSV 蓝色提取框选
    Mat hsv, mask;
    cvtColor(raw_frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(95, 80, 40), Scalar(140, 255, 255), mask);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // ==========================================================
    // 【修改】：支持阵列方向性的智能目标锁定算法 (左/中/右切换)
    // ==========================================================
    static float s_last_align_move_right = 0.0f; // 静态记忆变量，记录上次横移方向

    // 针对 align04 (ID=1) 面积较小，将面积过滤阈值降至 3000，其他任务保持 10000 (像素)
    float min_area_thresh = (task.raw_cmd == "align04") ? 3000.0f : 10000.0f;

    vector<int> valid_indices;
    float max_area = 0.0f;
    for (size_t i = 0; i < contours.size(); i++)
    {
        float area = contourArea(contours[i]);
        // 【核心优化 1】：动态阈值过滤，无视微小噪点
        if (area < min_area_thresh)
            continue;
        valid_indices.push_back(i);
        if (area > max_area)
            max_area = area; // 找到画面中最大的蓝色面积
    }

    int best_idx = -1;
    if (!valid_indices.empty())
    {

        // 【核心优化 2】：align02 专属策略，直接霸道锁定全场最大蓝色块！
        if (task.raw_cmd == "align02")
        {
            for (int idx : valid_indices)
            {
                if (contourArea(contours[idx]) >= max_area - 1.0f)
                {
                    best_idx = idx;
                    break;
                }
            }
            cout << ">>> [视觉对齐] 策略激活: align02 专属，直接锁定画面中最大面积的蓝色区域！" << endl;
        }
        else
        {
            // 筛选候选：面积 ≥ 最大面积的 60%
            std::vector<int> cand;
            for (int idx : valid_indices)
            {
                if (contourArea(contours[idx]) >= max_area * 0.6f)
                    cand.push_back(idx);
            }
            if (cand.empty())
                cand = valid_indices; // 保底

            if (task.raw_cmd == "align01" || task.raw_cmd == "align04")
            {
                // 阵列从左到右排列 → 锁最左侧
                float min_x = 1e9;
                for (int idx : cand)
                {
                    Rect r = boundingRect(contours[idx]);
                    float cx = r.x + r.width / 2.0f;
                    if (cx < min_x)
                    {
                        min_x = cx;
                        best_idx = idx;
                    }
                }
                cout << ">>> [视觉对齐] align01 锁定最左侧蓝色目标" << endl;
            }
            else if (task.raw_cmd == "align03")
            {
                // 阵列从左到右排列 → 锁最右侧
                float max_x = -1e9;
                for (int idx : cand)
                {
                    Rect r = boundingRect(contours[idx]);
                    float cx = r.x + r.width / 2.0f;
                    if (cx > max_x)
                    {
                        max_x = cx;
                        best_idx = idx;
                    }
                }
                cout << ">>> [视觉对齐] align03 锁定最右侧蓝色目标" << endl;
            }
            else
            {
                // 其他情况（含 align02）：锁定离画面中心最近的
                float min_dist = 1e9;
                Point2f img_center(raw_frame.cols / 2.0f, raw_frame.rows / 2.0f);
                for (int idx : cand)
                {
                    Rect r = boundingRect(contours[idx]);
                    Point2f c(r.x + r.width / 2.0f, r.y + r.height / 2.0f);
                    float d = norm(c - img_center);
                    if (d < min_dist)
                    {
                        min_dist = d;
                        best_idx = idx;
                    }
                }
                cout << ">>> [视觉对齐] 默认锁定画面中心最近目标" << endl;
            }
        }
    }

    if (best_idx == -1)
    {
        cout << ">>> [视觉对齐] 视野内未发现满足条件的蓝色物体，触发寻找恢复机制！" << endl;
        s_last_align_move_right = 0.0f; // 发生丢失时清空记忆

        // 【新增】：触发相机抬起并切入恢复模式
        std::thread([]() {
            extern int g_serial_fd;
            if (g_serial_fd >= 0) {
                char buf[64];
                float pan = (g_calibrated_pan > 0) ? g_calibrated_pan : 113.0f;
                sprintf(buf, "CAM %.1f 30.0\r\n", pan); // 抬起摄像头到30度
                write(g_serial_fd, buf, strlen(buf));
            }
            usleep(1500000); // 闭眼等待 1.5 秒
            
            std::lock_guard<std::mutex> lock(g_task_mtx);
            g_demo_task.pending = true;
            g_demo_task.raw_cmd = "ALIGN_RECOVERY";
        }).detach();

        return;
    }

    // ==========================================================
    // 【UI 可视化】：在图传画面中实时绘制候选框与最终锁定框
    // ==========================================================
    for (int idx : valid_indices)
    {
        Rect r = boundingRect(contours[idx]);
        // 所有满足 10000 面积的蓝色块，用黄色细框标出
        rectangle(raw_frame, r, Scalar(0, 255, 255), 2);
    }

    Rect bbox = boundingRect(contours[best_idx]);
    // 系统最终决定锁定的目标，用红色粗框标出，并配上文字
    rectangle(raw_frame, bbox, Scalar(0, 0, 255), 4);
    putText(raw_frame, "ALIGN TARGET", Point(bbox.x, max(bbox.y - 10, 10)), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
    // ==========================================================

    // 将外围框四周稍微放大 30 像素，防止切掉边缘检测线和角点
    int expand_px = 30;
    bbox.x -= expand_px;
    bbox.y -= expand_px;
    bbox.width += expand_px * 2;
    bbox.height += expand_px * 2;

    Rect safe_bbox = bbox & Rect(0, 0, raw_frame.cols, raw_frame.rows);
    if (safe_bbox.area() <= 0)
        return;

    extern cv::Rect g_last_locked_bbox;
    extern int g_last_locked_class_id;
    g_last_locked_bbox = safe_bbox;
    g_last_locked_class_id = class_id;
    cout << ">>> [视觉对齐] 已将该目标框写入 YOLO 跨帧记忆池中！" << endl;

    std::vector<Point2f> final_corners;
    float tilt_angle = 0.0f;

    // ==========================================================
    // 3 & 4. 角点提取与倾角计算 (按 ID 分支)
    // ==========================================================
    if (class_id == 0)
    {
        // --- ID=0: 局部送入 YOLO NEXT 提取 12 点，提取四大外围极值 ---
        Mat roi_frame = raw_frame(safe_bbox);
        std::vector<Point2f> raw_centers = runNextYoloInferenceRaw(roi_frame);
        std::vector<Point2f> global_raw;
        for (const auto &pt : raw_centers)
        {
            global_raw.push_back(Point2f(pt.x + safe_bbox.x, pt.y + safe_bbox.y));
        }

        std::vector<Point2f> pts = clusterPoints(global_raw, 12.0f);

        if (pts.size() < 4)
        {
            cout << ">>> [视觉对齐] ID=0 提取角点不足！" << endl;
            return;
        }

        // ID=0 无遮挡，直接通过算术极值提取四大外围角点
        Point2f P1 = pts[0], P4 = pts[0], P7 = pts[0], P10 = pts[0];
        float min_x_minus_y = 1e9, max_x_plus_y = -1e9, max_x_minus_y = -1e9, min_x_plus_y = 1e9;
        for (auto p : pts)
        {
            if (p.x - p.y < min_x_minus_y)
            {
                min_x_minus_y = p.x - p.y;
                P1 = p;
            }
            if (p.x + p.y > max_x_plus_y)
            {
                max_x_plus_y = p.x + p.y;
                P4 = p;
            }
            if (p.x - p.y > max_x_minus_y)
            {
                max_x_minus_y = p.x - p.y;
                P7 = p;
            }
            if (p.x + p.y < min_x_plus_y)
            {
                min_x_plus_y = p.x + p.y;
                P10 = p;
            }
        }

        std::vector<Point2f> corners = {P10, P7, P4, P1};
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
        final_corners = {top[0], top[1], bot[1], bot[0]};

        // ID=0 的倾角：直接利用 PnP 框的完美底边 (bot[0] 到 bot[1]) 计算
        tilt_angle = atan2(bot[1].y - bot[0].y, bot[1].x - bot[0].x) * 180.0 / CV_PI;
    }
    else
    {
        // --- ID=2,3: 纯 HSV 提取轮廓，并复用 DEMO 级别的精准角点提取 ---
        Mat roi_frame = raw_frame(safe_bbox);
        Mat roi_mask = mask(safe_bbox); // 直接使用刚才二值化好的 HSV 掩码！

        std::vector<Point2f> local_corners;
        bool feature_extracted = (class_id >= 1 && class_id <= 3) ? findWallCorners(roi_frame, local_corners, roi_mask, class_id) : findOrderedCorners(roi_frame, class_id, local_corners, roi_mask);

        if (feature_extracted && local_corners.size() == 4)
        {
            final_corners.clear();
            for (int i = 0; i < 4; i++)
            {
                final_corners.push_back(Point2f(safe_bbox.x + local_corners[i].x, safe_bbox.y + local_corners[i].y));
            }
            cout << ">>> [视觉对齐] ID=" << class_id << " 成功复用精准角点提取！" << endl;
        }
        else
        {
            // 兜底方案：如果精准提取失败，退回最小外接矩形
            RotatedRect rrect = minAreaRect(contours[best_idx]);
            Point2f pts[4];
            rrect.points(pts);
            std::vector<Point2f> corners(pts, pts + 4);
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
            final_corners = {top[0], top[1], bot[1], bot[0]};
            cout << ">>> [视觉对齐] 警告：精准角点提取失败，退回 minAreaRect 兜底！" << endl;
        }

        // --- 计算倾角 (保留原有的高精度双通道边缘逻辑) ---
        Rect bot_roi = safe_bbox;
        bot_roi.y = bot_roi.y + bot_roi.height * 2 / 3;
        bot_roi.height = bot_roi.height / 3;
        bot_roi &= Rect(0, 0, raw_frame.cols, raw_frame.rows);

        if (bot_roi.area() > 0)
        {
            Mat roi_hsv;
            cvtColor(raw_frame(bot_roi), roi_hsv, COLOR_BGR2HSV);
            vector<Mat> hsv_channels;
            split(roi_hsv, hsv_channels);

            Mat edges_s, edges_v, edges;
            Canny(hsv_channels[1], edges_s, 10, 30);
            Canny(hsv_channels[2], edges_v, 10, 30);
            bitwise_or(edges_s, edges_v, edges);

            vector<Vec4i> lines;
            HoughLinesP(edges, lines, 1, CV_PI / 180, 15, bot_roi.width * 0.4, 10);

            if (!lines.empty())
            {
                float sum_angle = 0;
                int count = 0;
                for (auto &l : lines)
                {
                    float a = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;
                    if (abs(a) < 45.0f || abs(a) > 135.0f)
                    {
                        sum_angle += a;
                        count++;
                    }
                }
                if (count > 0)
                    tilt_angle = sum_angle / count;
            }
        }
    }

    // 格式化角度到锐角相对偏差 (-90 到 90)
    if (tilt_angle > 90.0f)
        tilt_angle -= 180.0f;
    else if (tilt_angle < -90.0f)
        tilt_angle += 180.0f;

    // ==========================================================
    // 5. PnP 解算
    // ==========================================================
    std::vector<Point3f> obj_pts_3d = get3DModelPoints(class_id);
    Mat rvec, tvec;
    if (solvePnP(obj_pts_3d, final_corners, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
    {

        // 转换至对应的机械臂物理坐标系
        Pose6D arm_pose = calibrator.transform(rvec, tvec, arm_id);
        arm_pose.x /= -10.0;
        arm_pose.y /= -10.0;
        arm_pose.z /= -10.0;
        arm_pose.x += g_arm_x_offset_cm[arm_id];

        // 提前计算偏差，用于防爆过滤
        float dx = arm_pose.x - target_x;
        float dy = arm_pose.y - target_y;

        // ==========================================================
        // 【防爆修复】：记录初次 PnP 坐标，加入常识过滤器！
        // ==========================================================
        static float s_align_first_x = 0.0f;
        static float s_align_first_y = 0.0f;
        extern bool g_reset_align_memory;
        if (g_reset_align_memory)
        {
            // 如果算出距目标超过 30cm，绝对是兜底矩形或运动模糊导致的透视畸变！
            if (std::abs(dx) > 20.0f || std::abs(dy) > 20.0f)
            {
                cout << ">>> [视觉闭环防爆] 警告：初始PnP深度异常 (X:" << arm_pose.x << " Y:" << arm_pose.y << ")，判定为噪点，拒绝锁定锚点！" << endl;
            }
            else
            {
                s_align_first_x = arm_pose.x;
                s_align_first_y = arm_pose.y;
                g_reset_align_memory = false;
                cout << ">>> [视觉闭环] 已锁定初始物理锚点 -> 机械臂前(X):" << s_align_first_x << " cm, 右(Y):" << s_align_first_y << " cm" << endl;
            }
        }

        cout << "\n>>> [视觉对齐] 目标 ID:" << class_id << " | 边缘倾角:" << tilt_angle << "度" << endl;
        cout << ">>> [视觉对齐] 当前X:" << arm_pose.x << " Y:" << arm_pose.y << " | 原始偏差 dX:" << dx << " dY:" << dy << endl;

        // 【最严谨轴向映射】：小车(右+X, 前+Y) vs 机械臂(前-X, 右+Y)
        float move_fwd = -dx;
        float move_right = dy;
        float turn_a = tilt_angle;

        std::cout << ">>> [视觉对齐] 解析到底盘动作 -> "
                  << "需" << (move_fwd >= 0 ? "前进 " : "后退 ") << std::abs(move_fwd) << " cm | "
                  << "需" << (move_right >= 0 ? "右移 " : "左移 ") << std::abs(move_right) << " cm | "
                  << "需" << (turn_a >= 0 ? "右转 " : "左转 ") << std::abs(turn_a) << " 度"
                  << std::endl;

        // 误差阈值 (align02 放宽 dy 和 tilt)
        float th_dx = (task.raw_cmd == "align02") ? 3.0f : 3.0f, th_dy = (task.raw_cmd == "align02") ? 3.0f : 2.0f, th_tilt = (task.raw_cmd == "align02") ? 40.0f : 3.0f;
        if (std::abs(dx) < th_dx && std::abs(dy) < th_dy && std::abs(tilt_angle) < th_tilt)
        {
            cout << ">>> [视觉对齐] 精度已达标！无需进行底盘调整。" << endl;

            // 【已禁用】NAV_ADJ 位置更新：align 达标后不再补偿路径规划坐标
            // if (!g_reset_align_memory) { ... NAV_ADJ ... }

            s_last_align_move_right = 0.0f;
            extern bool g_wf_align_success;
            g_wf_align_success = true;
        }
        else
        {
            if (task.raw_cmd == "align01" || task.raw_cmd == "align02" || task.raw_cmd == "align03" || task.raw_cmd == "align04")
            {
                s_last_align_move_right = move_right;
            }

            cout << ">>> [视觉对齐] 误差超限，下发 ALIGN_MOVE 动作..." << endl;
            if (g_serial_fd >= 0)
            {
                // 角度已达标则不传给底盘做无谓旋转
                float send_tilt = (std::abs(tilt_angle) < th_tilt) ? 0.0f : tilt_angle;
                char buf[128];
                sprintf(buf, "ALIGN_MOVE %.1f %.1f %.1f\r\n", dx, dy, send_tilt);
                write(g_serial_fd, buf, strlen(buf));
            }
        }
    }
    else
    {
        cout << ">>> [视觉对齐] PnP 收敛失败！" << endl;
    }
}