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
#include <map>

using namespace cv;
using namespace std;

namespace TaskManager {
    DemoTask fetchTask() {
        std::lock_guard<std::mutex> lock(g_task_mtx);
        DemoTask task;
        if (g_demo_task.pending) {
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
                if (all_points_in) check_passed = true;
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
        next_pose.x /= -10.0; next_pose.y /= -10.0; next_pose.z /= -10.0;
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
            if (dx_pixel > 0.0f) { g_cl_state.last_pose.y -= STEP_CM; cout << "    [伺服决策] 目标框偏右，机械臂 Y 轴减小" << endl; }
            else                 { g_cl_state.last_pose.y += STEP_CM; cout << "    [伺服决策] 目标框偏左，机械臂 Y 轴增大" << endl; }
            
            if (dy_pixel > 0.0f) { g_cl_state.last_pose.x -= STEP_CM; cout << "    [伺服决策] 目标框偏下，机械臂 X 轴减小" << endl; }
            else                 { g_cl_state.last_pose.x += STEP_CM; cout << "    [伺服决策] 目标框偏上，机械臂 X 轴增大" << endl; }
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
        //g_global_x_offset_cm += delta_x_cm; // 将本次算出的误差，累加到全局补偿系统中

        // 从指令如 "FIX_131" 的第 4 位提取机械臂 ID，默认 fallback 到 ARM1
        int target_arm = (current_task.raw_cmd.length() > 4) ? (current_task.raw_cmd[4] - '0') : 1;
        if (target_arm != 0 && target_arm != 1) target_arm = 1;
        
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
            arm0_pose.x /= -10.0; arm0_pose.y /= -10.0; arm0_pose.z /= -10.0;
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
        
        // 针对 ID=9 的硬编码虚拟检测框，跳过主 YOLO 检测
        if (current_task.class_id == 9)
        {
            current_yolo_res.detected = true;
            current_yolo_res.objects.clear();

            ObjectMeta obj9;
            obj9.bbox = Rect(20, 0, 800, 460); // 左侧20，上侧0，宽800，高460
            obj9.center = Point2f(obj9.bbox.x + obj9.bbox.width / 2.0f, obj9.bbox.y + obj9.bbox.height / 2.0f);
            obj9.class_id = 9;
            obj9.confidence = 1.0f; // 虚拟置信度 100%
            obj9.has_refined_center = false;

            current_yolo_res.objects.push_back(obj9);
            cout << ">>> [特殊模式] 截获 ID=9，划定 600x500 固定选区作为目标送入 NEXT 模型！" << endl;
        }
        // ==========================================================
        // 【新增】：仅针对 DEMO001，直接使用画面中心偏移划定绝对安全框！
        // 其他 ID=0 的任务（如 DEMO000）依然会正常执行 YOLO 搜索
        // ==========================================================
        else if (current_task.raw_cmd == "DEMO001")
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
                            if (obj.class_id == 9 || current_task.raw_cmd == "DEMO001" || (!obj.ai_mask.empty() && obj.ai_mask.at<uchar>(py, px) > 0))
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

                        // 利用 1号和 4号点构造并膨胀出"虚拟 YOLO 框"
                        if (obj.class_id == 9 && obj.sub_centers.size() >= 4)
                        {
                            std::vector<Point2f> pts = obj.sub_centers;
                            std::vector<Point2f> best_v, best_h;
                            float max_v_score = -1e9, max_h_score = -1e9;

                            if (pts.size() > 40)
                                pts.resize(40);
                            int n = pts.size();

                            // 寻找纵轴 (粗略定位右侧竖线)
                            auto eval_v_subset = [&](const std::vector<Point2f> &sub)
                            {
                                float min_x = 1e9, max_x = -1e9, sum_x = 0;
                                for (auto p : sub)
                                {
                                    if (p.x < min_x)
                                        min_x = p.x;
                                    if (p.x > max_x)
                                        max_x = p.x;
                                    sum_x += p.x;
                                }
                                float x_span = max_x - min_x;
                                if (x_span < 25.0f)
                                {
                                    float score = sub.size() * 1000.0f + (sum_x / sub.size()) * 1.0f - x_span * 2.0f;
                                    if (score > max_v_score)
                                    {
                                        max_v_score = score;
                                        best_v = sub;
                                    }
                                }
                            };
                            for (int i = 0; i < n; i++)
                                for (int j = i + 1; j < n; j++)
                                {
                                    eval_v_subset({pts[i], pts[j]});
                                    for (int k = j + 1; k < n; k++)
                                    {
                                        eval_v_subset({pts[i], pts[j], pts[k]});
                                        for (int m = k + 1; m < n; m++)
                                            eval_v_subset({pts[i], pts[j], pts[k], pts[m]});
                                    }
                                }

                            // 剔除纵轴点，在剩余点中寻找横轴
                            std::vector<Point2f> rem_pts;
                            for (auto p : pts)
                            {
                                bool in_v = false;
                                for (auto vp : best_v)
                                    if (norm(p - vp) < 5.0f)
                                    {
                                        in_v = true;
                                        break;
                                    }
                                if (!in_v)
                                    rem_pts.push_back(p);
                            }
                            int m = rem_pts.size();
                            auto eval_h_subset = [&](const std::vector<Point2f> &sub)
                            {
                                float min_y = 1e9, max_y = -1e9, sum_y = 0;
                                for (auto p : sub)
                                {
                                    if (p.y < min_y)
                                        min_y = p.y;
                                    if (p.y > max_y)
                                        max_y = p.y;
                                    sum_y += p.y;
                                }
                                float y_span = max_y - min_y;
                                if (y_span < 25.0f)
                                {
                                    float score = sub.size() * 1000.0f + (sum_y / sub.size()) * 1.0f - y_span * 2.0f;
                                    if (score > max_h_score)
                                    {
                                        max_h_score = score;
                                        best_h = sub;
                                    }
                                }
                            };
                            for (int i = 0; i < m; i++)
                                for (int j = i + 1; j < m; j++)
                                {
                                    eval_h_subset({rem_pts[i], rem_pts[j]});
                                    for (int k = j + 1; k < m; k++)
                                    {
                                        eval_h_subset({rem_pts[i], rem_pts[j], rem_pts[k]});
                                        for (int l = k + 1; l < m; l++)
                                            eval_h_subset({rem_pts[i], rem_pts[j], rem_pts[k], rem_pts[l]});
                                    }
                                }

                            if (!best_v.empty() && !best_h.empty())
                            {
                                std::sort(best_v.begin(), best_v.end(), [](Point2f a, Point2f b)
                                          { return a.y < b.y; }); // 上到下
                                std::sort(best_h.begin(), best_h.end(), [](Point2f a, Point2f b)
                                          { return a.x < b.x; }); // 左到右

                                // 锁定竖线后，提取物理最底端的四棱台作为 4号点
                                float avg_v_x = 0;
                                for (auto p : best_v)
                                    avg_v_x += p.x;
                                avg_v_x /= best_v.size();

                                Point2f p4_pix = best_v.back(); // 兜底
                                float max_y = -1e9;
                                for (auto p : pts)
                                {
                                    // 放宽容差至 60 像素
                                    if (abs(p.x - avg_v_x) < 60.0f)
                                    {
                                        if (p.y > max_y)
                                        {
                                            max_y = p.y;
                                            p4_pix = p;
                                        }
                                    }
                                }

                                Point2f p1_pix = best_h.front(); // 横轴最左端 (1号点)
                                if (best_h.size() == 2)
                                {
                                    p1_pix = best_h[0] - (best_h[1] - best_h[0]);
                                    cout << ">>> 横轴 1号点缺失 已利用等距向量反推补齐" << endl;
                                }
                                else
                                {
                                    float avg_h_y = 0;
                                    for (auto p : best_h)
                                        avg_h_y += p.y;
                                    avg_h_y /= best_h.size();

                                    float min_x = 1e9;
                                    for (auto p : pts)
                                    {
                                        if (abs(p.y - avg_h_y) < 30.0f)
                                        {
                                            if (p.x < min_x)
                                            {
                                                min_x = p.x;
                                                p1_pix = p;
                                            }
                                        }
                                    }
                                }

                                // 生成虚拟紧凑框并执行四向膨胀
                                // 原点 P1为左上，P4为右下。上移20，左移60，下移40，右不变
                                int new_x = p1_pix.x - 60;
                                int new_y = p1_pix.y - 20;
                                int new_w = (p4_pix.x - p1_pix.x) + 60;
                                int new_h = (p4_pix.y - p1_pix.y) + 20 + 40;

                                obj.bbox = Rect(new_x, new_y, new_w, new_h);
                                cout << ">>> [极值锁定] ID=9 已锁最底端四棱台为4号点 准备送入底层二值化 PnP..." << endl;
                            }
                            else
                            {
                                obj.bbox = Rect(0, 0, 0, 0);
                            }
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
                if (safe_bbox.area() <= 0)
                    continue;

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
                        if (current_task.raw_cmd == "DEMO091") {
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
            cout << "[Monitor] 视觉检测结束。视野中未找到满足要求的目标 (ID=" << current_task.class_id << ")" << endl;
        return target_found; // 告诉调用者有没有找到
    }

void VisionEngine::processTask(const DemoTask &task, Mat &raw_frame)
{
    // ================== 新增路由 ==================
    if (task.raw_cmd == "HSV_FIND_ONESHOT")
    {
        handleHsvFindOneshot(task, raw_frame);
        return;
    }
    if (task.raw_cmd == "CHECK_091") {
        handleCheck091(raw_frame);
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
            if (obj.bbox.width < 15 || obj.bbox.height < 15) continue;
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

            for (const auto &pt : obj.sub_centers) circle(raw_frame, pt, 4, Scalar(0, 0, 255), -1);

            if (obj.has_refined_center && obj.corners_2d.size() == 4)
            {
                Scalar pnp_box_color = Scalar(255, 0, 255);
                for (int i = 0; i < 4; i++) line(raw_frame, obj.corners_2d[i], obj.corners_2d[(i + 1) % 4], pnp_box_color, 2);
                circle(raw_frame, obj.refined_center, 6, Scalar(0, 255, 0), 2);
                line(raw_frame, obj.center, obj.refined_center, Scalar(0, 255, 255), 1);
                char pose_text[256];
                snprintf(pose_text, sizeof(pose_text), "ID:%d P(X:%.1f Y:%.1f D:%.1f)mm | R(Rx:%.1f Ry:%.1f Rz:%.1f)deg", obj.class_id, obj.tx, obj.ty, obj.tz, obj.rx, obj.ry, obj.rz);
                putText(raw_frame, pose_text, Point(15, bottom_text_y), FONT_HERSHEY_SIMPLEX, 0.65, color, 2);
                bottom_text_y -= 30;
            }
            string label = "ID:" + to_string(obj.class_id) + " " + to_string(obj.confidence).substr(0, 4);
            if (obj.class_id != 9)
            {
                putText(raw_frame, label, Point(obj.bbox.x, max(obj.bbox.y - 5, 10)), FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            }
        }

        int pip_offset_y = 10;
        for (const auto &obj : current_yolo_res.objects)
        {
            if (!obj.roi_mask.empty() && obj.class_id != 9)
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
    inRange(hsv, Scalar(100, 100, 50), Scalar(130, 255, 255), mask);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int best_idx = -1;
    float min_dist = 1e9;
    Point2f img_center(raw_frame.cols / 2.0f, raw_frame.rows / 2.0f);

    for (size_t i = 0; i < contours.size(); i++) {
        if (contourArea(contours[i]) < 400) continue; // 过滤极小噪点
        Rect rect = boundingRect(contours[i]);
        Point2f center(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f);
        
        // 在图传画面中用红点点出中心
        circle(raw_frame, center, 6, Scalar(0, 0, 255), -1); 
        
        // 选中最接近画面中心的那一个
        float dist = norm(center - img_center);
        if (dist < min_dist) { min_dist = dist; best_idx = i; }
    }

    if (best_idx != -1) {
        // 回退到使用最小外接矩形 (RotatedRect)，抗远距离模糊能力更强
        RotatedRect rrect = minAreaRect(contours[best_idx]);
        Point2f pts[4];
        rrect.points(pts);
        
        // 矩形四角排序以适配 3D 模型的物理点位顺序 (左上、右上、右下、左下)
        std::vector<Point2f> corners(pts, pts + 4);
        std::vector<Point2f> top, bot;
        std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b) { return a.y < b.y; });
        top.push_back(corners[0]); top.push_back(corners[1]);
        bot.push_back(corners[2]); bot.push_back(corners[3]);
        if (top[0].x > top[1].x) std::swap(top[0], top[1]);
        if (bot[0].x > bot[1].x) std::swap(bot[0], bot[1]);

        // ==========================================================
        // 【核心修正】：削平 ID=1~4 顶部的“犄角”，下压 10% 还原真实物理高度
        // ==========================================================
        if (task.class_id >= 1 && task.class_id <= 4) {
            float height_left = bot[0].y - top[0].y;
            float height_right = bot[1].y - top[1].y;
            
            // 将顶部的两个角点向下平移自身高度的 10%
            top[0].y += height_left * 0.10f;
            top[1].y += height_right * 0.10f;
            
            cout << ">>> [特征修正] ID=" << task.class_id << " 发现顶部突起，角点已下压 10% 还原主体尺寸！" << endl;
        }

        corners = {top[0], top[1], bot[1], bot[0]};

        // 【UI】画出高精度提取的真实角点连线 (特意换成紫色 255,0,255，便于你在图传里核对)
        for (int i = 0; i < 4; i++) {
            line(raw_frame, corners[i], corners[(i + 1) % 4], Scalar(255, 0, 255), 2);
        }

        // 获取对应的 3D std::vector<Point2f> corners(pts物理尺寸并进行 PnP 位姿解算
        std::vector<Point3f> obj_pts_3d = get3DModelPoints(task.class_id);
        Mat rvec, tvec;
        if (solvePnP(obj_pts_3d, corners, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
            
            // --- 补偿镜头抬头 15 度带来的坐标系偏差 ---
            double theta = 15.0 * CV_PI / 180.0; 
            Mat R_pitch_comp = (Mat_<double>(3, 3) << 
                1, 0, 0, 
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
            arm1_pose.x /= -10.0; arm1_pose.y /= -10.0; arm1_pose.z /= -10.0;
            arm1_pose.x += g_arm_x_offset_cm[1]; // 专属 ARM1

            float px = arm1_pose.x;
            float py = arm1_pose.y;
            cout << ">>> [单帧寻物] HSV 外框 PnP 解析成功 | 位于 ARM1_X: " << px << " cm, ARM1_Y: " << py << " cm" << endl;

            // ===== 底盘移动核心解算逻辑 =====
            float target_x = -20.0f;
            float target_y = 8.0f;
            float forward_cm = target_x - px; 
            float right_cm = py - target_y;   
            
            cout << ">>> [底盘调度] 需前进 " << forward_cm << " cm, 需向右 " << right_cm << " cm" << endl;

            if (g_serial_fd >= 0) {
                char buf[128];
                // Y轴补偿：前后移动
                if (abs(forward_cm) > 0.5f) {
                    if (forward_cm > 0) sprintf(buf, "MW %.1f\r\n", forward_cm);
                    else sprintf(buf, "MS %.1f\r\n", -forward_cm);
                    write(g_serial_fd, buf, strlen(buf));
                    usleep(30000); // 防串口粘包
                }
                // X轴补偿：左右平移
                if (abs(right_cm) > 0.5f) {
                    if (right_cm > 0) sprintf(buf, "MD %.1f\r\n", right_cm);
                    else sprintf(buf, "MA %.1f\r\n", -right_cm);
                    write(g_serial_fd, buf, strlen(buf));
                }
            }
        } else {
            cout << ">>> [单帧寻物] PnP 矩阵收敛失败！" << endl;
        }
    } else {
        cout << ">>> [单帧寻物] 致命：画面中心周围完全没有蓝色目标！" << endl;
    }

    // 无论最终结果如何，看完最后一眼立刻把云台归位到 Nod 的位置
    if (g_serial_fd >= 0 && g_calibrated_pan >= 0) {
        char buf[64];
        sprintf(buf, "CAM %.1f %.1f\r\n", g_calibrated_pan, g_calibrated_tilt);
        write(g_serial_fd, buf, strlen(buf));
        cout << ">>> [单帧寻物] 流程收尾：云台已复位至 Nod (Pan:" << g_calibrated_pan << ", Tilt:" << g_calibrated_tilt << ")" << endl;
    }
}

void VisionEngine::handleCheck091(Mat &raw_frame)
{
    if (g_cache_091_bbox.area() <= 0) {
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

    if (right_roi.area() <= 0) return;

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

    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        float angle = atan2(abs(l[3] - l[1]), abs(l[2] - l[0])) * 180.0 / CV_PI;
        float length = norm(Point(l[0], l[1]) - Point(l[2], l[3]));

        // 判定条件：角度在 75~105 度之间，长度 >= ROI 高度的 60%
        if (angle > 75.0 && angle < 105.0 && length >= right_roi.height * 0.6) {
            int cx = (l[0] + l[2]) / 2;
            bool merged = false;
            // X轴聚类：相距 10 像素以内的线段认为是同一条边
            for (int &existing_x : valid_x_centers) {
                if (abs(cx - existing_x) < 10) { merged = true; break; }
            }
            if (!merged) {
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
    if (vertical_line_count >= 2) {
        g_cache_091_px += 0.5f; // 参数X加0.5
        Pose6D adj_pose = { g_cache_091_px, g_cache_091_py, g_cache_091_pz, 0, 0, 0 };
        cout << ">>> [视觉闭环] 未卡平 (竖线≥2)！下发微调指令 DEMO092 (X=" << g_cache_091_px << ")" << endl;
        pilot_comm.sendDemoCommand("DEMO092", adj_pose);
    } else {
        Pose6D adj_pose = { g_cache_091_px, g_cache_091_py, g_cache_091_pz, 0, 0, 0 };
        cout << ">>> [视觉闭环] 卡紧完毕 (竖线<2)！触发装配收尾指令 DEMO093" << endl;
        pilot_comm.sendDemoCommand("DEMO093", adj_pose);
    }
}