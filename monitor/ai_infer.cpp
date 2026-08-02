/**
 * @file ai_infer.cpp
 * @brief NCNN 模型加载与 YOLO 目标检测及分割
 */
#include "ai_infer.h"
#include "config.h"
#include <iostream>
#include <ncnn/net.h>
#include <ncnn/mat.h>

using namespace cv;
using namespace std;

// 局部静态变量，避免污染全局命名空间
static ncnn::Net yolo_ncnn;
static bool is_ncnn_loaded = false;
static ncnn::Net next_yolo_ncnn;
static bool is_next_ncnn_loaded = false;

YoloResult runYoloInference(const Mat &frame, int target_class_id)
{
    YoloResult result;
    result.detected = false;

    // 【新增】：跨帧记忆变量，用于第二层保险
    static cv::Rect s_last_bbox = cv::Rect(0, 0, 0, 0);
    static int s_last_class_id = -1;

    if (!is_ncnn_loaded)
    {
        yolo_ncnn.opt.use_vulkan_compute = false;
        yolo_ncnn.opt.num_threads = 1;
        if (yolo_ncnn.load_param("best.param") || yolo_ncnn.load_model("best.bin"))
        {
            std::cerr << "[Monitor] NCNN 模型加载失败！请检查文件路径" << std::endl;
            return result;
        }
        is_ncnn_loaded = true;
    }

    const int INPUT_SIZE = 320;
    int w = frame.cols, h = frame.rows;
    float scale = (w > h) ? ((float)INPUT_SIZE / w) : ((float)INPUT_SIZE / h);
    int w_pad = (int)(w * scale), h_pad = (int)(h * scale);

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(frame.data, ncnn::Mat::PIXEL_BGR2RGB, w, h, w_pad, h_pad);
    int pad_top = (INPUT_SIZE - h_pad) / 2, pad_bottom = INPUT_SIZE - h_pad - pad_top;
    int pad_left = (INPUT_SIZE - w_pad) / 2, pad_right = INPUT_SIZE - w_pad - pad_left;
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, pad_top, pad_bottom, pad_left, pad_right, ncnn::BORDER_CONSTANT, 114.0f);
    const float norm_vals[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    in_pad.substract_mean_normalize(0, norm_vals);

    ncnn::Extractor ex = yolo_ncnn.create_extractor();
    ex.input("in0", in_pad);
    ncnn::Mat out;
    ex.extract("out0", out);

    // int num_channels = out.h, num_anchors = out.w;
    // float best_score = 0.0f; int best_anchor_idx = -1;
    //
    // if (target_class_id >= 0 && (target_class_id + 4) < num_channels) {
    //    int c = target_class_id + 4;
    //    float current_thresh = (target_class_id == 1) ? SystemConfig::CONF_THRESH_TARGET : SystemConfig::CONF_THRESH_OTHER;
    //    for (int i = 0; i < num_anchors; i++) {
    //        float score = out.row(c)[i];
    //        if (score > current_thresh && score > best_score) {
    //            best_score = score; best_anchor_idx = i;
    //        }
    //    }

    int num_channels = out.h, num_anchors = out.w;
    float best_score = 0.0f;
    int best_anchor_idx = -1;

    if (target_class_id >= 0 && (target_class_id + 4) < num_channels)
    {
        int c = target_class_id + 4;
        float current_thresh = (target_class_id == 1) ? SystemConfig::CONF_THRESH_TARGET : SystemConfig::CONF_THRESH_OTHER;

        // ==============================================================
        // 【算力极限压榨版】：NMS 聚类与“最左侧优先”物理位置锁定
        // ==============================================================
        struct AnchorData
        {
            int idx;
            float score;
            Rect bbox;
        };
        std::vector<AnchorData> candidates;

        // 收集所有及格的候选框 (单纯的一维数组遍历，几乎不耗时)
        for (int i = 0; i < num_anchors; i++)
        {
            float score = out.row(c)[i];
            if (score > current_thresh)
            {
                float cx = out.row(0)[i], cy = out.row(1)[i];
                float bw = out.row(2)[i], bh = out.row(3)[i];
                float xmin = cx - bw / 2.0f, ymin = cy - bh / 2.0f;
                float xmax = cx + bw / 2.0f, ymax = cy + bh / 2.0f;
                int left = static_cast<int>((xmin - pad_left) / scale);
                int top = static_cast<int>((ymin - pad_top) / scale);
                int right = static_cast<int>((xmax - pad_left) / scale);
                int bottom = static_cast<int>((ymax - pad_top) / scale);

                // ==========================================================
                // 【新增空间过滤】：滤除 ID=0 时出现在画面左侧 2/7 区域的误检框
                // ==========================================================
                if (target_class_id == 0)
                {
                    float box_center_x = (left + right) / 2.0f;
                    float left_deadzone = frame.cols * (2.0f / 7.0f);
                    if (box_center_x < left_deadzone)
                        continue;
                }

                Rect current_bbox(left, top, right - left, bottom - top);
                candidates.push_back({i, score, current_bbox});
            }
        }

        if (!candidates.empty())
        {
            // 【算力狂飙点 1：局部排序降维】
            // 最多只有 3 个物理物体，经验上取前 12 个最高分的框足够覆盖它们了。
            int top_k = std::min((int)candidates.size(), 12);
            // 相比于 std::sort 全量排序，partial_sort 只把前 12 名排好，剩下成百上千的废框连碰都不碰
            std::partial_sort(candidates.begin(), candidates.begin() + top_k, candidates.end(), [](const AnchorData &a, const AnchorData &b)
                              { return a.score > b.score; });
            candidates.resize(top_k); // 瞬间丢弃 12 名开外的所有数据，把内存占用和后续计算量降到最低

            // 2. NMS (非极大值抑制)
            std::vector<AnchorData> nms_results;
            for (const auto &cand : candidates)
            {
                bool keep = true;
                for (const auto &kept : nms_results)
                {
                    float inter_area = (cand.bbox & kept.bbox).area();
                    float union_area = cand.bbox.area() + kept.bbox.area() - inter_area;
                    if (union_area > 0 && (inter_area / union_area) > 0.45f)
                    {
                        keep = false;
                        break;
                    }
                }
                if (keep)
                {
                    nms_results.push_back(cand);
                    // 删除了原有的 size() >= 3 强制 break 的地雷，保留所有合法物体
                }
            }

            // ==============================================================
            AnchorData best_conf_obj = nms_results[0]; // 得分最高者
            AnchorData final_choice = best_conf_obj;   // 默认兜底为最高分

            // 【最左侧物理优先】：无条件选得分 ≥ 0.25 的最左侧框
            {
                std::vector<AnchorData> sorted_by_x = nms_results;
                std::sort(sorted_by_x.begin(), sorted_by_x.end(), [](const AnchorData &a, const AnchorData &b)
                          { return a.bbox.x < b.bbox.x; });

                for (const auto &res : sorted_by_x)
                {
                    if (res.score >= 0.25f)
                    {
                        final_choice = res;
                        if (final_choice.idx != best_conf_obj.idx)
                            cout << ">>> [AI 决策] 触发最左侧优先！(选中得分:" << res.score << " vs 最高分:" << best_conf_obj.score << ")" << endl;
                        break;
                    }
                }
            }

            best_anchor_idx = final_choice.idx;
            best_score = final_choice.score;
        }
        if (best_anchor_idx != -1)
        {
            float cx = out.row(0)[best_anchor_idx], cy = out.row(1)[best_anchor_idx];
            float bw = out.row(2)[best_anchor_idx], bh = out.row(3)[best_anchor_idx];
            float xmin = cx - bw / 2.0f, ymin = cy - bh / 2.0f;
            float xmax = cx + bw / 2.0f, ymax = cy + bh / 2.0f;
            int left = static_cast<int>((xmin - pad_left) / scale);
            int top = static_cast<int>((ymin - pad_top) / scale);
            int right = static_cast<int>((xmax - pad_left) / scale);
            int bottom = static_cast<int>((ymax - pad_top) / scale);

            ObjectMeta obj;
            obj.bbox = Rect(left, top, right - left, bottom - top);
            obj.center = Point2f(obj.bbox.x + obj.bbox.width / 2.0f, obj.bbox.y + obj.bbox.height / 2.0f);
            obj.class_id = target_class_id;
            obj.confidence = best_score;

            // ====================================================
            // 新增：：：：解析 Segmentation 模型的像素掩码 (Mask)
            // ====================================================
            ncnn::Mat proto;
            // 兼容不同的导出节点命名，通常是 out1 或 output1
            int ret = ex.extract("out1", proto);
            if (ret != 0)
                ret = ex.extract("output1", proto);

            if (ret == 0)
            {
                int num_mask_coeffs = proto.c; // 通常是 32
                int inferred_num_classes = num_channels - 4 - num_mask_coeffs;

                if (inferred_num_classes > 0)
                {
                    // 1. 提取 32 个掩码系数
                    std::vector<float> mask_coeffs(num_mask_coeffs);
                    for (int k = 0; k < num_mask_coeffs; k++)
                    {
                        mask_coeffs[k] = out.row(4 + inferred_num_classes + k)[best_anchor_idx];
                    }

                    // 2. 矩阵乘法：系数 乘以 Proto特征图
                    cv::Mat mask_mat(proto.h, proto.w, CV_32FC1, cv::Scalar(0));
                    for (int p = 0; p < num_mask_coeffs; p++)
                    {
                        const float *ptr = proto.channel(p);
                        float coeff = mask_coeffs[p];
                        for (int y = 0; y < proto.h; y++)
                        {
                            float *row_ptr = mask_mat.ptr<float>(y);
                            for (int x = 0; x < proto.w; x++)
                            {
                                row_ptr[x] += ptr[y * proto.w + x] * coeff;
                            }
                        }
                    }

                    // 3. Sigmoid 激活映射到 0~1
                    cv::exp(-mask_mat, mask_mat);
                    mask_mat = 1.0f / (1.0f + mask_mat);

                    // 4. 尺寸还原 (缩放回原图大小并切掉 Padding)
                    cv::Mat mask_resized, final_mask;
                    cv::resize(mask_mat, mask_resized, cv::Size(INPUT_SIZE, INPUT_SIZE));
                    cv::Mat mask_cropped = mask_resized(cv::Rect(pad_left, pad_top, INPUT_SIZE - pad_left - pad_right, INPUT_SIZE - pad_top - pad_bottom));
                    cv::resize(mask_cropped, final_mask, cv::Size(frame.cols, frame.rows));

                    // 5. 二值化并强制用 Bbox 裁切 (过滤框外的噪点)
                    cv::Mat binary_mask;
                    cv::threshold(final_mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
                    binary_mask.convertTo(binary_mask, CV_8UC1);

                    cv::Mat safe_bbox_mask = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
                    cv::Rect safe_rect = obj.bbox & cv::Rect(0, 0, frame.cols, frame.rows);
                    if (safe_rect.area() > 0)
                    {
                        binary_mask(safe_rect).copyTo(safe_bbox_mask(safe_rect));
                    }
                    obj.ai_mask = safe_bbox_mask;
                }
            }
            // ====================================================

            // 更新记忆锁定框，供下一帧（或下个Demo）使用
            s_last_bbox = obj.bbox;
            s_last_class_id = target_class_id;

            result.objects.push_back(obj);
            cout << "[AI 专属锁定] 类别 ID: " << obj.class_id << " | 置信度: " << obj.confidence << endl;
        }
    }
    result.detected = !result.objects.empty();
    if (!result.detected)
    {
        cout << "[AI 扫描] 视野中未找到指定目标 (ID=" << target_class_id << ")" << endl;
    }
    return result;
}

std::vector<cv::Point2f> runNextYoloInferenceRaw(const cv::Mat &roi_frame)
{
    std::vector<cv::Point2f> centers;
    if (!is_next_ncnn_loaded)
    {
        next_yolo_ncnn.opt.use_vulkan_compute = false;
        next_yolo_ncnn.opt.num_threads = 1;
        if (next_yolo_ncnn.load_param("next.param") || next_yolo_ncnn.load_model("next.bin"))
        {
            std::cerr << "[Monitor] next.pt 加载失败！请检查文件路径" << std::endl;
            return centers;
        }
        is_next_ncnn_loaded = true;
    }
    if (roi_frame.empty() || roi_frame.cols <= 0 || roi_frame.rows <= 0)
        return centers;

    const int INPUT_SIZE = 320;
    int w = roi_frame.cols, h = roi_frame.rows;
    float scale = std::min((float)INPUT_SIZE / w, (float)INPUT_SIZE / h);
    int new_w = std::round(w * scale);
    int new_h = std::round(h * scale);
    int pad_x = (INPUT_SIZE - new_w) / 2;
    int pad_y = (INPUT_SIZE - new_h) / 2;

    cv::Mat resized_frame;
    cv::resize(roi_frame, resized_frame, cv::Size(new_w, new_h));
    cv::Mat padded_frame;
    cv::copyMakeBorder(resized_frame, padded_frame, pad_y, INPUT_SIZE - new_h - pad_y, pad_x, INPUT_SIZE - new_w - pad_x, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    ncnn::Mat in_pad = ncnn::Mat::from_pixels(padded_frame.data, ncnn::Mat::PIXEL_BGR2RGB, INPUT_SIZE, INPUT_SIZE);
    const float norm_vals[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    in_pad.substract_mean_normalize(0, norm_vals);

    ncnn::Extractor ex = next_yolo_ncnn.create_extractor();
    ex.input("in0", in_pad);
    ncnn::Mat out;
    ex.extract("out0", out);

    int num_channels = out.h, num_anchors = out.w;
    float conf_threshold = 0.35f;
    float global_max_score = -100.0f;

    for (int i = 0; i < num_anchors; i++)
    {
        float max_score = 0.0f;
        for (int c = 4; c < num_channels; c++)
        {
            if (out.row(c)[i] > max_score)
                max_score = out.row(c)[i];
        }
        if (max_score > global_max_score)
            global_max_score = max_score;
        if (max_score > conf_threshold)
        {
            float cx = out.row(0)[i], cy = out.row(1)[i];
            float original_cx = (cx - pad_x) / scale;
            float original_cy = (cy - pad_y) / scale;
            centers.push_back(cv::Point2f(original_cx, original_cy));
        }
    }
    std::cout << ">>> [底层探针] next.pt 识别完毕 | 目标最高置信度: " << global_max_score << std::endl;
    return centers;
}