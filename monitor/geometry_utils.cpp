/**
 * @file geometry_utils.cpp
 * @brief 几何算法与手眼标定实现
 */
#include "geometry_utils.h"
#include <cmath>

using namespace cv;
using namespace std;

const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) << 996.7979, 0, 594.9983, 0, 997.4737, 381.4251, 0, 0, 1.0);
const Mat DIST_COEFFS = (Mat_<double>(5, 1) << -0.1852, -0.2471, 0.0, 0.0, 0.0);

// ============================================================================
// 数学与矩阵算法辅助函数
// ============================================================================
Point2f getBasePoint(int index, const std::vector<Point2f> &corners)
{
    if (corners.size() != 4)
        return Point2f(0, 0);
    float w1 = 2.0f / 3.0f;
    float w2 = 1.0f / 3.0f;
    switch (index)
    {
    // 底部边缘 (从左下到右下)
    case 1:
        return corners[3];
    case 2:
        return corners[3] * w1 + corners[2] * w2;
    case 3:
        return corners[3] * w2 + corners[2] * w1;
    case 4:
        return corners[2];
    // 右侧边缘 (从右下到右上)
    case 5:
        return corners[2] * w1 + corners[1] * w2;
    case 6:
        return corners[2] * w2 + corners[1] * w1;
    case 7:
        return corners[1];
    // 顶部边缘 (从右上到左上)
    case 8:
        return corners[1] * w1 + corners[0] * w2;
    case 9:
        return corners[1] * w2 + corners[0] * w1;
    case 10:
        return corners[0];
    // 左侧边缘 (从左上到左下)
    case 11:
        return corners[0] * w1 + corners[3] * w2;
    case 12:
        return corners[0] * w2 + corners[3] * w1;
    default:
        return Point2f(0, 0);
    }
}


std::vector<Point3f> get3DModelPoints(int class_id)
{
    std::vector<Point3f> pts;
    if (class_id == 0)
    {
        float w = 120.0f, h = 120.0f;
        pts.push_back(Point3f(0, -h, 0));
        pts.push_back(Point3f(w, -h, 0));
        pts.push_back(Point3f(w, 0, 0));
        pts.push_back(Point3f(0, 0, 0));
        return pts;
    }
    if (class_id == 9)
    {
        float half_w = 120.0f / 2.0f, half_h = 85.0f / 2.0f;
        pts.push_back(Point3f(-half_w, -half_h, 0)); 
        pts.push_back(Point3f(half_w, -half_h, 0));  
        pts.push_back(Point3f(half_w, half_h, 0));   
        pts.push_back(Point3f(-half_w, half_h, 0));  
        return pts;
    }
    
    float half_w = 20.0f, half_h = 30.0f;
    if (class_id == 1)      { half_w = 26.0f / 2.0f;  half_h = 85.0f / 2.0f; }
    else if (class_id == 2) { half_w = 105.0f / 2.0f; half_h = 85.0f / 2.0f; }
    else if (class_id == 3) { half_w = 74.0f / 2.0f;  half_h = 85.0f / 2.0f; }
    else if (class_id == 4) { half_w = 105.0f / 2.0f; half_h = 85.0f / 2.0f; }

    pts.push_back(Point3f(-half_w, -half_h, 0));
    pts.push_back(Point3f(half_w, -half_h, 0));
    pts.push_back(Point3f(half_w, half_h, 0));
    pts.push_back(Point3f(-half_w, half_h, 0));
    return pts;
}

std::vector<Point2f> clusterPoints(const std::vector<Point2f> &raw_pts, float dist_thresh)
{
    std::vector<Point2f> clusters;
    std::vector<int> counts;
    for (const auto &p : raw_pts)
    {
        bool found = false;
        for (size_t i = 0; i < clusters.size(); i++)
        {
            if (cv::norm(clusters[i] - p) < dist_thresh)
            {
                clusters[i].x = (clusters[i].x * counts[i] + p.x) / (counts[i] + 1);
                clusters[i].y = (clusters[i].y * counts[i] + p.y) / (counts[i] + 1);
                counts[i]++;
                found = true;
                break;
            }
        }
        if (!found)
        {
            clusters.push_back(p);
            counts.push_back(1);
        }
    }
    return clusters;
}

Mat HandEyeCalibrator::getTransformationMatrix(int arm_id)
{
    if (arm_id < 0 || arm_id > 1) arm_id = 0;
    Mat R_base = (Mat_<double>(3, 3) << 0.0, -0.707106, 0.707106, -1.0, 0.0, 0.0, 0.0, -0.707106, -0.707106);
    double r_rx = rx_[arm_id] * CV_PI / 180.0;
    double r_ry = ry_[arm_id] * CV_PI / 180.0; 
    double r_rz = rz_[arm_id] * CV_PI / 180.0;
    Mat Rx = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(r_rx), -sin(r_rx), 0, sin(r_rx), cos(r_rx));
    Mat Ry = (Mat_<double>(3, 3) << cos(r_ry), 0, sin(r_ry), 0, 1, 0, -sin(r_ry), 0, cos(r_ry));
    Mat Rz = (Mat_<double>(3, 3) << cos(r_rz), -sin(r_rz), 0, sin(r_rz), cos(r_rz), 0, 0, 0, 1);
    Mat T = Mat::eye(4, 4, CV_64F);
    Mat R_final = Rz * Ry * Rx * R_base;
    R_final.copyTo(T(Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = tx_[arm_id];
    T.at<double>(1, 3) = ty_[arm_id];
    T.at<double>(2, 3) = tz_[arm_id];
    return T;
}

Pose6D HandEyeCalibrator::transform(const Mat &rvec_cam, const Mat &tvec_cam, int arm_id)
{
    Mat T_cam2base = getTransformationMatrix(arm_id);
    Mat R_obj2cam;
    cv::Rodrigues(rvec_cam, R_obj2cam);
    Mat T_obj_cam = Mat::eye(4, 4, CV_64F);
    R_obj2cam.copyTo(T_obj_cam(Rect(0, 0, 3, 3)));
    T_obj_cam.at<double>(0, 3) = tvec_cam.at<double>(0);
    T_obj_cam.at<double>(1, 3) = tvec_cam.at<double>(1);
    T_obj_cam.at<double>(2, 3) = tvec_cam.at<double>(2);

    Mat T_obj_base = T_cam2base * T_obj_cam;
    Pose6D final_pose;
    final_pose.x = T_obj_base.at<double>(0, 3);
    final_pose.y = T_obj_base.at<double>(1, 3);
    final_pose.z = T_obj_base.at<double>(2, 3);

    Mat R_final = T_obj_base(Rect(0, 0, 3, 3));
    double sy = sqrt(R_final.at<double>(0, 0) * R_final.at<double>(0, 0) + R_final.at<double>(1, 0) * R_final.at<double>(1, 0));
    if (sy >= 1e-6)
    {
        final_pose.rx = atan2(R_final.at<double>(2, 1), R_final.at<double>(2, 2)) * 180 / CV_PI;
        final_pose.ry = atan2(-R_final.at<double>(2, 0), sy) * 180 / CV_PI;
        final_pose.rz = atan2(R_final.at<double>(1, 0), R_final.at<double>(0, 0)) * 180 / CV_PI;
    }
    else
    {
        final_pose.rx = atan2(-R_final.at<double>(1, 2), R_final.at<double>(1, 1)) * 180 / CV_PI;
        final_pose.ry = atan2(-R_final.at<double>(2, 0), sy) * 180 / CV_PI;
        final_pose.rz = 0;
    }
    return final_pose;
}