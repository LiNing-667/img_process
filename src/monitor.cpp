/**
 * @file monitor.cpp
 * @brief 视觉监测上位机 (Brain Node) - 终极修复与面向对象重构版
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstring>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <cmath>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <ncnn/net.h>
#include <ncnn/mat.h>
#include <atomic> 
#include <fstream>
#include <map> 
#include <csignal> 
#include <opencv2/aruco.hpp> 
#define PROTOCOL_VIDEO_ENABLE
#include "protocol.hpp"

using namespace cv;
using namespace std;

// ============================================================================
// [1] 全局变量与系统配置 (状态机与硬件句柄)
// ============================================================================
VideoCapture* g_cap_ptr = nullptr;
int g_serial_fd = -1;

struct DemoTask {
    bool pending = false;
    int arm_id = -1;   
    int class_id = -1;  
    int action_id = -1; 
    std::string raw_cmd = ""; 
};
std::mutex g_task_mtx;
DemoTask g_demo_task;

struct Pose6D { float x, y, z; float rx, ry, rz; };
struct ClosedLoopState {
    std::vector<Point2f> base_corners_2d; 
    Pose6D last_pose;         
    int retry_count = 0;      
    Mat last_rvec;            
    Mat last_tvec;
    Point2f last_obj_center;  
};
ClosedLoopState g_cl_state;

std::atomic<bool> g_trigger_aruco_fix{false};
cv::Point2f g_fixed_aruco_center(-1.0f, -1.0f); 

std::atomic<bool> g_auto_cam_running{false};
float g_cam_pan = 40.0f;  
float g_cam_tilt = 40.0f; 

// 工作流同步信号
std::atomic<bool> g_wf_chassis_done{false}; // 标志小车是否到达
std::atomic<bool> g_wf_find_failed{false};  // 标志巡航是否彻底失败
// 全局 X 轴视觉校准补偿参数
float g_global_x_offset_cm = 0.0f;

namespace SystemConfig {
    const char* SERIAL_PORT       = "/dev/ttyS1";  
    const int   CAM_WIDTH         = 1280;
    const int   CAM_HEIGHT        = 720;
    const int   CAM_FPS           = 15;
    const int   JPEG_QUALITY      = 50;  
    const int   HTTP_STREAM_PORT  = 8080;  
    const float CONF_THRESH_TARGET= 0.4f;          
    const float CONF_THRESH_OTHER = 0.03f;         
}

struct ObjectMeta {
    Rect bbox;          
    Point2f center;     
    int class_id;       
    float confidence;   
    bool has_refined_center; 
    Point2f refined_center;  
    Mat roi_mask;            
    std::vector<Point2f> corners_2d; 
    double tx, ty, tz; 
    double rx, ry, rz;
    std::vector<Point2f> sub_centers;
};

struct YoloResult {
    bool detected;
    std::vector<ObjectMeta> objects;
};

struct SharedFrame {
    Mat frame;
    mutex mtx;
    condition_variable cv;
    bool ready = false;
};

struct CLTransition {
    int target_id;                   
    std::vector<int> required_points;
    std::string retry_cmd;           
    std::string success_cmd;         
};

// ============================================================================
// [2] 数学与矩阵算法辅助函数
// ============================================================================
Point2f getBasePoint(int index, const std::vector<Point2f>& corners) {
    if (corners.size() != 4) return Point2f(0,0);
    float w1 = 2.0f / 3.0f; 
    float w2 = 1.0f / 3.0f;
    switch(index) {
        // 底部边缘 (从左下到右下)
        case 1: return corners[3];
        case 2: return corners[3] * w1 + corners[2] * w2;
        case 3: return corners[3] * w2 + corners[2] * w1;
        case 4: return corners[2];
        // 右侧边缘 (从右下到右上)
        case 5: return corners[2] * w1 + corners[1] * w2;
        case 6: return corners[2] * w2 + corners[1] * w1;
        case 7: return corners[1];
        // 顶部边缘 (从右上到左上)
        case 8: return corners[1] * w1 + corners[0] * w2;
        case 9: return corners[1] * w2 + corners[0] * w1;
        case 10: return corners[0];
        // 左侧边缘 (从左上到左下)
        case 11: return corners[0] * w1 + corners[3] * w2;
        case 12: return corners[0] * w2 + corners[3] * w1;
        default: return Point2f(0,0);
    }
}

std::vector<Point3f> get3DModelPoints(int class_id) {
    std::vector<Point3f> pts;
    if (class_id == 0) { 
        float w = 120.0f; float h = 120.0f; 
        pts.push_back(Point3f(0, -h, 0)); pts.push_back(Point3f(w, -h, 0)); 
        pts.push_back(Point3f(w, 0, 0));  pts.push_back(Point3f(0, 0, 0));  
        return pts;
    }
    if (class_id == 9) {
        float half_w = 120.0f / 2.0f;
        float half_h = 85.0f / 2.0f;
        pts.push_back(Point3f(-half_w, -half_h, 0)); // TL (左上)
        pts.push_back(Point3f( half_w, -half_h, 0)); // TR (右上)
        pts.push_back(Point3f( half_w,  half_h, 0)); // BR (右下)
        pts.push_back(Point3f(-half_w,  half_h, 0)); // BL (左下)
        return pts;
    }
    float half_w = 20.0f, half_h = 30.0f; 
    if (class_id == 1) { half_w = 26.0f / 2.0f; half_h = 85.0f / 2.0f; }
    else if (class_id == 2) { half_w = 105.0f / 2.0f; half_h = 85.0f / 2.0f; }
    else if (class_id == 3) { half_w = 74.0f / 2.0f; half_h = 85.0f / 2.0f; }
    else if (class_id == 4) { half_w = 105.0f / 2.0f; half_h = 85.0f / 2.0f; }

    pts.push_back(Point3f(-half_w, -half_h, 0)); pts.push_back(Point3f( half_w, -half_h, 0)); 
    pts.push_back(Point3f( half_w,  half_h, 0)); pts.push_back(Point3f(-half_w,  half_h, 0)); 
    return pts;
}

const Mat CAMERA_MATRIX = (Mat_<double>(3, 3) << 996.7979, 0, 594.9983, 0, 997.4737, 381.4251, 0, 0, 1.0);
const Mat DIST_COEFFS = (Mat_<double>(5, 1) << -0.1852, -0.2471, 0.0, 0.0, 0.0);

std::vector<Point2f> clusterPoints(const std::vector<Point2f>& raw_pts, float dist_thresh = 10.0f) {
    std::vector<Point2f> clusters;
    std::vector<int> counts;
    for (const auto& p : raw_pts) {
        bool found = false;
        for (size_t i = 0; i < clusters.size(); i++) {
            if (cv::norm(clusters[i] - p) < dist_thresh) {
                clusters[i].x = (clusters[i].x * counts[i] + p.x) / (counts[i] + 1);
                clusters[i].y = (clusters[i].y * counts[i] + p.y) / (counts[i] + 1);
                counts[i]++;
                found = true;
                break;
            }
        }
        if (!found) { clusters.push_back(p); counts.push_back(1); }
    }
    return clusters;
}

class HandEyeCalibrator {
private:
    double tx_[2] = {-2.40, -2.60}; 
    double ty_[2] = {91.0, -91.0};  
    double tz_[2] = {205.0, 205.0}; 
    double rx_[2] = {0.0, 0.0}; 
    double ry_[2] = {0.0, 0.0}; 
    double rz_[2] = {0.0, 0.0};

    Mat getTransformationMatrix(int arm_id) {
        if (arm_id < 0 || arm_id > 1) arm_id = 0; 
        Mat R_base = (Mat_<double>(3, 3) << 0.0, -0.707106, 0.707106, -1.0, 0.0, 0.0, 0.0, -0.707106, -0.707106);
        double r_rx = rx_[arm_id] * CV_PI / 180.0, r_ry = ry_[arm_id] * CV_PI / 180.0, r_rz = rz_[arm_id] * CV_PI / 180.0;
        Mat Rx = (Mat_<double>(3,3) << 1,0,0, 0,cos(r_rx),-sin(r_rx), 0,sin(r_rx),cos(r_rx));
        Mat Ry = (Mat_<double>(3,3) << cos(r_ry),0,sin(r_ry), 0,1,0, -sin(r_ry),0,cos(r_ry));
        Mat Rz = (Mat_<double>(3,3) << cos(r_rz),-sin(r_rz),0, sin(r_rz),cos(r_rz),0, 0,0,1);
        Mat T = Mat::eye(4, 4, CV_64F);
        Mat R_final = Rz * Ry * Rx * R_base; 
        R_final.copyTo(T(Rect(0, 0, 3, 3)));
        T.at<double>(0, 3) = tx_[arm_id]; T.at<double>(1, 3) = ty_[arm_id]; T.at<double>(2, 3) = tz_[arm_id];
        return T;
    }

public:
    Pose6D transform(const Mat& rvec_cam, const Mat& tvec_cam, int arm_id) {
        Mat T_cam2base = getTransformationMatrix(arm_id);
        Mat R_obj2cam; cv::Rodrigues(rvec_cam, R_obj2cam);
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
        double sy = sqrt(R_final.at<double>(0,0) * R_final.at<double>(0,0) + R_final.at<double>(1,0) * R_final.at<double>(1,0));
        if (sy >= 1e-6) {
            final_pose.rx = atan2(R_final.at<double>(2,1), R_final.at<double>(2,2)) * 180 / CV_PI;
            final_pose.ry = atan2(-R_final.at<double>(2,0), sy) * 180 / CV_PI;
            final_pose.rz = atan2(R_final.at<double>(1,0), R_final.at<double>(0,0)) * 180 / CV_PI;
        } else {
            final_pose.rx = atan2(-R_final.at<double>(1,2), R_final.at<double>(1,1)) * 180 / CV_PI;
            final_pose.ry = atan2(-R_final.at<double>(2,0), sy) * 180 / CV_PI;
            final_pose.rz = 0;
        }
        return final_pose;
    }
};

// ============================================================================
// [3] 通信模块与系统初始化封装
// ============================================================================
void signalHandler(int signum) {
    std::cout << "\n\n========================================================" << std::endl;
    std::cout << "[系统] 收到强制终止信号 (" << signum << ")，正在安全释放硬件资源..." << std::endl;
    if (g_cap_ptr && g_cap_ptr->isOpened()) {
        g_cap_ptr->release();
        std::cout << "[系统] 摄像头节点已安全释放。" << std::endl;
    }
    if (g_serial_fd >= 0) {
        close(g_serial_fd);
        std::cout << "[系统] 串口已安全关闭。" << std::endl;
    }
    std::cout << "========================================================\n" << std::endl;
    exit(signum);
}

int initSerialPort(const char* portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;
    cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK; tty.c_lflag = 0; tty.c_oflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return fd;
}

namespace SystemInit {
    void initAll() {
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
        cout << "[Monitor] 初始化 2K0300 系统..." << endl;
        g_serial_fd = initSerialPort(SystemConfig::SERIAL_PORT); 
        if (g_serial_fd < 0) {
            cerr << "[警告] 串口打开失败，将无法向 Pilot 发送数据！" << endl;
        } else {
            cout << "[Monitor] 串口通信就绪！" << endl;
        }
    }
}

class PilotCommunicator {
public:
    void sendDemoCommand(const std::string& demo_name, const Pose6D& pose) {
        if (g_serial_fd < 0) {
            std::cout << "[警告] 串口未打开，无法下发指令！" << std::endl;
            return;
        }
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "%s %.2f %.2f %.2f %.2f %.2f %.2f\r\n", 
                 demo_name.c_str(), pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz);
        std::cout << "\n=============================================" << std::endl;
        std::cout << "[通信层] -> 发送宏指令至 Pilot: " << buffer;
        std::cout << "=============================================\n" << std::endl;
        write(g_serial_fd, buffer, strlen(buffer));
    }
};
// ============================================================================
// 【新增】统一文本指令解析网关 (支持本地终端输入与远端 PC 发送)
// ============================================================================
void processTextCommand(const std::string& cmd_line) {
    if (cmd_line.empty()) return;
    std::string lower_cmd = cmd_line; 
    for (auto& c : lower_cmd) c = tolower(c);

    if (lower_cmd == "fix") {
        g_trigger_aruco_fix = true;
        std::cout << "\n>>> [静态标定] 已下发单次 ArUco 捕捉指令..." << std::endl;
        return;
    }
    if (lower_cmd == "nod") {
        g_auto_cam_running = true;
        g_cam_pan = 37.0f; g_cam_tilt = 38.0f;
        if (g_serial_fd >= 0) {
            char buf[64];
            sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
            write(g_serial_fd, buf, strlen(buf));
        }
        std::cout << "\n>>> [自适应云台] 启动！开始提取车板特征曲线..." << std::endl;
        return;
    }
    if (lower_cmd == "find") {
        std::cout << "\n>>> [状态机] 启动巡航搜索，下发云台就位指令 DEMO220..." << std::endl;
        if (g_serial_fd >= 0) {
            std::string send_str = "DEMO220 0 0 0 0 0 0 0 0 0 0\r\n";
            write(g_serial_fd, send_str.c_str(), send_str.length());
        }
        return;
    }

    if (lower_cmd == "start") {
        std::thread([]() {
            std::cout << "\n=============================================" << std::endl;
            std::cout << ">>> 全自动装配宏动作链 [START] 启动！" << std::endl;
            std::cout << "=============================================\n" << std::endl;

            auto send_serial_cmd = [](const std::string& cmd, int wait_ms) {
                if (g_serial_fd >= 0) {
                    std::string full_cmd = cmd + "\r\n";
                    write(g_serial_fd, full_cmd.c_str(), full_cmd.length());
                    std::cout << ">>> [动作链] 下发指令: " << cmd << " (等待 " << wait_ms/1000.0 << " 秒)" << std::endl;
                }
                usleep(wait_ms * 1000);
            };

            auto do_nod = []() {
                std::cout << ">>> [动作链] 触发云台视觉调平 (Nod)..." << std::endl;
                g_auto_cam_running = true;
                g_cam_pan = 37.0f; g_cam_tilt = 38.0f;
                if (g_serial_fd >= 0) {
                    char buf[64]; sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt);
                    write(g_serial_fd, buf, strlen(buf));
                }
                usleep(1000000);  
                g_auto_cam_running = true;
                while (g_auto_cam_running) { usleep(100000); }
            };

            auto do_vision_demo = [](int arm_id, int class_id, int action_id, const std::string& cmd, int wait_ms) {
                std::cout << ">>> [动作链] 派发视觉任务: " << cmd << " (ARM" << arm_id << " 锁定 ID=" << class_id << ")" << std::endl;
                {
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.arm_id = arm_id;     
                    g_demo_task.class_id = class_id;   
                    g_demo_task.action_id = action_id;  
                    g_demo_task.raw_cmd = cmd;
                }
                usleep(wait_ms * 1000); 
            };

                            // =======================================================
                // 动作链正式开始编排 (时间单位为毫秒 ms)
                // =======================================================

            //  send_serial_cmd("DEMO", 5000); 
            //  // 第1步：小车移动。
            //  send_serial_cmd("MW 35", 3000);  
            //  send_serial_cmd("MQ", 3500);  
            //  send_serial_cmd("MW 40", 2000);  
            //  send_serial_cmd("MQ", 3500); 
            //  send_serial_cmd("CH 8 20", 3000);
            //  send_serial_cmd("MW 12", 2000);

            //  // 第2步：运行nod指令。
            //  do_nod();
            //  usleep(500000); // 停顿 0.5 秒让云台稳定

            //  // 第3步：运行demo131 
            //  do_vision_demo(1, 3, 1, "DEMO131", 25000); 

            //  // 第4步：运行两三个小车移动指令。
            //  send_serial_cmd("MS 18", 2000); 
            //  send_serial_cmd("ME", 4000);
            //  send_serial_cmd("MW 22", 2000);  

            //  // 第5步：运行demo000 (寻找底座ID=0测试定位)
            //  do_vision_demo(0, 0, 0, "DEMO000", 20000); 

            //  // 第6步：运行两三个小车移动指令。
            //  send_serial_cmd("MS 40", 3000); 
            //  send_serial_cmd("ME ", 4000);   
            //  send_serial_cmd("MW 20", 3000); 


            // 第7步：运行demo091 (虚拟框ID=9的坐标提取，并由底座缓存坐标)
            do_vision_demo(0, 9, 1, "DEMO091", 20000); 
            // 第8步：运行do031 (交接 + ARM0提取091记忆坐标)
            send_serial_cmd("DO031", 25000); 
            // 9 :
            send_serial_cmd("MS 30", 2000) ;

            std::cout << ">>> 动作链结束！" << std::endl;
        }).detach(); 
        return;
    }

    if (lower_cmd.rfind("demo", 0) == 0 && lower_cmd.length() == 7) {
        int x = lower_cmd[4] - '0'; 
        int y = lower_cmd[5] - '0'; 
        int z = lower_cmd[6] - '0'; 
        if (x >= 0 && x <= 1 && y >= 0 && z >= 0) { 
            std::lock_guard<std::mutex> lock(g_task_mtx);
            g_demo_task.pending = true;
            g_demo_task.arm_id = x; g_demo_task.class_id = y; g_demo_task.action_id = z;
            std::string upper_cmd = lower_cmd;
            for (auto& c : upper_cmd) c = toupper(c);
            g_demo_task.raw_cmd = upper_cmd; 
            std::cout << "[Monitor] 已接收视觉任务 -> 目标臂: ARM" << x << " | 物体ID: " << y << std::endl;
            return; 
        }
    }
    if (lower_cmd.rfind("do", 0) == 0 && lower_cmd.length() == 5) {
        std::string upper_cmd = lower_cmd;
        for (auto& c : upper_cmd) c = toupper(c); 
        if (g_serial_fd >= 0) {
            std::string send_str = upper_cmd + " 0 0 0 0 0 0 0 0 0 0\r\n";
            write(g_serial_fd, send_str.c_str(), send_str.length());
            std::cout << "[Monitor] 下发盲操作 -> " << upper_cmd << std::endl;
        }
        return; 
    }
    // 未知指令，直接向底盘透传
    if (g_serial_fd >= 0) {
        std::string send_str = cmd_line + "\r\n";
        write(g_serial_fd, send_str.c_str(), send_str.length());
        std::cout << "[串口发往Pilot] -> " << cmd_line << std::endl;
    }
}

void terminalCommandThreadFunc() {
    std::cout << "========================================================" << std::endl;
    std::cout << "[终端] 串口遥控模式就绪！输入 demoxyz 触发视觉检测与抓取！" << std::endl;
    std::cout << "========================================================" << std::endl;

    std::string cmd_line;
    while (std::getline(std::cin, cmd_line)) {
        processTextCommand(cmd_line); // 统一交由网关处理
    }
}

void serialReadThreadFunc() {
    char buffer[256];
    std::string rx_buffer;
    while (true) {
        if (g_serial_fd < 0) { usleep(100000); continue; }
        int n = read(g_serial_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            rx_buffer += buffer;
            size_t pos;
            while ((pos = rx_buffer.find_first_of("\r\n")) != std::string::npos) {
                std::string line = rx_buffer.substr(0, pos);
                rx_buffer.erase(0, pos + 1);
                if (line.empty()) continue;
                if (line.rfind("H", 0) == 0 && line.length() >= 3) {
                    std::cout << "\n[Monitor 接收] 收到底层组装完成信号: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHECK_" + line; 
                }
                else if (line.rfind("FIX_111", 0) == 0 || line.rfind("FIX_131", 0) == 0) {
                    std::cout << "\n[Monitor 接收] 收到底层视觉对齐请求: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = line; 
                }
                else if (line.rfind("FIND_ACK_", 0) == 0) {
                    std::cout << "\n[Monitor 接收] 收到云台就位确认: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = line; 
                }
                else if (line.rfind("CHASSIS_DONE", 0) == 0) {
                    std::cout << "\n[Monitor 接收] 收到小车底盘就位确认: " << line << std::endl;
                    std::lock_guard<std::mutex> lock(g_task_mtx);
                    g_demo_task.pending = true;
                    g_demo_task.raw_cmd = "CHASSIS_DONE"; 
                }
            }
        } else { usleep(10000); }
    }
}

namespace CommunicationManager {
    void startThreads() {
        thread cmd_thread(terminalCommandThreadFunc);
        cmd_thread.detach();
        thread rx_thread(serialReadThreadFunc);
        rx_thread.detach();
    }
}

// ============================================================================
// [4] 硬件控制与推流服务器封装
// ============================================================================
namespace CameraManager {
    // 增加 silent 参数，静默重连时不再疯狂打印探测信息
    bool forceCameraFormat(int dev_id, bool silent = false) {
        char dev_name[32];
        sprintf(dev_name, "/dev/video%d", dev_id);
        int fd = open(dev_name, O_RDWR);
        if (fd < 0) return false;
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = SystemConfig::CAM_WIDTH;
        fmt.fmt.pix.height = SystemConfig::CAM_HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            if (!silent) std::cerr << "[底层强控] 警告：格式强制指令失败！" << std::endl;
        } else {
            if (!silent) std::cout << "[底层强控] 请求 1280x720 MJPG。硬件分配: " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << std::endl;
        }
        struct v4l2_streamparm parm;
        memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = SystemConfig::CAM_FPS;
        ioctl(fd, VIDIOC_S_PARM, &parm);
        close(fd);
        return true;
    }

    void cameraThreadFunc(VideoCapture *cap, SharedFrame *shared) {
        Mat temp; 
        int fail_count = 0;
        bool error_reported = false; // 增加标志位，掉线只报一次错

        while (true) {
            // 加入 cap->isOpened() 判断，防止未初始化时崩溃
            if (!cap->isOpened() || !cap->read(temp) || temp.empty()) { 
                fail_count++;
                if (fail_count > 30) { 
                    if (!error_reported) {
                        std::cerr << "\n[Camera Watchdog] 摄像头连接断开！已进入后台静默重连，不打扰其他模块调试..." << std::endl;
                        error_reported = true; // 锁定报错，不再刷屏
                    }
                    cap->release(); 
                    usleep(20000000); // 延长重连间隔为 20 秒一次，极大地降低 CPU 占用

                    for (int dev_id = 0; dev_id < 4; ++dev_id) {
                        forceCameraFormat(dev_id, true); // true 代表静默探测
                        std::vector<int> params = {
                            CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                            CAP_PROP_FRAME_WIDTH, SystemConfig::CAM_WIDTH,
                            CAP_PROP_FRAME_HEIGHT, SystemConfig::CAM_HEIGHT,
                            CAP_PROP_FPS, SystemConfig::CAM_FPS,
                            CAP_PROP_BUFFERSIZE, 1
                        };
                        cap->open(dev_id, CAP_V4L2, params);
                        if (cap->isOpened() && cap->get(CAP_PROP_FRAME_WIDTH) >= 640) {
                            fail_count = 0;
                            error_reported = false; // 重置报错标志
                            std::cout << "\n[Camera Watchdog] 摄像头热插拔重连成功！画面恢复更新。\n" << std::endl;
                            break;
                        } else { 
                            cap->release(); 
                        }
                    }
                } else { 
                    usleep(30000); 
                }
                continue; 
            }
            
            // 成功读到画面，清除错误状态
            fail_count = 0; 
            error_reported = false;
            
            {
                lock_guard<mutex> lock(shared->mtx);
                temp.copyTo(shared->frame);
                shared->ready = true;
            }
            shared->cv.notify_one();
        }
    }

    VideoCapture probeAndInit() {
        VideoCapture cap; bool camera_opened = false;
        for (int retry = 0; retry < 3; ++retry) {
            for (int dev_id = 0; dev_id < 4; ++dev_id) { 
                cout << "[Monitor] 正在探测 /dev/video" << dev_id << "..." << endl;
                forceCameraFormat(dev_id, false); usleep(100000);
                std::vector<int> params = {
                    CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    CAP_PROP_FRAME_WIDTH, SystemConfig::CAM_WIDTH,
                    CAP_PROP_FRAME_HEIGHT, SystemConfig::CAM_HEIGHT,
                    CAP_PROP_FPS, SystemConfig::CAM_FPS,
                    CAP_PROP_BUFFERSIZE, 1
                };
                cap.open(dev_id, CAP_V4L2, params);
                if (cap.isOpened() && cap.get(CAP_PROP_FRAME_WIDTH) >= 640) {
                    cout << "[Monitor] 成功锁定视频节点 /dev/video" << dev_id << endl;
                    camera_opened = true; break;
                } else { cap.release(); }
            }
            if (camera_opened) break;
            sleep(1); 
        }
        
        // 去掉 exit(-1) 强制退出，改为警告并放行，支持完全无头启动
        if (!camera_opened) { 
            cerr << "\n[Monitor] 警告：系统初始化时未检测到摄像头！\n>>> 已进入无摄像头调试模式，后续插上摄像头将自动热插拔重连..." << endl; 
        } else {
            cout << "[Monitor] 实际分辨率: " << cap.get(CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
            Mat temp; for (int i = 0; i < 5; i++) cap >> temp; // 预热
        }
        
        return cap;
    }

    void startCaptureThread(VideoCapture& cap, SharedFrame& shared) {
        g_cap_ptr = &cap;
        thread cam_thread(cameraThreadFunc, &cap, &shared);
        cam_thread.detach();
    }

    bool getLatestFrame(SharedFrame &shared, Mat &frame, int timeout_ms) {
        unique_lock<mutex> lock(shared.mtx);
        if (!shared.cv.wait_for(lock, chrono::milliseconds(timeout_ms), [&shared] { return shared.ready; })) return false;
        if (shared.frame.empty()) return false;
        shared.frame.copyTo(frame); shared.ready = false; 
        return true;
    }
}


class HttpStreamServer {
public:
    int server_fd;
    HttpStreamServer(int port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1; setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // 【核心修复】：必须将 server_fd 设置为非阻塞模式！
        // 否则主循环的 accept() 会把整个图像处理和图传线程彻底卡死
        int flags = fcntl(server_fd, F_GETFL, 0);
        fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

        struct sockaddr_in address; address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY; address.sin_port = htons(port);
        bind(server_fd, (struct sockaddr *)&address, sizeof(address));
        listen(server_fd, 3);
        cout << "[Monitor] 视觉推流就绪。浏览器访问: http://[本板IP]:" << port << endl;
    }

    int acceptClient() {
        int client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket < 0) return -1; // 非阻塞模式下，没人连会直接 return -1，不卡主线程
        struct timeval timeout; timeout.tv_sec = 1; timeout.tv_usec = 0;
        setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
        cout << "\n检测到浏览器连接 开始推流" << endl;
        string header = "HTTP/1.0 200 OK\r\nConnection: close\r\nContent-Type: multipart/x-mixed-replace; boundary=--myboundary\r\n\r\n";
        send(client_socket, header.c_str(), header.size(), MSG_NOSIGNAL);
        return client_socket;
    }
    bool sendFrame(int client_socket, const Mat& raw_frame, vector<uchar>& buffer, const vector<int>& encode_params) {
        imencode(".jpg", raw_frame, buffer, encode_params);
        string chunk_header = "--myboundary\r\nContent-Type: image/jpeg\r\nContent-Length: " + to_string(buffer.size()) + "\r\n\r\n";
        if (send(client_socket, chunk_header.c_str(), chunk_header.size(), MSG_NOSIGNAL) < 0) return false;
        if (send(client_socket, buffer.data(), buffer.size(), MSG_NOSIGNAL) < 0) return false;
        if (send(client_socket, "\r\n", 2, MSG_NOSIGNAL) < 0) return false;
        return true;
    }
};

// ============================================================================
// PC 上位机二进制协议服务器 (Socket)
// ============================================================================
class PcProtocolServer {
private:
    int cmd_server_fd, video_server_fd;
    std::atomic<int> cmd_sock{-1};
    std::atomic<int> video_sock{-1};
    
    int createServer(int port) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1; setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr; addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY; addr.sin_port = htons(port);
        bind(fd, (struct sockaddr *)&addr, sizeof(addr));
        listen(fd, 3);
        return fd;
    }

    void cmdAcceptLoop() {
        while (true) {
            int client = accept(cmd_server_fd, nullptr, nullptr);
            if (client >= 0) {
                if (cmd_sock >= 0) close(cmd_sock);
                cmd_sock = client;
                std::cout << "\n>>> [PC协议] 指令链路已连接! (来自上位机) <<<" << std::endl;
                std::thread(&PcProtocolServer::cmdWorker, this, client).detach();
            }
        }
    }

    void videoAcceptLoop() {
        while (true) {
            int client = accept(video_server_fd, nullptr, nullptr);
            if (client >= 0) {
                // 【修复】：自动挤掉并释放旧连接，防止上位机断线重连导致 fd 泄漏
                int old_sock = video_sock.exchange(client);
                if (old_sock >= 0) close(old_sock);
                std::cout << ">>> [PC协议] 图传链路已连接! 开始高速推流 <<<" << std::endl;
            }
        }
    }

    void cmdWorker(int sock) {
        std::vector<uint8_t> buf;
        uint8_t tmp[4096];
        while (cmd_sock == sock) {
            int n = recv(sock, tmp, sizeof(tmp), 0);
            if (n <= 0) break;
            buf.insert(buf.end(), tmp, tmp + n);

            while (true) {
                protocol::ParsedFrame f;
                size_t consumed = protocol::consume(buf.data(), buf.size(), f);
                if (!consumed) break; // 数据不足或CRC错误，等待下一波接收
                buf.erase(buf.begin(), buf.begin() + consumed);

                // ==========================================================
                // 【核心指令路由】：对接 Monitor 原有系统
                // ==========================================================
                // ==========================================================
                // 【核心指令路由】：对接 Monitor 原有系统
                // ==========================================================
                switch (f.cmd) {
                    case protocol::CMD_HEARTBEAT: {
                        auto resp = protocol::build_resp_ok("alive");
                        send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                        break;
                    }
                    case protocol::CMD_VEHICLE_POS: {
                        float x, y, z, yaw;
                        protocol::parse_vehicle_pos(f, x, y, z, yaw);
                        std::cout << "[PC指令] 收到小车坐标: X=" << x << " Y=" << y << " Yaw=" << yaw << std::endl;
                        auto resp = protocol::build_resp_ok();
                        send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                        break;
                    }
                    case protocol::CMD_ARM_JOINTS: {
                        uint8_t num = protocol::parse_arm_num(f);
                        std::cout << "[PC指令] 收到 " << (int)num << " 个关节角度下发" << std::endl;
                        auto resp = protocol::build_resp_ok();
                        send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                        break;
                    }
                    case protocol::CMD_EXEC_PROGRAM: {
                        uint8_t prog_id = f.payload[0];
                        char cmd_buf[32];
                        sprintf(cmd_buf, "DEMO%03d", prog_id); 
                        std::cout << "[PC指令] 触发系统动作流水线: " << cmd_buf << std::endl;
                        
                        std::lock_guard<std::mutex> lock(g_task_mtx);
                        g_demo_task.pending = true;
                        g_demo_task.raw_cmd = cmd_buf;
                        g_demo_task.class_id = 0; 
                        
                        auto resp = protocol::build_resp_ok();
                        send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                        break;
                    }
                    // 【新增】：急停指令
                    case protocol::CMD_EMERGENCY: {
                        std::cout << "\n[PC指令] !!! 收到 EMERGENCY 急停指令 !!!" << std::endl;
                        if (g_serial_fd >= 0) {
                            std::string send_str = "0\r\n"; // "0" 在 pilot.cpp 中直接触发底盘锁死
                            write(g_serial_fd, send_str.c_str(), send_str.length());
                        }
                        auto resp = protocol::build_resp_ok();
                        send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                        break;
                    }

                    // 【修改】：文本指令路由打通
                    case protocol::CMD_TEXT: {
                        std::string text((const char*)f.payload, f.plen);
                        // 清理上位机可能不小心发送过来的换行符
                        text.erase(std::remove(text.begin(), text.end(), '\r'), text.end());
                        text.erase(std::remove(text.begin(), text.end(), '\n'), text.end());
                        
                        std::cout << "\n[PC指令] 收到文本命令: " << text << std::endl;
                        
                        // 【核心打通】：将上位机发来的文本交给中央网关解析
                        processTextCommand(text); 
                        
                        auto resp = protocol::build_resp_ok();
                        send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                        break;
                    }
                }
            }
        }
        close(sock);
        if (cmd_sock == sock) cmd_sock = -1;
        std::cout << "[PC协议] 指令链路断开" << std::endl;
    }

public:
    PcProtocolServer(int cmd_port, int video_port) {
        cmd_server_fd = createServer(cmd_port);
        video_server_fd = createServer(video_port);
        std::thread(&PcProtocolServer::cmdAcceptLoop, this).detach();
        std::thread(&PcProtocolServer::videoAcceptLoop, this).detach();
        std::cout << "[Monitor] PC 端二进制协议网关已启动 | 指令端口: " << cmd_port << " | 图传端口: " << video_port << std::endl;
    }

    void sendVideo(const cv::Mat& frame) {
        int sock = video_sock.load();
        if (sock >= 0) {
            // 使用 protocol.hpp 中的 namespace video 下的方法
            if (!video::send_jpeg(sock, frame, 60)) {
                close(sock);
                video_sock = -1;
            }
        }
    }
};

// ============================================================================
// [5] AI 算法网络层封装
// ============================================================================
ncnn::Net yolo_ncnn;
bool is_ncnn_loaded = false;
ncnn::Net next_yolo_ncnn;
bool is_next_ncnn_loaded = false;

YoloResult runYoloInference(const Mat& frame, int target_class_id) {
    YoloResult result; result.detected = false;
    if (!is_ncnn_loaded) {
        yolo_ncnn.opt.use_vulkan_compute = false;
        yolo_ncnn.opt.num_threads = 1;
        if (yolo_ncnn.load_param("best.param") || yolo_ncnn.load_model("best.bin")) {
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
    ex.input("in0", in_pad); ncnn::Mat out; ex.extract("out0", out); 

    int num_channels = out.h, num_anchors = out.w;
    float best_score = 0.0f; int best_anchor_idx = -1;

    if (target_class_id >= 0 && (target_class_id + 4) < num_channels) {
        int c = target_class_id + 4; 
        float current_thresh = (target_class_id == 1) ? SystemConfig::CONF_THRESH_TARGET : SystemConfig::CONF_THRESH_OTHER; 
        for (int i = 0; i < num_anchors; i++) {
            float score = out.row(c)[i]; 
            if (score > current_thresh && score > best_score) {
                best_score = score; best_anchor_idx = i;
            }
        }
        if (best_anchor_idx != -1) {
            float cx = out.row(0)[best_anchor_idx], cy = out.row(1)[best_anchor_idx];
            float bw  = out.row(2)[best_anchor_idx], bh  = out.row(3)[best_anchor_idx];
            float xmin = cx - bw / 2.0f, ymin = cy - bh / 2.0f;
            float xmax = cx + bw / 2.0f, ymax = cy + bh / 2.0f;
            int left   = static_cast<int>((xmin - pad_left) / scale);
            int top    = static_cast<int>((ymin - pad_top) / scale);
            int right  = static_cast<int>((xmax - pad_left) / scale);
            int bottom = static_cast<int>((ymax - pad_top) / scale);

            ObjectMeta obj;
            obj.bbox = Rect(left, top, right - left, bottom - top);
            obj.center = Point2f(obj.bbox.x + obj.bbox.width / 2.0f, obj.bbox.y + obj.bbox.height / 2.0f);
            obj.class_id = target_class_id; obj.confidence = best_score;
            result.objects.push_back(obj);
            cout << "[AI 专属锁定] 类别 ID: " << obj.class_id << " | 置信度: " << obj.confidence << endl;
        }
    }
    result.detected = !result.objects.empty();
    if (!result.detected) { cout << "[AI 扫描] 视野中未找到指定目标 (ID=" << target_class_id << ")" << endl; }
    return result;
}

std::vector<Point2f> runNextYoloInferenceRaw(const Mat& roi_frame) {
    std::vector<Point2f> centers;
    if (!is_next_ncnn_loaded) {
        next_yolo_ncnn.opt.use_vulkan_compute = false;
        next_yolo_ncnn.opt.num_threads = 1;
        if (next_yolo_ncnn.load_param("next.param") || next_yolo_ncnn.load_model("next.bin")) {
            std::cerr << "[Monitor] next.pt 加载失败！请检查文件路径" << std::endl;
            return centers;
        }
        is_next_ncnn_loaded = true;
    }
    if (roi_frame.empty() || roi_frame.cols <= 0 || roi_frame.rows <= 0) return centers;

    const int INPUT_SIZE = 320;
    int w = roi_frame.cols, h = roi_frame.rows;
    float scale = std::min((float)INPUT_SIZE / w, (float)INPUT_SIZE / h);
    int new_w = std::round(w * scale); int new_h = std::round(h * scale);
    int pad_x = (INPUT_SIZE - new_w) / 2; int pad_y = (INPUT_SIZE - new_h) / 2;

    Mat resized_frame; cv::resize(roi_frame, resized_frame, Size(new_w, new_h));
    Mat padded_frame; cv::copyMakeBorder(resized_frame, padded_frame, pad_y, INPUT_SIZE - new_h - pad_y, pad_x, INPUT_SIZE - new_w - pad_x, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    ncnn::Mat in_pad = ncnn::Mat::from_pixels(padded_frame.data, ncnn::Mat::PIXEL_BGR2RGB, INPUT_SIZE, INPUT_SIZE);
    const float norm_vals[3] = {1.0f / 255.0f, 1.0f / 255.0f, 1.0f / 255.0f};
    in_pad.substract_mean_normalize(0, norm_vals);

    ncnn::Extractor ex = next_yolo_ncnn.create_extractor();
    ex.input("in0", in_pad); ncnn::Mat out; ex.extract("out0", out); 

    int num_channels = out.h, num_anchors = out.w;
    float conf_threshold = 0.35f; 
    float global_max_score = -100.0f; 

    for (int i = 0; i < num_anchors; i++) {
        float max_score = 0.0f;
        for (int c = 4; c < num_channels; c++) {
            if (out.row(c)[i] > max_score) max_score = out.row(c)[i];
        }
        if (max_score > global_max_score) global_max_score = max_score;
        if (max_score > conf_threshold) {
            float cx = out.row(0)[i], cy = out.row(1)[i];
            float original_cx = (cx - pad_x) / scale;
            float original_cy = (cy - pad_y) / scale;
            centers.push_back(Point2f(original_cx, original_cy));
        }
    }
    std::cout << ">>> [底层探针] next.pt 识别完毕 | 目标最高置信度: " << global_max_score << std::endl;
    return centers;
}

// 传统视觉辅助算法提取
bool findOrderedCorners(const Mat& roi_frame, int class_id, std::vector<Point2f>& ordered_corners, Mat& out_mask) {
    if (roi_frame.empty() || roi_frame.cols < 15 || roi_frame.rows < 15) return false;
    Mat gray, blurred; cvtColor(roi_frame, gray, COLOR_BGR2GRAY); GaussianBlur(gray, blurred, Size(9, 9), 0);
    Mat sample_patch;
    if (class_id == 3) {
        int p_size = 20;
        Point2f pt_left(roi_frame.cols * 0.25f, roi_frame.rows / 2.0f);
        Point2f pt_right(roi_frame.cols * 0.75f, roi_frame.rows / 2.0f);
        Point2f pt_bottom(roi_frame.cols / 2.0f, roi_frame.rows * 0.75f);
        Rect r_left(pt_left.x - p_size/2, pt_left.y - p_size/2, p_size, p_size);
        Rect r_right(pt_right.x - p_size/2, pt_right.y - p_size/2, p_size, p_size);
        Rect r_bottom(pt_bottom.x - p_size/2, pt_bottom.y - p_size/2, p_size, p_size);
        r_left &= Rect(0, 0, roi_frame.cols, roi_frame.rows); r_right &= Rect(0, 0, roi_frame.cols, roi_frame.rows); r_bottom &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
        std::vector<Mat> patches;
        if (r_left.area() > 0) patches.push_back(blurred(r_left));
        if (r_right.area() > 0) patches.push_back(blurred(r_right));
        if (r_bottom.area() > 0) patches.push_back(blurred(r_bottom));
        if (patches.empty()) return false;
        cv::vconcat(patches, sample_patch);
    } else {
        Point2f sample_pt; int p_size = 30;
        if (class_id == 4) { sample_pt = Point2f(roi_frame.cols / 2.0f, roi_frame.rows * 0.3f); p_size = 20; } 
        else { sample_pt = Point2f(roi_frame.cols / 2.0f, roi_frame.rows / 2.0f); }
        Rect center_patch_rect(sample_pt.x - p_size/2, sample_pt.y - p_size/2, p_size, p_size);  
        center_patch_rect &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
        if (center_patch_rect.area() <= 0) return false;
        sample_patch = blurred(center_patch_rect);
    }
    Scalar mean_val, stddev_val; meanStdDev(sample_patch, mean_val, stddev_val);
    double tolerance = max(stddev_val[0] * 3.0, 30.0);   
    Mat mask; inRange(blurred, mean_val[0] - tolerance, mean_val[0] + tolerance, mask);
    Mat open_kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    Mat close_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, open_kernel); morphologyEx(mask, mask, MORPH_CLOSE, close_kernel); mask.copyTo(out_mask);

    vector<vector<Point>> contours; findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    double max_area = 0; vector<Point> best_contour;
    for (const auto& c : contours) {
        double area = contourArea(c);
        if (area > max_area) { max_area = area; best_contour = c; }
    }
    if (max_area < roi_frame.cols * roi_frame.rows * 0.05) return false;
    RotatedRect rect = minAreaRect(best_contour); Point2f rect_pts[4]; rect.points(rect_pts); Point2f center = rect.center;
    
    std::vector<Point2f> corners;
    for (int i = 0; i < 4; i++) {
        Point2f v_dir = rect_pts[i] - center;
        double max_dot = -1e9; Point2f best_pt;
        for (const auto& cp : best_contour) {
            Point2f pt(cp.x, cp.y); Point2f v_pt = pt - center;
            double dot_prod = v_pt.x * v_dir.x + v_pt.y * v_dir.y;
            if (dot_prod > max_dot) { max_dot = dot_prod; best_pt = pt; }
        }
        corners.push_back(best_pt);
    }
    std::vector<Point2f> top, bot;
    std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b){ return a.y < b.y; });
    top.push_back(corners[0]); top.push_back(corners[1]); bot.push_back(corners[2]); bot.push_back(corners[3]);
    if (top[0].x > top[1].x) std::swap(top[0], top[1]);
    if (bot[0].x > bot[1].x) std::swap(bot[0], bot[1]);
    ordered_corners = {top[0], top[1], bot[1], bot[0]}; return true;
}

bool findWallCorners(const Mat& roi_frame, std::vector<Point2f>& ordered_corners, Mat& out_mask) {
    if (roi_frame.empty() || roi_frame.cols < 15 || roi_frame.rows < 15) return false;
    Mat gray, blurred, mask; 
    cvtColor(roi_frame, gray, COLOR_BGR2GRAY); 
    GaussianBlur(gray, blurred, Size(9, 9), 0);
    
    // 动态阈值提取对象
    Point2f center(roi_frame.cols / 2.0f, roi_frame.rows / 2.0f); 
    Rect center_patch_rect(center.x - 15, center.y - 15, 30, 30);  
    center_patch_rect &= Rect(0, 0, roi_frame.cols, roi_frame.rows);
    if (center_patch_rect.area() <= 0) return false;
    
    Mat center_patch = blurred(center_patch_rect);
    Scalar mean_val, stddev_val; meanStdDev(center_patch, mean_val, stddev_val);
    double tolerance = max(stddev_val[0] * 3.0, 30.0);   
    inRange(blurred, mean_val[0] - tolerance, mean_val[0] + tolerance, mask);
    
    Mat open_kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
    Mat close_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, open_kernel); 
    morphologyEx(mask, mask, MORPH_CLOSE, close_kernel); 
    
    // 将原始的掩码保存给外部显示
    out_mask = mask.clone();
    
    // 1. 用一个较大的核进行开运算，强制“吃掉”左右两侧较薄的凸起矩形
    Mat body_mask;
    Mat strong_open_kernel = getStructuringElement(MORPH_RECT, Size(15, 15)); 
    morphologyEx(mask, body_mask, MORPH_OPEN, strong_open_kernel);

    // 2. 获取纯净的中间主矩形轮廓
    vector<vector<Point>> body_contours;
    findContours(body_mask, body_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (body_contours.empty()) return false;
    
    double max_area = 0; vector<Point> best_body;
    for (const auto& c : body_contours) {
        double area = contourArea(c);
        if (area > max_area) { max_area = area; best_body = c; }
    }
    if (max_area < roi_frame.cols * roi_frame.rows * 0.05) return false;

    // 3. 计算中间主矩形的精确位姿 (提取真实的绝对高度和旋转角度)
    RotatedRect body_rect = minAreaRect(best_body);
    Point2f bp[4];
    body_rect.points(bp);

    // 判断哪条是长轴(宽度方向)，哪条是短轴(高度方向)
    float len01 = norm(bp[1] - bp[0]);
    float len12 = norm(bp[2] - bp[1]);
    Point2f v_long, v_short;
    float true_height;
    
    if (len01 > len12) {
        v_long = (bp[1] - bp[0]) / len01;  // 长轴单位向量
        v_short = (bp[2] - bp[1]) / len12; // 短轴单位向量
        true_height = len12;               // 绝对真实的等高高度
    } else {
        v_long = (bp[2] - bp[1]) / len12;
        v_short = (bp[1] - bp[0]) / len01;
        true_height = len01;
    }

    // 4. 将原始完整 mask(包含侧边凸起) 的所有点，降维投影到长轴上
    vector<Point> all_pts;
    findNonZero(mask, all_pts);
    if (all_pts.empty()) return false;

    float min_proj = 1e9, max_proj = -1e9;
    for (const auto& p : all_pts) {
        Point2f v = Point2f(p.x, p.y) - body_rect.center;
        // 点乘计算在这个向量方向上的物理延伸长度
        float proj = v.x * v_long.x + v.y * v_long.y; 
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }

    // 5. 虚拟重构：利用 真实宽度 + 真实高度 凭空组装出完美的 4 个角点
    float true_length = max_proj - min_proj;
    float center_shift = (max_proj + min_proj) / 2.0f;
    Point2f true_center = body_rect.center + v_long * center_shift;

    Point2f half_long = v_long * (true_length / 2.0f);
    Point2f half_short = v_short * (true_height / 2.0f);

    std::vector<Point2f> corners;
    corners.push_back(true_center + half_long + half_short);
    corners.push_back(true_center + half_long - half_short);
    corners.push_back(true_center - half_long - half_short);
    corners.push_back(true_center - half_long + half_short);

    // 6. 将生成的理想点顺时针排序 (TL, TR, BR, BL)
    std::vector<Point2f> top, bot;
    std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b){ return a.y < b.y; });
    top.push_back(corners[0]); top.push_back(corners[1]); 
    bot.push_back(corners[2]); bot.push_back(corners[3]);
    
    if (top[0].x > top[1].x) std::swap(top[0], top[1]);
    if (bot[0].x > bot[1].x) std::swap(bot[0], bot[1]);
    ordered_corners = {top[0], top[1], bot[1], bot[0]}; 
    
    return true;
}

// ============================================================================
// [6] 高级视觉处理引擎封装 
// ============================================================================
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

class VisionEngine {
private:
    PilotCommunicator& pilot_comm;
    HandEyeCalibrator calibrator;
    YoloResult current_yolo_res;
    bool has_detection = false;

    void handleClosedLoopCheck(const DemoTask& current_task, Mat& raw_frame) {
        std::map<std::string, CLTransition> cl_map = {
            {"CHECK_H11", {1, {1},    "DEMO101", "DEMO001"}},
            {"CHECK_H21", {2, {2, 3}, "DEMO001", "DEMO021"}},
            {"CHECK_H12", {1, {1},    "DEMO102", "DEMO002"}},
            {"CHECK_H22", {2, {2, 3}, "DEMO002", "DEMO022"}}
        };
        if (cl_map.find(current_task.raw_cmd) == cl_map.end()) {
            cout << "[视觉闭环] 未知的闭环复核指令: " << current_task.raw_cmd << endl;
            return;
        }
        CLTransition trans = cl_map[current_task.raw_cmd];
        cout << "[视觉闭环] 正在扫描 ID=" << trans.target_id << " 进行拼装位重复核..." << endl;
        
        current_yolo_res = runYoloInference(raw_frame, trans.target_id); 
        has_detection = true;
        bool check_passed = false; bool found_target_obj = false;
        Point2f active_obj_center, active_target_pt;
        
        if (current_yolo_res.detected) {
            for (auto& obj : current_yolo_res.objects) {
                if (obj.class_id == trans.target_id) {
                    found_target_obj = true; active_obj_center = obj.center;
                    active_target_pt = getBasePoint(trans.required_points[0], g_cl_state.base_corners_2d);
                    bool all_points_in = true; float margin = 10.0f; 
                    for (int pt_idx : trans.required_points) {
                        Point2f pt = getBasePoint(pt_idx, g_cl_state.base_corners_2d);
                        if (!(pt.x >= obj.bbox.x && pt.x <= obj.bbox.x + obj.bbox.width &&
                              pt.y >= obj.bbox.y && pt.y <= obj.bbox.y + obj.bbox.height - margin)) {
                            all_points_in = false; break; 
                        }
                    }
                    if (all_points_in) check_passed = true;
                    break;
                }
            }
        }
        if (check_passed) {
            cout << ">>> [闭环成功] 指定特征点被完美覆盖，装配精准！下发 " << trans.success_cmd << "..." << endl;
            g_cl_state.retry_count = 0;
            int next_arm = trans.success_cmd[4] - '0'; 
            Pose6D next_pose = calibrator.transform(g_cl_state.last_rvec, g_cl_state.last_tvec, next_arm);
            next_pose.x /= -10.0; next_pose.y /= -10.0; next_pose.z /= -10.0;
            next_pose.x += g_global_x_offset_cm;
            g_cl_state.last_pose = next_pose;
            pilot_comm.sendDemoCommand(trans.success_cmd, next_pose);
        } else {
            g_cl_state.retry_count++;
            cout << ">>> [闭环失败] 残差未达标。计算图像伺服反馈... (重试:" << g_cl_state.retry_count << ")" << endl;
            if (found_target_obj) {
                const float STEP_CM = 0.5f; 
                float dx_pixel = active_obj_center.x - active_target_pt.x;
                float dy_pixel = active_obj_center.y - active_target_pt.y;
                cout << "    [残差分析] 像素偏差 -> DX: " << dx_pixel << " px | DY: " << dy_pixel << " px" << endl;
                if (dx_pixel > 0.0f) { g_cl_state.last_pose.y -= STEP_CM; cout << "    [伺服决策] 目标框偏右，机械臂 Y 轴减小 2.0mm" << endl; } 
                else { g_cl_state.last_pose.y += STEP_CM; cout << "    [伺服决策] 目标框偏左，机械臂 Y 轴增大 2.0mm" << endl; }
                if (dy_pixel > 0.0f) { g_cl_state.last_pose.x -= STEP_CM; cout << "    [伺服决策] 目标框偏下，机械臂 X 轴减小 2.0mm" << endl; } 
                else { g_cl_state.last_pose.x += STEP_CM; cout << "    [伺服决策] 目标框偏上，机械臂 X 轴增大 2.0mm" << endl; }
            } else { cout << "    [伺服警告] 视野内丢失目标类目标框，保持原位坐标重试。" << endl; }
            pilot_comm.sendDemoCommand(trans.retry_cmd, g_cl_state.last_pose);
        }
    }

    void handleSingleAxisServo(const DemoTask& current_task, Mat& raw_frame) {
        Mat clean_gray; cvtColor(raw_frame, clean_gray, COLOR_BGR2GRAY);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        parameters->minMarkerPerimeterRate = 0.01; 
        std::vector<int> marker_ids; std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(clean_gray, dictionary, marker_corners, marker_ids, parameters);

        Point2f aruco_center(-1, -1);
        if (!marker_ids.empty()) {
            Point2f p1 = marker_corners[0][0]; Point2f p2 = marker_corners[0][1]; 
            Point2f p3 = marker_corners[0][2]; Point2f p4 = marker_corners[0][3]; 
            aruco_center = Point2f((p1.x+p2.x+p3.x+p4.x)/4.0f, (p1.y+p2.y+p3.y+p4.y)/4.0f);
        }
        Point2f obj_center = g_cl_state.last_obj_center;

        if (aruco_center.y >= 0 && obj_center.y >= 0 && !g_cl_state.last_tvec.empty()) {
            double tz_mm = g_cl_state.last_tvec.at<double>(2);
            float dynamic_scale_cm = (float)(tz_mm / 7053.0); 
            float delta_x_cm = (obj_center.y - aruco_center.y) * dynamic_scale_cm;

            g_global_x_offset_cm += delta_x_cm;    // 将本次算出的误差，累加到全局补偿系统中
            float calibrated_px = g_cl_state.last_pose.x + delta_x_cm;

            std::cout << "\n>>> [视觉对齐成功] PnP 深度提取: Tz=" << tz_mm << "mm | 动态比例尺: " << dynamic_scale_cm << " cm/px" << std::endl;
            std::cout << ">>> ArUco纵向: " << aruco_center.y << " | 物体纵向: " << obj_center.y << std::endl;
            std::cout << ">>> X轴(前后) 校准量: " << delta_x_cm << " cm" << std::endl;
            
            Pose6D fix_pose = g_cl_state.last_pose;
            fix_pose.x = calibrated_px; 
            // 根据接收到的 FIX 信号，动态决定下一步该发什么抓取指令
            std::string next_cmd = "DEMO112"; 
            if (current_task.raw_cmd == "FIX_131") {
                next_cmd = "DEMO132";
            }
            
            pilot_comm.sendDemoCommand(next_cmd, fix_pose);
        } else {
            std::cout << "\n>>> [伺服失败] 未找齐 ArUco 与目标，或深度矩阵丢失！1秒后重试..." << std::endl;
            usleep(1000000);
            std::lock_guard<std::mutex> lock(g_task_mtx);
            g_demo_task.pending = true; 
        }
    }

    bool handleBlindOperations(const DemoTask& current_task) {
        if (current_task.raw_cmd == "DEMO002"|| current_task.raw_cmd == "DEMO003"|| current_task.raw_cmd == "DEMO004") {
            if (!g_cl_state.last_rvec.empty() && !g_cl_state.last_tvec.empty()) {
                cout << ">>> [视觉记忆跳跃] 检测到单独下发 " << current_task.raw_cmd << "，调用物理矩阵跳过 YOLO！" << endl;
                Pose6D arm0_pose = calibrator.transform(g_cl_state.last_rvec, g_cl_state.last_tvec, 0);
                arm0_pose.x /= -10.0; arm0_pose.y /= -10.0; arm0_pose.z /= -10.0;
                arm0_pose.x += g_global_x_offset_cm;
                g_cl_state.last_pose = arm0_pose;
                pilot_comm.sendDemoCommand(current_task.raw_cmd, arm0_pose);
                return true; 
            } else {
                cout << ">>> [警告] 内存中暂无底座矩阵(请先执行101/102)，降级为重新视觉扫描..." << endl;
            }
        }
        return false;
    }

    bool handleYoloAndPnP(const DemoTask& current_task, Mat& raw_frame) {
        cout << "[Monitor] 正在执行神经网络与 6D 位姿解算 (锁定ID=" << current_task.class_id << ")..." << endl;
        // 针对 ID=9 的硬编码虚拟检测框，跳过主 YOLO 检测
        if (current_task.class_id == 9) {
            current_yolo_res.detected = true;
            current_yolo_res.objects.clear();
            
            ObjectMeta obj9;
            obj9.bbox = Rect(20, 0, 800, 460); // 左侧20，上侧0，宽600，高500
            obj9.center = Point2f(obj9.bbox.x + obj9.bbox.width / 2.0f, obj9.bbox.y + obj9.bbox.height / 2.0f);
            obj9.class_id = 9;
            obj9.confidence = 1.0f; // 虚拟置信度 100%
            obj9.has_refined_center = false;
            
            current_yolo_res.objects.push_back(obj9);
            cout << ">>> [特殊模式] 截获 ID=9，划定 600x500 固定选区作为目标送入 NEXT 模型！" << endl;
        } else {
            // 对于其他 ID，走正常的 YOLO 推理流程
            current_yolo_res = runYoloInference(raw_frame, current_task.class_id);
        }
        has_detection = true; bool target_found = false;

        if (current_yolo_res.detected) {
            for (auto& obj : current_yolo_res.objects) {
                if (obj.class_id == 0 || obj.class_id == 9) {
                    Rect safe_crop = obj.bbox & Rect(0, 0, raw_frame.cols, raw_frame.rows);
                    if (safe_crop.area() > 0) {
                        Mat roi_frame = raw_frame(safe_crop); 
                        std::vector<Point2f> raw_centers = runNextYoloInferenceRaw(roi_frame);
                        std::vector<Point2f> global_raw;
                        for (const auto& pt : raw_centers) global_raw.push_back(Point2f(pt.x + safe_crop.x, pt.y + safe_crop.y));
                        obj.sub_centers = clusterPoints(global_raw, 12.0f); 
                        cout << ">>> [二次级联] ID=" << obj.class_id << " 区域内原始 " << raw_centers.size() << " 个候选点，聚类出 " << obj.sub_centers.size() << " 个四棱台特征。" << endl;

                        // 注意：只有 ID=0 才去做透视矩阵的角点修复
                        // 注意：只有 ID=0 才去做透视矩阵的角点修复
                        if (obj.class_id == 0 && obj.sub_centers.size() >= 4) {
                            bool perspective_fixed = false;
                            
                            // ==============================================================
                            // 把 ID=0 框切成两半，只取右半边
                            // ==============================================================
                            std::vector<Point2f> pts;
                            float mid_x = obj.bbox.x + obj.bbox.width / 2.0f; // 获取 YOLO 框的正中心 X 坐标
                            for (auto p : obj.sub_centers) {
                                // 严格执行：只看右半面的点，左半边的杂点瞬间物理湮灭！
                                if (p.x > mid_x) {
                                    pts.push_back(p);
                                }
                            }
                            
                            if (pts.size() > 40) pts.resize(40);
                            int n = pts.size();

                            // 只要右半边剩下的点足够构成一条竖线，就直接开干！
                            if (n >= 4) {
                                std::vector<Point2f> best_v;
                                float min_x_span = 1e9;
                                
                                // 1. 大道至简：暴力穷举找 X 跨度最小的 4 个点 (锁定最右侧竖线)
                                for(int i=0; i<n; i++) {
                                    for(int j=i+1; j<n; j++) {
                                        for(int k=j+1; k<n; k++) {
                                            for(int m=k+1; m<n; m++) {
                                                float min_x = min({pts[i].x, pts[j].x, pts[k].x, pts[m].x});
                                                float max_x = max({pts[i].x, pts[j].x, pts[k].x, pts[m].x});
                                                float min_y = min({pts[i].y, pts[j].y, pts[k].y, pts[m].y});
                                                float max_y = max({pts[i].y, pts[j].y, pts[k].y, pts[m].y});
                                                float x_span = max_x - min_x;
                                                float y_span = max_y - min_y;
                                                
                                                // Y 跨度只需大于 35，保证是一根有长度的物理线即可
                                                if (y_span > 35.0f && x_span < min_x_span) {
                                                    min_x_span = x_span;
                                                    best_v = {pts[i], pts[j], pts[k], pts[m]};
                                                }
                                            }
                                        }
                                    }
                                }

                                if (best_v.size() == 4) {
                                    std::sort(best_v.begin(), best_v.end(), [](Point2f a, Point2f b){ return a.y < b.y; });
                                    Point2f P7 = best_v[0]; // 右上角 (7号点)
                                    Point2f P4 = best_v[3]; // 右下角 (4号点)

                                    std::vector<Point2f> rem_pts;
                                    for (auto p : pts) {
                                        bool in_v = false;
                                        for (auto vp : best_v) if (norm(p - vp) < 5.0f) { in_v = true; break; }
                                        if (!in_v) rem_pts.push_back(p);
                                    }

                                    // 2. 在极其纯净的右半区，使用“物理距离”死锁最近的两个点！
                                    std::vector<Point2f> bot_line, top_line;
                                    for (auto p : rem_pts) {
                                        if (abs(p.y - P4.y) < 45.0f && p.x < P4.x - 5.0f) bot_line.push_back(p);
                                        if (abs(p.y - P7.y) < 45.0f && p.x < P7.x - 5.0f) top_line.push_back(p);
                                    }

                                    // 必须往左至少有两个点 (即存在 3和2, 8和9)
                                    if (bot_line.size() >= 2 && top_line.size() >= 2) {
                                        // 按照物理直线距离从小到大排序
                                        std::sort(bot_line.begin(), bot_line.end(), [P4](Point2f a, Point2f b){ return norm(a - P4) < norm(b - P4); });
                                        std::sort(top_line.begin(), top_line.end(), [P7](Point2f a, Point2f b){ return norm(a - P7) < norm(b - P7); });
                                        
                                        // 由于左侧脏点已被全歼，第 0 个必是 3/8 号，第 1 个绝对就是 2/9 号！
                                        Point2f P2 = bot_line[1]; 
                                        Point2f P9 = top_line[1]; 

                                        // 物理上 4到2 是底边全宽的 2/3，直接外推 1.5 倍补全
                                        Point2f P1 = P4 + 1.5f * (P2 - P4);
                                        Point2f P10 = P7 + 1.5f * (P9 - P7);

                                        obj.corners_2d = {P10, P7, P4, P1};
                                        perspective_fixed = true;
                                        cout << ">>> [右半区绝对隔离] 成功抹杀左半边噪点，完美定位右侧骨架及 2/9 号点！" << endl;
                                    }
                                }
                            }

                            // ==============================================================
                            // 3. 万能兜底逻辑（没遮挡或提取失败时执行）
                            // ==============================================================
                            if (!perspective_fixed) {
                                RotatedRect min_rect = minAreaRect(obj.sub_centers); 
                                Point2f rect_pts[4]; min_rect.points(rect_pts);
                                std::vector<Point2f> corners; 
                                std::vector<Point2f> available_pts = obj.sub_centers; 
                                for (int i = 0; i < 4; i++) {
                                    int best_idx = -1; float min_dist = 1e9;
                                    for (size_t j = 0; j < available_pts.size(); j++) {
                                        float d = norm(rect_pts[i] - available_pts[j]);
                                        if (d < min_dist) { min_dist = d; best_idx = j; }
                                    }
                                    if (best_idx != -1) { corners.push_back(available_pts[best_idx]); available_pts.erase(available_pts.begin() + best_idx); }
                                }
                                std::vector<Point2f> top, bot;
                                std::sort(corners.begin(), corners.end(), [](Point2f a, Point2f b){ return a.y < b.y; });
                                top.push_back(corners[0]); top.push_back(corners[1]); bot.push_back(corners[2]); bot.push_back(corners[3]);
                                if (top[0].x > top[1].x) std::swap(top[0], top[1]);
                                if (bot[0].x > bot[1].x) std::swap(bot[0], bot[1]);
                                obj.corners_2d = {top[0], top[1], bot[1], bot[0]};
                            }
                        }
                
                        // -------------------------------------------------------------
                        // 【核心重构】：利用 1号和 4号点构造并膨胀出“虚拟 YOLO 框”
                        // -------------------------------------------------------------
                        if (obj.class_id == 9 && obj.sub_centers.size() >= 4) {
                            std::vector<Point2f> pts = obj.sub_centers;
                            std::vector<Point2f> best_v, best_h;
                            float max_v_score = -1e9, max_h_score = -1e9;

                            if (pts.size() > 40) pts.resize(40);
                            int n = pts.size();

                            // 1. 寻找纵轴 (最下方且 X 方差小)
                            auto eval_v_subset = [&](const std::vector<Point2f>& sub) {
                                float min_x = 1e9, max_x = -1e9, sum_x = 0;
                                for(auto p : sub) { if(p.x < min_x) min_x = p.x; if(p.x > max_x) max_x = p.x; sum_x += p.x; }
                                float x_span = max_x - min_x;
                                if (x_span < 25.0f) {
                                    float score = sub.size() * 1000.0f + (sum_x/sub.size()) * 1.0f - x_span * 2.0f;
                                    if (score > max_v_score) { max_v_score = score; best_v = sub; }
                                }
                            };
                            for(int i=0; i<n; i++) for(int j=i+1; j<n; j++) {
                                eval_v_subset({pts[i], pts[j]});
                                for(int k=j+1; k<n; k++) {
                                    eval_v_subset({pts[i], pts[j], pts[k]});
                                    for(int m=k+1; m<n; m++) eval_v_subset({pts[i], pts[j], pts[k], pts[m]});
                                }
                            }

                            // 2. 剔除纵轴点，在剩余点中找横轴 (最左方且 Y 方差小)
                            std::vector<Point2f> rem_pts;
                            for(auto p : pts) {
                                bool in_v = false;
                                for(auto vp : best_v) if (norm(p - vp) < 5.0f) { in_v = true; break; }
                                if (!in_v) rem_pts.push_back(p);
                            }
                            int m = rem_pts.size();
                            auto eval_h_subset = [&](const std::vector<Point2f>& sub) {
                                float min_y = 1e9, max_y = -1e9, sum_y = 0;
                                for(auto p : sub) { if(p.y < min_y) min_y = p.y; if(p.y > max_y) max_y = p.y; sum_y += p.y; }
                                float y_span = max_y - min_y;
                                if (y_span < 25.0f) {
                                    float score = sub.size() * 1000.0f + (sum_y/sub.size()) * 1.0f - y_span * 2.0f;
                                    if (score > max_h_score) { max_h_score = score; best_h = sub; }
                                }
                            };
                            for(int i=0; i<m; i++) for(int j=i+1; j<m; j++) {
                                eval_h_subset({rem_pts[i], rem_pts[j]});
                                for(int k=j+1; k<m; k++) {
                                    eval_h_subset({rem_pts[i], rem_pts[j], rem_pts[k]});
                                    for(int l=k+1; l<m; l++) eval_h_subset({rem_pts[i], rem_pts[j], rem_pts[k], rem_pts[l]});
                                }
                            }

                            if (!best_v.empty() && !best_h.empty()) {
                                std::sort(best_v.begin(), best_v.end(), [](Point2f a, Point2f b){ return a.y < b.y; }); // 上到下
                                std::sort(best_h.begin(), best_h.end(), [](Point2f a, Point2f b){ return a.x < b.x; }); // 左到右

                                Point2f p4_pix = best_v.back();  // 纵轴最底端 (4号点)
                                Point2f p1_pix = best_h.front(); // 横轴最左端 (1号点)

                                if (best_h.size() == 2) {
                                    p1_pix = best_h[0] - (best_h[1] - best_h[0]);
                                    cout << ">>> [降维预警] 横轴 1号点缺失，已利用等距向量反推补齐！" << endl;
                                }

                                // 3. 生成虚拟紧凑框并执行四向膨胀
                                // 原点 P1为左上，P4为右下。要求：上移20，左移60，下移40，右不变。
                                int new_x = p1_pix.x - 60;
                                int new_y = p1_pix.y - 20;
                                int new_w = (p4_pix.x - p1_pix.x) + 60;      // 左侧移了60，右侧不变，故宽增加60
                                int new_h = (p4_pix.y - p1_pix.y) + 20 + 40; // 上移20下移40，故高增加60

                                // 直接覆盖掉原始的巨大假框，让它化身为一个精准的局部先验框！
                                obj.bbox = Rect(new_x, new_y, new_w, new_h);
                                cout << ">>> [无缝接入] ID=9 已成功膨胀虚拟边界框，准备送入底层二值化 PnP..." << endl;
                            } else {
                                obj.bbox = Rect(0,0,0,0); // 未能提取出合理骨架，置空使其安全跳过本帧
                            }
                        } else if (obj.class_id == 9) {
                            obj.bbox = Rect(0,0,0,0); // 点数不足，置空安全跳过
                        }
                        // -------------------------------------------------------------
                    }
                }

                obj.has_refined_center = false; Rect safe_bbox = obj.bbox & Rect(0, 0, raw_frame.cols, raw_frame.rows);
                if (safe_bbox.area() <= 0) continue;
                bool feature_extracted = false;
                if (obj.class_id == 0) { 
                    if (obj.corners_2d.size() == 4) feature_extracted = true; 
                }
                else {
                    Mat roi_frame = raw_frame(safe_bbox); std::vector<Point2f> local_corners;
                    feature_extracted = (obj.class_id == 1 || obj.class_id == 2) ? 
                                        findWallCorners(roi_frame, local_corners, obj.roi_mask) : 
                                        findOrderedCorners(roi_frame, obj.class_id, local_corners, obj.roi_mask);
                    if (feature_extracted) {
                        std::vector<Point2f> global_corners(4);
                        for(int i=0; i<4; i++) global_corners[i] = Point2f(safe_bbox.x + local_corners[i].x, safe_bbox.y + local_corners[i].y);
                        obj.corners_2d = global_corners;
                    }
                }

                if (feature_extracted) {
                    std::vector<Point3f> obj_pts_3d = get3DModelPoints(obj.class_id);
                    Mat rvec, tvec;
                    if (solvePnP(obj_pts_3d, obj.corners_2d, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
                        //if (obj.class_id == 1) {
                        //    Mat R_obj; cv::Rodrigues(rvec, R_obj);
                        //    double theta_rad = -14.036 * CV_PI / 180.0; 
                        //    Mat R_corr = (Mat_<double>(3,3) << cos(theta_rad), -sin(theta_rad), 0, sin(theta_rad),  cos(theta_rad), 0, 0, 0, 1);
                        //    Mat t_corr = (Mat_<double>(3,1) << -2.1, 4.0, 0.0); 
                        //    Mat R_new = R_obj * R_corr; Mat t_new = R_obj * t_corr + tvec;
                        //    cv::Rodrigues(R_new, rvec); tvec = t_new;
                        //}
                        obj.tx = tvec.at<double>(0); obj.ty = tvec.at<double>(1); obj.tz = tvec.at<double>(2);
                        Mat R_temp; cv::Rodrigues(rvec, R_temp);
                        double sy_temp = sqrt(R_temp.at<double>(0,0)*R_temp.at<double>(0,0) + R_temp.at<double>(1,0)*R_temp.at<double>(1,0));
                        obj.rx = atan2(R_temp.at<double>(2,1), R_temp.at<double>(2,2)) * 180 / CV_PI;
                        obj.ry = atan2(-R_temp.at<double>(2,0), sy_temp) * 180 / CV_PI;
                        obj.rz = atan2(R_temp.at<double>(1,0), R_temp.at<double>(0,0)) * 180 / CV_PI;
                        
                        obj.has_refined_center = true;
                        std::vector<Point3f> center_3d = {Point3f(0,0,0)}; std::vector<Point2f> center_2d;
                        projectPoints(center_3d, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS, center_2d);
                        obj.refined_center = center_2d[0];

                        Pose6D arm_target_pose = calibrator.transform(rvec, tvec, current_task.arm_id);
                        arm_target_pose.x /= -10.0; arm_target_pose.y /= -10.0; arm_target_pose.z /= -10.0;
                        arm_target_pose.x += g_global_x_offset_cm;

                        if ((current_task.raw_cmd == "DEMO101" || current_task.raw_cmd == "DEMO102") && obj.corners_2d.size() == 4) {
                            g_cl_state.base_corners_2d = obj.corners_2d; g_cl_state.retry_count = 0;
                        }
                        g_cl_state.last_pose = arm_target_pose;
                        rvec.copyTo(g_cl_state.last_rvec); tvec.copyTo(g_cl_state.last_tvec);
                        g_cl_state.last_obj_center = obj.center;

                        cout << ">>> [坐标转化完成] 发现目标物体 ID: " << obj.class_id << "\n    下发串口指令 -> " << current_task.raw_cmd 
                             << " 移动至: X=" << arm_target_pose.x << " Y=" << arm_target_pose.y << " Z=" << arm_target_pose.z << " (厘米)" << endl;
                        pilot_comm.sendDemoCommand(current_task.raw_cmd, arm_target_pose);
                        target_found = true; break; 
                    }
                }
            }
        }
        if (!target_found) cout << "[Monitor] 视觉检测结束。视野中未找到满足要求的目标 (ID=" << current_task.class_id << ")" << endl;
        return target_found; // 告诉调用者有没有找到
    }

public:
    VisionEngine(PilotCommunicator& comm) : pilot_comm(comm) {}

   // 核心路由控制器
    void processTask(const DemoTask& task, Mat& raw_frame) {
        
        // 1. 处理巡航寻找逻辑
        if (task.raw_cmd.rfind("FIND_ACK_", 0) == 0) {
            cout << "\n>>> [巡航搜索] 云台已就位，开始扫描 ID=2..." << endl;
            // 构造一个虚拟任务，用 YOLO+PnP 流水线
            DemoTask search_task;
            search_task.class_id = 2; // 锁定寻找 ID=2
            search_task.arm_id = 1;   // 解算基准 ARM1
            search_task.raw_cmd = "CHASSIS_MOVE"; // 占位指令：一旦算好坐标，就把坐标连同此指令发给Pilot
            
            // 流水线内部会自动把 "CHASSIS_MOVE X Y Z..." 发给 Pilot
            bool found = handleYoloAndPnP(search_task, raw_frame);
            
            if (!found) {
                if (task.raw_cmd == "FIND_ACK_220") {
                    cout << ">>> [巡航搜索] 视角1 未发现目标 继续搜索..." << endl;
                    Pose6D empty_pose{0,0,0,0,0,0};
                    pilot_comm.sendDemoCommand("DEMO221", empty_pose);
                } 
                else if (task.raw_cmd == "FIND_ACK_221") {
                    cout << ">>> [巡航搜索] 两个视角均未发现 ID=2 搜索终止。" << endl;
                    g_wf_find_failed = true; 
                }
            }
            return;
        }

        // 拦截小车完成信号，通知线程
        if (task.raw_cmd == "CHASSIS_DONE") {
            g_wf_chassis_done = true; 
            return;
        }

        if (task.raw_cmd.rfind("CHECK_H", 0) == 0) {
            handleClosedLoopCheck(task, raw_frame); return;
        }
        if (task.raw_cmd == "FIX_111" || task.raw_cmd == "FIX_131") {
            handleSingleAxisServo(task, raw_frame); return;
        }
        if (handleBlindOperations(task)) {
            return;
        }
        handleYoloAndPnP(task, raw_frame);
    }

    void processAutoCamera(Mat& raw_frame) {
        if (!g_auto_cam_running) return;
        // 每 8 帧提取一次画面并下发一次指令
        static int throttle_counter = 0;
        if (throttle_counter++ % 8 != 0) {
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
        int black_thresh = 110; 
        threshold(blurred, binary, black_thresh, 255, THRESH_BINARY_INV);
        
        // 填补胶带上可能存在的细小灰尘点或缝隙
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(binary, binary, MORPH_CLOSE, kernel);

        int left_y_sum = 0, left_count = 0, right_y_sum = 0, right_count = 0, center_y_sum = 0, center_count = 0;
        for (int x = 0; x < binary.cols; x += 10) {
            for (int y = 0; y < binary.rows; y++) {
                if (binary.at<uchar>(y, x) == 255) {
                    if (x < binary.cols * 0.2) { left_y_sum += y; left_count++; }
                    else if (x > binary.cols * 0.8) { right_y_sum += y; right_count++; }
                    else if (x > binary.cols * 0.4 && x < binary.cols * 0.6) { center_y_sum += y; center_count++; }
                    circle(raw_frame, Point(x, y + roi_rect.y), 2, Scalar(0, 255, 0), -1);
                    break; 
                }
            }
        }

        if (center_count > 5 && left_count > 5 && right_count > 5) {
            float center_y = (float)center_y_sum / center_count;
            float left_y = (float)left_y_sum / left_count; 
            float right_y = (float)right_y_sum / right_count;
            
            float target_dist_from_bottom = 72.0f; // 如果想让车板在画面里更靠下（镜头抬高），把这个值改小，单位：像素
            float target_y = roi_h - target_dist_from_bottom; 

            float err_tilt = center_y - target_y; 
            float err_pan = left_y - right_y;
            float Kp_tilt = 0.02f, Kp_pan = 0.02f;
            bool tilt_ok = abs(err_tilt) < 18.0f, pan_ok = abs(err_pan) < 18.0f;

            if (tilt_ok && pan_ok) {
                g_auto_cam_running = false;
                std::cout << "\n>>> [自适应云台] 校准完美！落点 -> Pan: " << g_cam_pan << " Tilt: " << g_cam_tilt << std::endl;
            } else {
                if (!tilt_ok) g_cam_tilt += Kp_tilt * err_tilt;
                if (!pan_ok)  g_cam_pan  += Kp_pan * err_pan;
                if (g_cam_pan < 20) g_cam_pan = 20; if (g_cam_pan > 70) g_cam_pan = 70;
                if (g_cam_tilt < 20.0f) g_cam_tilt = 20.0f; 
                if (g_cam_tilt > 60.0f) g_cam_tilt = 60.0f;

                //static int frame_counter = 0;
                //if (frame_counter++ % 4 == 0) { 
                //    if (g_serial_fd >= 0) {
                //        char buf[64]; sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt); write(g_serial_fd, buf, strlen(buf));
                //    }
                //    std::cout << "[伺服动态] 俯仰残差: " << err_tilt << " | 偏航残差: " << err_pan << " | 下发 CAM: " << g_cam_pan << " " << g_cam_tilt << std::endl;
                //}

                if (g_serial_fd >= 0) {
                    char buf[64]; 
                    sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt); 
                    write(g_serial_fd, buf, strlen(buf));
                }
                std::cout << "[伺服动态] 俯仰残差: " << err_tilt << " | 偏航残差: " << err_pan 
                          << " | 下发 CAM: " << g_cam_pan << " " << g_cam_tilt << std::endl;
            }
        } else {
            std::cout << "[云台伺服警告] 视野内丢失车板！正在自动向下低头寻找..." << std::endl;
            // 每次找不到时，让俯仰角向下低头 1.0 度 (度数越大越往下)
            g_cam_tilt += 1.0f; 
            if (g_cam_tilt > 50.0f) g_cam_tilt = 50.0f; // 物理限位保护
            // 同样需要降帧发送，避免指令发得太快导致舵机卡死或过度震荡
            static int search_frame_counter = 0;
            if (search_frame_counter++ % 4 == 0) { 
                if (g_serial_fd >= 0) {
                    char buf[64]; 
                    sprintf(buf, "CAM %.1f %.1f\r\n", g_cam_pan, g_cam_tilt); 
                    write(g_serial_fd, buf, strlen(buf));
                }
            }
        }
        putText(raw_frame, "AUTO_CAM ALIGNING...", Point(15, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 165, 255), 3);
        rectangle(raw_frame, roi_rect, Scalar(255, 0, 0), 2); 
    }

    void processArucoFix(Mat& raw_frame) {
        if (g_trigger_aruco_fix) {
            g_trigger_aruco_fix = false; 
            Mat clean_gray_for_aruco; cvtColor(raw_frame, clean_gray_for_aruco, COLOR_BGR2GRAY);
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
            cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
            parameters->minMarkerPerimeterRate = 0.01; 
            std::vector<int> marker_ids; std::vector<std::vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(clean_gray_for_aruco, dictionary, marker_corners, marker_ids, parameters);

            if (!marker_ids.empty()) {
                std::cout << "\n>>> [ArUco 标定成功] 成功锁定机械爪基准码 (ID: " << marker_ids[0] << ")" << std::endl;
                Point2f p1 = marker_corners[0][0], p2 = marker_corners[0][1], p3 = marker_corners[0][2], p4 = marker_corners[0][3]; 
                g_fixed_aruco_center = Point2f((p1.x+p2.x+p3.x+p4.x)/4.0f, (p1.y+p2.y+p3.y+p4.y)/4.0f);
            } else { std::cout << "\n>>> [ArUco 标定失败] 未找到基准码，请检查画面清晰度或是否被遮挡！" << std::endl; }
        }

        if (g_fixed_aruco_center.x >= 0 && g_fixed_aruco_center.y >= 0) {
            circle(raw_frame, g_fixed_aruco_center, 5, Scalar(255, 0, 255), -1);
            line(raw_frame, Point(g_fixed_aruco_center.x - 20, g_fixed_aruco_center.y), Point(g_fixed_aruco_center.x + 20, g_fixed_aruco_center.y), Scalar(255, 0, 255), 2);
            line(raw_frame, Point(g_fixed_aruco_center.x, g_fixed_aruco_center.y - 20), Point(g_fixed_aruco_center.x, g_fixed_aruco_center.y + 20), Scalar(255, 0, 255), 2);
        }
    }

    void renderOsd(Mat& raw_frame) {
        if (has_detection && current_yolo_res.detected) {
            int bottom_text_y = raw_frame.rows - 20;
            for (auto& obj : current_yolo_res.objects) {
                obj.bbox &= Rect(0, 0, raw_frame.cols, raw_frame.rows);
                if (obj.bbox.width < 15 || obj.bbox.height < 15) continue; 
                Scalar color( (obj.class_id * 80) % 255, (obj.class_id * 150) % 255, (obj.class_id * 200 + 100) % 255 );
                rectangle(raw_frame, obj.bbox, color, 2); 
                
                for (const auto& pt : obj.sub_centers) circle(raw_frame, pt, 4, Scalar(0, 0, 255), -1); 

                if (obj.has_refined_center && obj.corners_2d.size() == 4) {
                    Scalar pnp_box_color = Scalar(255, 0, 255); 
                    for (int i = 0; i < 4; i++) line(raw_frame, obj.corners_2d[i], obj.corners_2d[(i+1)%4], pnp_box_color, 2);
                    circle(raw_frame, obj.refined_center, 6, Scalar(0, 255, 0), 2);
                    line(raw_frame, obj.center, obj.refined_center, Scalar(0, 255, 255), 1);
                    char pose_text[256];
                    snprintf(pose_text, sizeof(pose_text), "ID:%d P(X:%.1f Y:%.1f D:%.1f)mm | R(Rx:%.1f Ry:%.1f Rz:%.1f)deg", obj.class_id, obj.tx, obj.ty, obj.tz, obj.rx, obj.ry, obj.rz);
                    putText(raw_frame, pose_text, Point(15, bottom_text_y), FONT_HERSHEY_SIMPLEX, 0.65, color, 2);
                    bottom_text_y -= 30; 
                }
                string label = "ID:" + to_string(obj.class_id) + " " + to_string(obj.confidence).substr(0,4);
                if (obj.class_id != 9) {
                    putText(raw_frame, label, Point(obj.bbox.x, max(obj.bbox.y - 5, 10)), FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                }
            }

            int pip_offset_y = 10;
            for (const auto& obj : current_yolo_res.objects) {
                if (!obj.roi_mask.empty()) {
                    Mat mask_bgr; cvtColor(obj.roi_mask, mask_bgr, COLOR_GRAY2BGR);
                    Rect pip_rect(10, pip_offset_y, mask_bgr.cols, mask_bgr.rows); pip_rect &= Rect(0, 0, raw_frame.cols, raw_frame.rows);
                    if(pip_rect.area() > 0) {
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
};

// ============================================================================
// [999] 主程序入口
// ============================================================================
int main() {
    SystemInit::initAll();
    VideoCapture cap = CameraManager::probeAndInit();
    SharedFrame shared_frame;
    CameraManager::startCaptureThread(cap, shared_frame);
    PilotCommunicator pilot_comm;
    CommunicationManager::startThreads();

    // 4. HTTP 推流服务器初始化 (供浏览器预览)
    HttpStreamServer stream_server(SystemConfig::HTTP_STREAM_PORT); // 默认 8080

    // 【修改】：使用正确的上位机通信端口 8000 和 8001
    PcProtocolServer pc_server(8000, 8001);

    VisionEngine engine(pilot_comm);
    const vector<int> encode_params = {IMWRITE_JPEG_QUALITY, SystemConfig::JPEG_QUALITY};

    // 为 HTTP 客户端定义局部 socket (原代码逻辑)
    int client_socket = -1;

    while (true) {
        // HTTP 依然保留非阻塞接受逻辑，防止没开网页时卡死
        if (client_socket < 0) {
            // 【修复】：使用封装好的非阻塞方法，而不是原生的 accept
            client_socket = stream_server.acceptClient(); 
        }

        Mat raw_frame;
        vector<uchar> buffer;
        buffer.reserve(128 * 1024);

        while (true) {
            if (!CameraManager::getLatestFrame(shared_frame, raw_frame, 50)) continue;

            DemoTask current_task = TaskManager::fetchTask();
            engine.processAutoCamera(raw_frame);
            if (current_task.pending) {
                engine.processTask(current_task, raw_frame); 
            }
            engine.processArucoFix(raw_frame);
            engine.renderOsd(raw_frame);

            // =======================================================
            // 【核心注入】：将画面喂给 PC 二进制高速图传
            // =======================================================
            pc_server.sendVideo(raw_frame);

            // 原有的 HTTP 推流逻辑
            if (client_socket >= 0) {
                if (!stream_server.sendFrame(client_socket, raw_frame, buffer, encode_params)) {
                    close(client_socket);
                    client_socket = -1; // 客户端断开，退回外层重新 accept
                    break;
                }
            }
        }
    }
    return 0;
}