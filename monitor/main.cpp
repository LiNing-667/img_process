/**
 * @file main.cpp
 * @brief 视觉监测上位机 (Brain Node)
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>

// 引入各个子模块
#include "config.h"         // 全局系统配置常量
#include "types.h"          // 系统核心数据结构定义
#include "global_state.h"   // 全局变量声明 (Extern)
#include "serial_driver.h"  // 系统信号和串口初始化
#include "camera_driver.h"  // 摄像头硬件抽象层与推流线程
#include "pilot_comm.h"     // 往 Pilot 板下发宏指令
#include "cmd_gateway.h"    // 指令网关与接收线程 (包含写定的工作流)
#include "network_server.h" // 网络服务与上位机图传协议层
#include "vision_engine.h"  //核心业务状态机与任务管线

using namespace cv;
using namespace std;

// ============================================================================
// [主程序入口]
// ============================================================================
int main()
{
    // 1. 系统与通信初始化
    SystemInit::initAll();

    // 2. 摄像头探测与启动
    VideoCapture cap = CameraManager::probeAndInit();
    SharedFrame shared_frame;
    CameraManager::startCaptureThread(cap, shared_frame);

    // 3. 通信网关启动
    PilotCommunicator pilot_comm;
    CommunicationManager::startThreads();

    // 4. 网络推流与协议服务初始化
    HttpStreamServer stream_server(SystemConfig::HTTP_STREAM_PORT);
    stream_server.startControlLoop(); // 启动手机浏览器遥控 HTTP 服务线程
    PcProtocolServer pc_server(8000, 8001);

    // 5. 实例化视觉业务大脑
    VisionEngine engine(pilot_comm);
    const vector<int> encode_params = {IMWRITE_JPEG_QUALITY, SystemConfig::JPEG_QUALITY};
    int client_socket = -1;

    // 6. 主事件流 (非阻塞)
    while (true)
    {
        Mat raw_frame;
        vector<uchar> buffer;
        buffer.reserve(128 * 1024);

        // [流水线 1] 抓最新帧
        if (!CameraManager::getLatestFrame(shared_frame, raw_frame, 50))
            continue;

        // [流水线 2] Http 断线重连/探测
        if (client_socket < 0)
        {
            client_socket = stream_server.acceptClient();
        }

        // [流水线 3] 获取任务并执行视觉伺服、检测、画 OSD 等
        DemoTask current_task = TaskManager::fetchTask();
        engine.processAutoCamera(raw_frame);

        if (current_task.pending)
        {
            engine.processTask(current_task, raw_frame);
        }

        engine.processArucoFix(raw_frame);
        engine.renderOsd(raw_frame);

        // [流水线 4] 将画面喂给 PC 二进制高速图传
        pc_server.sendVideo(raw_frame);

        // [流水线 5] 传统的 HTTP 网页推流
        if (client_socket >= 0)
        {
            if (!stream_server.sendFrame(client_socket, raw_frame, buffer, encode_params))
            {
                close(client_socket);
                client_socket = -1;
            }
        }
    }

    return 0;
}