/**
 * @file config.h
 * @brief 全局系统配置常量
 */
#pragma once

namespace SystemConfig
{
    const char SERIAL_PORT[] = "/dev/ttyS1";
    const int CAM_WIDTH = 1280;
    const int CAM_HEIGHT = 720;
    const int CAM_FPS = 8;
    const int JPEG_QUALITY = 50;
    const int HTTP_STREAM_PORT = 8080;
    const float CONF_THRESH_TARGET = 0.4f;
    const float CONF_THRESH_OTHER = 0.03f;
}

// ==========================================================
// 摄像头舵机默认角度 (通道7=Pan, 通道8=Tilt)
// 所有其他角度都基于这两个值相对计算，不再另写硬编码
// ==========================================================
#define CAM_DEFAULT_PAN 45.0f      // CH7 默认水平角度
#define CAM_DEFAULT_TILT 48.0f     // CH8 默认俯仰角度
#define CAM_FIND_TILT_OFFSET 15.0f // find/恢复时 CH8 相对抬高的度数