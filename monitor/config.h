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