/**
 * @file network_server.h
 * @brief 网络服务与上位机图传协议层
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class HttpStreamServer {
public:
    int server_fd;
    HttpStreamServer(int port);
    int acceptClient();
    bool sendFrame(int client_socket, const cv::Mat &raw_frame, std::vector<uchar> &buffer, const std::vector<int> &encode_params);
};

class PcProtocolServer {
public:
    PcProtocolServer(int cmd_port, int video_port);
    void sendVideo(const cv::Mat &frame);
};