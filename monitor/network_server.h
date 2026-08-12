/**
 * @file network_server.h
 * @brief 网络服务与上位机图传协议层
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <mutex>

class HttpStreamServer
{
public:
    int server_fd;
    HttpStreamServer(int port);
    // 非阻塞获取一个 /stream 推流 socket（由手机遥控线程 accept 后转发）；无连接时返回 -1
    int acceptClient();
    bool sendFrame(int client_socket, const cv::Mat &raw_frame, std::vector<uchar> &buffer, const std::vector<int> &encode_params);
    // 启动手机浏览器遥控 HTTP 服务线程（处理 "/" 控制页面 与 "/ctrl" 速度指令）
    void startControlLoop();

private:
    int port_;
    std::mutex stream_mtx_;
    int stream_sock_ = -1;

    void controlLoop();
    // 读取 HTTP 请求。返回: 1=/stream 2=/ctrl 3=/cmd 0=其它(页面/未知)
    int readHttpRequest(int fd, std::string &request, std::string &path);
    void sendControlPage(int fd);
    void handleControlCmd(int fd, const std::string &request);
    void handleTextCmd(int fd, const std::string &request);
};

class PcProtocolServer
{
public:
    PcProtocolServer(int cmd_port, int video_port);
    void sendVideo(const cv::Mat &frame);
};