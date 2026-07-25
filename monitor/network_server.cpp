/**
 * @file network_server.cpp
 * @brief 网络服务与上位机图传协议层
 */
#include "network_server.h"
#include "global_state.h"
#include "cmd_gateway.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <algorithm>
#define PROTOCOL_VIDEO_ENABLE
#include "protocol.hpp"

using namespace std;
using namespace cv;

HttpStreamServer::HttpStreamServer(int port)
{
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 3);
    cout << "[Monitor] 视觉推流就绪。浏览器访问: http://[本板IP]:" << port << endl;
}

int HttpStreamServer::acceptClient()
{
    int client_socket = accept(server_fd, nullptr, nullptr);
    if (client_socket < 0) return -1; 
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(client_socket, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout));
    cout << "\n检测到浏览器连接 开始推流" << endl;
    string header = "HTTP/1.0 200 OK\r\nConnection: close\r\nContent-Type: multipart/x-mixed-replace; boundary=--myboundary\r\n\r\n";
    send(client_socket, header.c_str(), header.size(), MSG_NOSIGNAL);
    return client_socket;
}

bool HttpStreamServer::sendFrame(int client_socket, const Mat &raw_frame, vector<uchar> &buffer, const vector<int> &encode_params)
{
    imencode(".jpg", raw_frame, buffer, encode_params);
    string chunk_header = "--myboundary\r\nContent-Type: image/jpeg\r\nContent-Length: " + to_string(buffer.size()) + "\r\n\r\n";
    if (send(client_socket, chunk_header.c_str(), chunk_header.size(), MSG_NOSIGNAL) < 0) return false;
    if (send(client_socket, buffer.data(), buffer.size(), MSG_NOSIGNAL) < 0) return false;
    if (send(client_socket, "\r\n", 2, MSG_NOSIGNAL) < 0) return false;
    return true;
}

// ---------------------------------------------------------
// PcProtocolServer 内部实现类定义
// ---------------------------------------------------------
class PcProtocolServerImpl {
public:
    int cmd_server_fd, video_server_fd;
    std::atomic<int> cmd_sock{-1};
    std::atomic<int> video_sock{-1};

    int createServer(int port)
    {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        bind(fd, (struct sockaddr *)&addr, sizeof(addr));
        listen(fd, 3);
        return fd;
    }

    void cmdAcceptLoop()
    {
        while (true)
        {
            int client = accept(cmd_server_fd, nullptr, nullptr);
            if (client >= 0)
            {
                if (cmd_sock >= 0) close(cmd_sock);
                cmd_sock = client;
                std::cout << "\n>>> [PC协议] 指令链路已连接! (来自上位机) <<<" << std::endl;
                std::thread(&PcProtocolServerImpl::cmdWorker, this, client).detach();
            }
        }
    }

    void videoAcceptLoop()
    {
        while (true)
        {
            int client = accept(video_server_fd, nullptr, nullptr);
            if (client >= 0)
            {
                int old_sock = video_sock.exchange(client);
                if (old_sock >= 0) close(old_sock);
                std::cout << ">>> [PC协议] 图传链路已连接! 开始高速推流 <<<" << std::endl;
            }
        }
    }

    void cmdWorker(int sock)
    {
        std::vector<uint8_t> buf;
        uint8_t tmp[4096];
        while (cmd_sock == sock)
        {
            int n = recv(sock, tmp, sizeof(tmp), 0);
            if (n <= 0) break;
            buf.insert(buf.end(), tmp, tmp + n);

            while (true)
            {
                protocol::ParsedFrame f;
                size_t consumed = protocol::consume(buf.data(), buf.size(), f);
                if (!consumed) break; 
                buf.erase(buf.begin(), buf.begin() + consumed);

                switch (f.cmd)
                {
                case protocol::CMD_HEARTBEAT:
                {
                    auto resp = protocol::build_resp_ok("alive");
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_VEHICLE_POS:
                {
                    float x, y, z, yaw;
                    protocol::parse_vehicle_pos(f, x, y, z, yaw);
                    std::cout << "[PC指令] 收到小车坐标: X=" << x << " Y=" << y << " Yaw=" << yaw << std::endl;
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_ARM_JOINTS:
                {
                    uint8_t num = protocol::parse_arm_num(f);
                    std::cout << "[PC指令] 收到 " << (int)num << " 个关节角度下发" << std::endl;
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_EXEC_PROGRAM:
                {
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
                case protocol::CMD_EMERGENCY:
                {
                    std::cout << "\n[PC指令] !!! 收到 EMERGENCY 急停指令 !!!" << std::endl;
                    if (g_serial_fd >= 0)
                    {
                        std::string send_str = "0\r\n"; 
                        write(g_serial_fd, send_str.c_str(), send_str.length());
                    }
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_TEXT:
                {
                    std::string text((const char *)f.payload, f.plen);
                    text.erase(std::remove(text.begin(), text.end(), '\r'), text.end());
                    text.erase(std::remove(text.begin(), text.end(), '\n'), text.end());

                    std::cout << "\n[PC指令] 收到文本命令: " << text << std::endl;
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
};

// PcProtocolServer 代理类实现 (PImpl idiom) 隐藏 protocol 头文件依赖
static PcProtocolServerImpl* pServerImpl = nullptr;

PcProtocolServer::PcProtocolServer(int cmd_port, int video_port)
{
    pServerImpl = new PcProtocolServerImpl();
    pServerImpl->cmd_server_fd = pServerImpl->createServer(cmd_port);
    pServerImpl->video_server_fd = pServerImpl->createServer(video_port);
    std::thread(&PcProtocolServerImpl::cmdAcceptLoop, pServerImpl).detach();
    std::thread(&PcProtocolServerImpl::videoAcceptLoop, pServerImpl).detach();
    std::cout << "[Monitor] PC 端二进制协议网关已启动 | 指令端口: " << cmd_port << " | 图传端口: " << video_port << std::endl;
}

void PcProtocolServer::sendVideo(const cv::Mat &frame)
{
    if (!pServerImpl) return;
    int sock = pServerImpl->video_sock.load();
    if (sock >= 0)
    {
        if (!video::send_jpeg(sock, frame, 60))
        {
            close(sock);
            pServerImpl->video_sock = -1;
        }
    }
}