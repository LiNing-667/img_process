/**
 * @file network_server.cpp
 * @brief 网络服务与上位机图传协议层
 */
#include "network_server.h"
#include "global_state.h"
#include "cmd_gateway.h"
#include "monitor_log.h"
#include <iostream>
#include <mutex>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
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

HttpStreamServer::HttpStreamServer(int port) : port_(port)
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
    monitor_log << "[Monitor] 视觉推流就绪。浏览器访问: http://[本板IP]:" << port << std::endl;
}

// 非阻塞获取一个 /stream 推流 socket（由手机遥控线程 accept 后转发）
int HttpStreamServer::acceptClient()
{
    std::lock_guard<std::mutex> lock(stream_mtx_);
    if (stream_sock_ < 0)
        return -1;
    int s = stream_sock_;
    stream_sock_ = -1;
    return s;
}

// 启动手机浏览器遥控服务线程
void HttpStreamServer::startControlLoop()
{
    std::thread(&HttpStreamServer::controlLoop, this).detach();
}

void HttpStreamServer::controlLoop()
{
    monitor_log << "[Monitor] 手机遥控服务已启动: 浏览器打开 http://[本板IP]:" << port_ << "/" << std::endl;
    while (true)
    {
        int client = accept(server_fd, nullptr, nullptr);
        if (client < 0)
        {
            usleep(10000);
            continue;
        }

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        std::string request, path;
        int type = readHttpRequest(client, request, path);

        if (type == 1) // /stream -> 转交给主循环推流
        {
            monitor_log << "\n检测到浏览器连接 开始推流" << std::endl;
            std::string header = "HTTP/1.0 200 OK\r\nConnection: close\r\nContent-Type: multipart/x-mixed-replace; boundary=--myboundary\r\n\r\n";
            send(client, header.c_str(), header.size(), MSG_NOSIGNAL);
            {
                std::lock_guard<std::mutex> lock(stream_mtx_);
                stream_sock_ = client;
            }
        }
        else if (type == 2) // /ctrl 速度指令
        {
            handleControlCmd(client, request);
            close(client);
        }
        else if (type == 3) // /cmd 文本指令
        {
            handleTextCmd(client, request);
            close(client);
        }
        else // 默认: 控制页面
        {
            sendControlPage(client);
            close(client);
        }
    }
}

// 读取并解析 HTTP 请求首行
int HttpStreamServer::readHttpRequest(int fd, std::string &request, std::string &path)
{
    char tmp[1024];
    while (request.size() < 8192)
    {
        int n = recv(fd, tmp, sizeof(tmp), 0);
        if (n > 0)
        {
            request.append(tmp, n);
            if (request.find("\r\n\r\n") != std::string::npos)
                break;
        }
        else
        {
            break; // 0=断开, -1=超时/错误
        }
    }

    path = "/";
    size_t nl = request.find("\r\n");
    if (nl == std::string::npos)
        return 0;
    std::string req_line = request.substr(0, nl);
    size_t p1 = req_line.find(' ');
    if (p1 == std::string::npos)
        return 0;
    size_t p2 = req_line.find(' ', p1 + 1);
    path = (p2 != std::string::npos) ? req_line.substr(p1 + 1, p2 - p1 - 1)
                                     : req_line.substr(p1 + 1);

    if (path.find("/stream") == 0)
        return 1;
    if (path.find("/ctrl") == 0)
        return 2;
    if (path.find("/cmd") == 0)
        return 3;
    return 0;
}

static std::string urlDecode(const std::string &in); // 前向声明 (定义在下方辅助函数区)

// 处理 /ctrl?vel=vx,vy,vz -> 通过串口下发 VEL 指令给 Pilot
void HttpStreamServer::handleControlCmd(int fd, const std::string &request)
{
    float vx = 0, vy = 0, vz = 0;
    size_t vp = request.find("vel=");
    if (vp != std::string::npos)
    {
        size_t ve = request.find_first_of(" \r\n&", vp + 4);
        std::string val = (ve == std::string::npos)
                              ? request.substr(vp + 4)
                              : request.substr(vp + 4, ve - vp - 4);
        val = urlDecode(val); // 关键：还原 %2C 等编码，否则按逗号分割失败
        sscanf(val.c_str(), "%f,%f,%f", &vx, &vy, &vz);
    }

    // 防呆限幅，防止异常输入打爆电机
    auto clampf = [](float v, float lo, float hi)
    { return v < lo ? lo : (v > hi ? hi : v); };
    vx = clampf(vx, -800.0f, 800.0f);
    vy = clampf(vy, -800.0f, 800.0f);
    vz = clampf(vz, -800.0f, 800.0f);

    // 通过串口下发 VEL 指令到 Pilot
    if (g_serial_fd >= 0)
    {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "VEL %.2f %.2f %.2f\r\n", vx, vy, vz);
        write(g_serial_fd, cmd, strlen(cmd));
        if (vx != 0 || vy != 0 || vz != 0)
            monitor_log << "[手机遥控] VEL " << vx << " " << vy << " " << vz << std::endl;
    }

    std::string resp =
        "HTTP/1.0 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: 2\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n"
        "OK";
    send(fd, resp.c_str(), resp.size(), MSG_NOSIGNAL);
}

// ==========================================================
// 手机遥控页面资源 (独立文件: web/index.html)
// ==========================================================
// 读取文本文件到 string；失败返回空串
static std::string readFileToString(const std::string &path)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs)
        return "";
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

// 取路径所在目录 (不含末尾 '/')；无分隔符时返回 "."
static std::string dirOf(const std::string &p)
{
    size_t slash = p.find_last_of('/');
    if (slash == std::string::npos)
        return ".";
    return p.substr(0, slash);
}

// 取可执行文件所在目录 (Linux: /proc/self/exe)；失败返回空串
static std::string exeDir()
{
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0)
        return "";
    buf[n] = '\0';
    return dirOf(std::string(buf));
}

// 简单 URL 解码 (%XX 与 '+')
static int hexVal(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}
static std::string urlDecode(const std::string &in)
{
    std::string out;
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        if (in[i] == '+')
            out += ' ';
        else if (in[i] == '%' && i + 2 < in.size())
        {
            int h = hexVal(in[i + 1]), l = hexVal(in[i + 2]);
            if (h >= 0 && l >= 0)
            {
                out += (char)((h << 4) | l);
                i += 2;
            }
            else
                out += in[i];
        }
        else
            out += in[i];
    }
    return out;
}

// 返回手机遥控控制页面 (从独立文件 web/index.html 读取)
void HttpStreamServer::sendControlPage(int fd)
{
    // 查找顺序: 1) 二进制同目录 (推荐部署方式) 2) __FILE__ 推导的源码 web/ 3) 常见相对路径
    std::string html;
    std::vector<std::string> candidates;
    const std::string exe = exeDir();
    if (!exe.empty())
    {
        candidates.push_back(exe + "/index.html");
        candidates.push_back(exe + "/web/index.html");
    }
    const std::string webBase = dirOf(__FILE__) + "/web/index.html";
    candidates.push_back(webBase);
    candidates.push_back("web/index.html");
    candidates.push_back("monitor/web/index.html");
    candidates.push_back("../monitor/web/index.html");

    for (const auto &c : candidates)
    {
        html = readFileToString(c);
        if (!html.empty())
            break;
    }

    if (html.empty())
    {
        monitor_log << "[Monitor] ⚠ 未找到控制页面 web/index.html (搜索: " << webBase << " 等)，返回占位页" << std::endl;
        html = "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'>"
               "<title>未找到页面</title></head>"
               "<body style='background:#111;color:#eee;font-family:sans-serif;padding:20px'>"
               "<h3>未找到控制页面 web/index.html</h3>"
               "<p>请确保程序工作目录位于 img_process/ 或 img_process/build/ 下，"
               "并确认 monitor/web/index.html 存在。</p></body></html>";
    }

    std::string header =
        "HTTP/1.0 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Content-Length: " +
        std::to_string(html.size()) + "\r\n"
                                      "\r\n";
    send(fd, header.c_str(), header.size(), MSG_NOSIGNAL);
    send(fd, html.c_str(), html.size(), MSG_NOSIGNAL);
}

// 处理 /cmd?text=XXX -> 调用 processTextCommand 执行任意文本指令
void HttpStreamServer::handleTextCmd(int fd, const std::string &request)
{
    std::string text;
    size_t tp = request.find("text=");
    if (tp != std::string::npos)
    {
        size_t te = request.find_first_of(" \r\n&", tp + 5);
        text = (te == std::string::npos) ? request.substr(tp + 5)
                                         : request.substr(tp + 5, te - tp - 5);
        text = urlDecode(text);
    }

    if (!text.empty())
    {
        monitor_log << "\n[手机指令] 收到文本指令: " << text << std::endl;
        processTextCommand(text);
    }

    std::string resp =
        "HTTP/1.0 200 OK\r\n"
        "Connection: close\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: 2\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n"
        "OK";
    send(fd, resp.c_str(), resp.size(), MSG_NOSIGNAL);
}

bool HttpStreamServer::sendFrame(int client_socket, const Mat &raw_frame, vector<uchar> &buffer, const vector<int> &encode_params)
{
    imencode(".jpg", raw_frame, buffer, encode_params);
    string chunk_header = "--myboundary\r\nContent-Type: image/jpeg\r\nContent-Length: " + to_string(buffer.size()) + "\r\n\r\n";
    if (send(client_socket, chunk_header.c_str(), chunk_header.size(), MSG_NOSIGNAL) < 0)
        return false;
    if (send(client_socket, buffer.data(), buffer.size(), MSG_NOSIGNAL) < 0)
        return false;
    if (send(client_socket, "\r\n", 2, MSG_NOSIGNAL) < 0)
        return false;
    return true;
}

// ---------------------------------------------------------
// PcProtocolServer 内部实现类定义
// ---------------------------------------------------------
class PcProtocolServerImpl
{
public:
    int cmd_server_fd, video_server_fd;
    std::atomic<int> cmd_sock{-1};
    std::atomic<int> video_sock{-1};

    // 给已接受的连接 socket 设置发送超时：断联/对端不读时 send 返回 -1，而不是无限阻塞
    static void setSendTimeout(int sock, int sec = 1)
    {
        struct timeval tv;
        tv.tv_sec = sec;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

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
                setSendTimeout(client); // 断联时 send 不再无限阻塞
                if (cmd_sock >= 0)
                    close(cmd_sock);
                cmd_sock = client;
                monitor_log << "\n>>> [PC协议] 指令链路已连接! (来自上位机) <<<" << std::endl;
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
                setSendTimeout(client, 2); // 图传帧大，超时放宽到 2s
                int old_sock = video_sock.exchange(client);
                if (old_sock >= 0)
                    close(old_sock);
                monitor_log << ">>> [PC协议] 图传链路已连接! 开始高速推流 <<<" << std::endl;
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
            if (n <= 0)
                break;
            buf.insert(buf.end(), tmp, tmp + n);

            while (true)
            {
                protocol::ParsedFrame f;
                size_t consumed = protocol::consume(buf.data(), buf.size(), f);
                if (!consumed)
                    break;
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
                    monitor_log << "[PC指令] 收到小车坐标: X=" << x << " Y=" << y << " Yaw=" << yaw << std::endl;
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_ARM_JOINTS:
                {
                    uint8_t arm_id = protocol::parse_arm_armid(f);
                    uint8_t num = (f.plen >= 2) ? f.payload[1] : 0;
                    std::vector<float> angles;
                    for (uint8_t i = 0; i < num && i < 6; ++i)
                        angles.push_back(protocol::parse_arm_joint_arm(f, i));
                    if (angles.size() >= 6)
                    {
                        monitor_log << "[PC指令] 收到 ARM" << (int)arm_id << " 关节角度: "
                                    << angles[0] << " " << angles[1] << " " << angles[2] << " "
                                    << angles[3] << " " << angles[4] << " " << angles[5] << std::endl;
                        // 转发给 Pilot 直接关节角控制（平滑移动）
                        if (g_serial_fd >= 0)
                        {
                            char buf[128];
                            snprintf(buf, sizeof(buf), "JNT %d %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
                                     arm_id, angles[0], angles[1], angles[2], angles[3], angles[4], angles[5]);
                            write(g_serial_fd, buf, strlen(buf));
                        }
                    }
                    else
                    {
                        monitor_log << "[PC指令] 收到 ARM" << (int)arm_id << " 关节角度 " << (int)num << " 个 (不足6，忽略)" << std::endl;
                    }
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_EXEC_PROGRAM:
                {
                    uint8_t prog_id = f.payload[0];
                    char cmd_buf[32];
                    sprintf(cmd_buf, "DEMO%03d", prog_id);
                    monitor_log << "[PC指令] 触发系统动作流水线: " << cmd_buf << std::endl;

                    DemoTask t;
                    t.raw_cmd = cmd_buf;
                    t.class_id = 0;
                    bool accepted = task_try_submit(t); // 任务槽忙则拒绝，避免覆盖
                    auto resp = accepted ? protocol::build_resp_ok()
                                         : protocol::build_resp_error("task busy");
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_EMERGENCY:
                {
                    monitor_log << "\n[PC指令] !!! 收到 EMERGENCY 急停指令 !!!" << std::endl;
                    if (g_serial_fd >= 0)
                    {
                        std::string send_str = "0\r\n";
                        write(g_serial_fd, send_str.c_str(), send_str.length());
                    }
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_VIDEO_CTRL:
                {
                    bool on = (f.plen >= 1) ? (f.payload[0] != 0) : true;
                    int quality = (f.plen >= 2) ? (int)f.payload[1] : g_video_quality.load();
                    if (quality < 1)
                        quality = 1;
                    if (quality > 100)
                        quality = 100;
                    g_video_stream_on.store(on);
                    g_video_quality.store(quality);
                    monitor_log << "[PC指令] 图传控制: " << (on ? "开启" : "关闭")
                                << " 质量=" << quality << std::endl;
                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                case protocol::CMD_TEXT:
                {
                    std::string text((const char *)f.payload, f.plen);
                    text.erase(std::remove(text.begin(), text.end(), '\r'), text.end());
                    text.erase(std::remove(text.begin(), text.end(), '\n'), text.end());

                    monitor_log << "\n[PC指令] 收到文本命令: " << text << std::endl;
                    processTextCommand(text);

                    auto resp = protocol::build_resp_ok();
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                default:
                {
                    monitor_log << "[PC协议] 未知指令 0x" << std::hex << (int)f.cmd << std::dec
                                << "，已拒绝" << std::endl;
                    auto resp = protocol::build_resp_error("unknown cmd");
                    send(sock, resp.data(), resp.size(), MSG_NOSIGNAL);
                    break;
                }
                }
            }
        }
        close(sock);
        if (cmd_sock == sock)
            cmd_sock = -1;
        monitor_log << "[PC协议] 指令链路断开" << std::endl;
    }
};

// PcProtocolServer 代理类实现 (PImpl idiom) 隐藏 protocol 头文件依赖
static PcProtocolServerImpl *pServerImpl = nullptr;

PcProtocolServer::PcProtocolServer(int cmd_port, int video_port)
{
    pServerImpl = new PcProtocolServerImpl();
    pServerImpl->cmd_server_fd = pServerImpl->createServer(cmd_port);
    pServerImpl->video_server_fd = pServerImpl->createServer(video_port);
    std::thread(&PcProtocolServerImpl::cmdAcceptLoop, pServerImpl).detach();
    std::thread(&PcProtocolServerImpl::videoAcceptLoop, pServerImpl).detach();
    monitor_log << "[Monitor] PC 端二进制协议网关已启动 | 指令端口: " << cmd_port << " | 图传端口: " << video_port << std::endl;
}

void PcProtocolServer::sendVideo(const cv::Mat &frame)
{
    if (!pServerImpl)
        return;
    if (!g_video_stream_on.load())
        return; // 图传总开关 (CMD_VIDEO_CTRL)
    int sock = pServerImpl->video_sock.load();
    if (sock >= 0)
    {
        if (!video::send_jpeg(sock, frame, g_video_quality.load()))
        {
            close(sock);
            pServerImpl->video_sock = -1;
        }
    }
}

// ==========================================================
// 上位机下行文本通道 (CMD_DOWNLINK_MSG)
// 供 monitor_log 统一日志转发使用；PC 未连接时自动忽略。
// 加互斥锁保证多线程并发下帧不会在半路交错，避免破坏 PC 端拆帧。
// ==========================================================
static std::mutex g_downlink_mtx;

// 完整发送一帧；失败（断联/超时）返回 false，由调用方负责清理 socket
static bool sendFrameAll(int sock, const std::vector<uint8_t> &frame)
{
    size_t sent = 0;
    while (sent < frame.size())
    {
        ssize_t n = send(sock, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
        if (n <= 0)
            return false;
        sent += (size_t)n;
    }
    return true;
}

// 发送失败时关闭 socket 并清除全局句柄，避免死连接被反复重试
static void dropCmdSocket(int sock)
{
    if (sock < 0)
        return;
    close(sock);
    if (pServerImpl && pServerImpl->cmd_sock == sock)
        pServerImpl->cmd_sock = -1;
}

void pc_send_downlink(const std::string &text)
{
    if (!pServerImpl)
        return;
    int sock = pServerImpl->cmd_sock.load();
    if (sock < 0)
        return;
    auto frame = protocol::build_downlink_msg(text.c_str());
    std::lock_guard<std::mutex> lock(g_downlink_mtx);
    if (!sendFrameAll(sock, frame))
        dropCmdSocket(sock);
}

void pc_send_arm_joints(uint8_t arm_id, const std::vector<float> &angles)
{
    if (!pServerImpl)
        return;
    int sock = pServerImpl->cmd_sock.load();
    if (sock < 0)
        return;
    // 补齐 6 轴
    std::vector<float> a(6, 0.0f);
    for (size_t i = 0; i < angles.size() && i < 6; ++i)
        a[i] = angles[i];
    auto frame = protocol::build_arm_joints_arm(arm_id, a.data(), 6);
    std::lock_guard<std::mutex> lock(g_downlink_mtx);
    if (!sendFrameAll(sock, frame))
        dropCmdSocket(sock);
}

void pc_send_vehicle_pos(float x, float y, float z, float yaw)
{
    if (!pServerImpl)
        return;
    int sock = pServerImpl->cmd_sock.load();
    if (sock < 0)
        return;
    auto frame = protocol::build_vehicle_pos(x, y, z, yaw);
    std::lock_guard<std::mutex> lock(g_downlink_mtx);
    if (!sendFrameAll(sock, frame))
        dropCmdSocket(sock);
}