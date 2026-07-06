/**
 * @file protocol.hpp
 * @brief 变长帧通信协议 —— 上位机 main.py 与下位机之间的二进制协议
 *
 * 帧格式 (大端):
 *   HEADER(2B) | CMD(1B) | PAYLOAD_LEN(2B) | PAYLOAD(N B) | CRC16(2B)
 *
 * 指令表:
 *   0x01 心跳      0x02 小车坐标(4f)   0x03 机械臂关节(1B计数+N×4B)
 *   0x04 执行程序  0x05 图传控制       0x06 急停
 *   0x80 应答 OK   0x81 应答 ERROR
 *
 * 使用:
 *   #include "protocol.hpp"
 *   Frame frame = protocol::build_resp_ok("OK");
 *   send(sock, frame.data(), frame.size(), 0);
 */

#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>
#include <cstring>
#include <vector>
#include <stdexcept>

namespace protocol
{

    // ==========================================
    // 常量
    // ==========================================
    constexpr uint16_t HEADER = 0x5A5A;
    constexpr int HEADER_SZ = 2;
    constexpr int CMD_SZ = 1;
    constexpr int LEN_SZ = 2;
    constexpr int CRC_SZ = 2;
    constexpr int FIXED_SZ = HEADER_SZ + CMD_SZ + LEN_SZ; // 5
    constexpr int MIN_FRAME = FIXED_SZ + CRC_SZ;          // 7

    // 指令
    constexpr uint8_t CMD_HEARTBEAT = 0x01;
    constexpr uint8_t CMD_VEHICLE_POS = 0x02;
    constexpr uint8_t CMD_ARM_JOINTS = 0x03;
    constexpr uint8_t CMD_EXEC_PROGRAM = 0x04;
    constexpr uint8_t CMD_VIDEO_CTRL = 0x05;
    constexpr uint8_t CMD_EMERGENCY = 0x06;
    constexpr uint8_t CMD_TEXT = 0x10;

    // 应答
    constexpr uint8_t RESP_OK = 0x80;
    constexpr uint8_t RESP_ERROR = 0x81;

    using Frame = std::vector<uint8_t>;

    // ==========================================
    // CRC-16/Modbus
    // ==========================================
    inline uint16_t crc16(const uint8_t *data, size_t len)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= data[i];
            for (int b = 0; b < 8; ++b)
                crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
        return crc;
    }

    // ==========================================
    // 大端写入辅助
    // ==========================================
    inline void write_be16(uint8_t *buf, uint16_t v)
    {
        buf[0] = v >> 8;
        buf[1] = v & 0xFF;
    }
    inline void write_be32(uint8_t *buf, uint32_t v)
    {
        buf[0] = v >> 24;
        buf[1] = (v >> 16) & 0xFF;
        buf[2] = (v >> 8) & 0xFF;
        buf[3] = v & 0xFF;
    }
    inline uint16_t read_be16(const uint8_t *buf) { return (buf[0] << 8) | buf[1]; }
    inline uint32_t read_be32(const uint8_t *buf)
    {
        return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    }

    // ==========================================
    // 帧构建
    // ==========================================
    inline Frame build(uint8_t cmd, const uint8_t *payload, uint16_t plen)
    {
        size_t total = FIXED_SZ + plen + CRC_SZ;
        Frame f(total);
        write_be16(f.data(), HEADER);
        f[HEADER_SZ] = cmd;
        write_be16(f.data() + HEADER_SZ + CMD_SZ, plen);
        if (plen)
            std::memcpy(f.data() + FIXED_SZ, payload, plen);
        uint16_t crc = crc16(f.data(), FIXED_SZ + plen);
        write_be16(f.data() + FIXED_SZ + plen, crc);
        return f;
    }

    inline Frame build(uint8_t cmd, const std::vector<uint8_t> &payload = {})
    {
        return build(cmd, payload.data(), (uint16_t)payload.size());
    }

    // ---- 便捷构建 ----
    inline Frame build_heartbeat() { return build(CMD_HEARTBEAT); }
    inline Frame build_emergency() { return build(CMD_EMERGENCY); }
    inline Frame build_video_ctrl(bool on, uint8_t quality = 60)
    {
        uint8_t p[] = {on ? (uint8_t)1 : (uint8_t)0, quality};
        return build(CMD_VIDEO_CTRL, p, 2);
    }
    inline Frame build_vehicle_pos(float x, float y, float z, float yaw)
    {
        uint8_t p[16];
        uint32_t raw;
        std::memcpy(&raw, &x, 4); // 补上字节数 4
        write_be32(p, raw);
        std::memcpy(&raw, &y, 4); // 补上字节数 4
        write_be32(p + 4, raw);
        std::memcpy(&raw, &z, 4); // 补上字节数 4
        write_be32(p + 8, raw);
        std::memcpy(&raw, &yaw, 4); // 补上字节数 4
        write_be32(p + 12, raw);
        return build(CMD_VEHICLE_POS, p, 16);
    }
    inline Frame build_arm_joints(const float *angles, uint8_t num)
    {
        std::vector<uint8_t> p(1 + num * 4);
        p[0] = num;
        for (uint8_t i = 0; i < num; ++i)
        {
            uint32_t raw;
            std::memcpy(&raw, angles + i, 4); // 补上字节数 4
            write_be32(p.data() + 1 + i * 4, raw);
        }
        return build(CMD_ARM_JOINTS, p);
    }
    inline Frame build_exec_program(uint8_t prog_id)
    {
        return build(CMD_EXEC_PROGRAM, &prog_id, 1);
    }
    inline Frame build_text(const char *text)
    {
        return build(CMD_TEXT, (const uint8_t *)text, (uint16_t)std::strlen(text));
    }
    inline Frame build_resp_ok(const char *msg = "OK")
    {
        return build(RESP_OK, (const uint8_t *)msg, (uint16_t)std::strlen(msg));
    }
    inline Frame build_resp_error(const char *msg = "ERROR")
    {
        return build(RESP_ERROR, (const uint8_t *)msg, (uint16_t)std::strlen(msg));
    }

    // ==========================================
    // 帧解析
    // ==========================================
    struct ParsedFrame
    {
        bool valid;
        uint8_t cmd;
        const uint8_t *payload;
        uint16_t plen;
    };

    /// 返回已解析帧；valid=false 表示数据不足/帧头错误/CRC 错误
    inline ParsedFrame parse(const uint8_t *data, size_t len)
    {
        ParsedFrame r{};
        if (len < MIN_FRAME)
            return r;
        if (read_be16(data) != HEADER)
            return r;
        r.cmd = data[HEADER_SZ];
        r.plen = read_be16(data + HEADER_SZ + CMD_SZ);
        size_t total = FIXED_SZ + r.plen + CRC_SZ;
        if (len < total)
            return r;
        uint16_t expected = read_be16(data + FIXED_SZ + r.plen);
        if (crc16(data, FIXED_SZ + r.plen) != expected)
            return r;
        r.valid = true;
        r.payload = data + FIXED_SZ;
        return r;
    }

    /// 从字节流中消费一帧；返回消费的字节数，0 表示解析失败
    inline size_t consume(const uint8_t *data, size_t len, ParsedFrame &out)
    {
        out = parse(data, len);
        return out.valid ? FIXED_SZ + out.plen + CRC_SZ : 0;
    }

    // ---- 解析便捷方法 ----
    inline void parse_vehicle_pos(const ParsedFrame &f, float &x, float &y, float &z, float &yaw)
    {
        if (f.plen >= 16)
        {
            uint32_t raw;
            raw = read_be32(f.payload);
            std::memcpy(&x, &raw, 4);
            raw = read_be32(f.payload + 4);
            std::memcpy(&y, &raw, 4);
            raw = read_be32(f.payload + 8);
            std::memcpy(&z, &raw, 4);
            raw = read_be32(f.payload + 12);
            std::memcpy(&yaw, &raw, 4);
        }
    }
    inline uint8_t parse_arm_num(const ParsedFrame &f) { return f.plen > 0 ? f.payload[0] : 0; }
    inline float parse_arm_joint(const ParsedFrame &f, int idx)
    {
        if (idx < 0 || (size_t)(1 + idx * 4 + 4) > f.plen)
            return 0;
        uint32_t raw = read_be32(f.payload + 1 + idx * 4);
        float v;
        std::memcpy(&v, &raw, 4);
        return v;
    }

} // namespace protocol

// ==========================================
// 图传发送器 (需 OpenCV + socket)
// ==========================================
#ifdef PROTOCOL_VIDEO_ENABLE

#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <vector>

namespace video
{

    /// 发送一帧 JPEG 到图传 socket
    /// 格式: SIZE(4B 大端) + JPEG_DATA
    /// 返回 true 表示发送成功
    inline bool send_jpeg(int sock, const cv::Mat &frame, int quality = 60)
    {
        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
        if (!cv::imencode(".jpg", frame, buf, params))
            return false;

        uint32_t size_be = htonl((uint32_t)buf.size());
        if (send(sock, &size_be, 4, 0) != 4)
            return false;
        if (send(sock, buf.data(), buf.size(), 0) != (ssize_t)buf.size())
            return false;
        return true;
    }

} // namespace video

#endif // PROTOCOL_VIDEO_ENABLE

#endif // PROTOCOL_HPP
