/**
 * @file serial_router.h
 * @brief 监控端指令的路由与分发
 */
#include "serial_router.h"
#include "pilot_config.h"
#include "pilot_global.h"
#include "arm_controller.h"
#include "chassis_controller.h"
#include "demo_manager.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <algorithm>

int SerialRouter::initPort(const char *portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return -1;
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, SystemConfig::BAUD_RATE);
    cfsetispeed(&tty, SystemConfig::BAUD_RATE);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

void SerialRouter::dispatchCommand(const std::string &cmd_str)
{
    if (cmd_str.empty()) return;

    char cmd[16] = {0};
    float px = 0, py = 0, pz = 0, zx = 0, zy = 0, zz = 0, xx = 0, xy = 0, xz = 0, claw = 0;

    int num = sscanf(cmd_str.c_str(), "%15s %f %f %f %f %f %f %f %f %f %f",
                     cmd, &px, &py, &pz, &zx, &zy, &zz, &xx, &xy, &xz, &claw);

    std::string ack = "OK\r\n";

    for (int i = 0; cmd[i]; i++) {
        cmd[i] = toupper(cmd[i]);
    }

    if (strcmp(cmd, "ARM2") == 0) {
        std::cout << "\n>>> [Pilot] 接收云台平滑移动指令 (ARM2) | Pan=" << px << " Tilt=" << py << std::endl;
        g_arm.moveCameraSmooth(px, py);
    }
    else if (strcmp(cmd, "CAM") == 0) {
        g_arm.setCameraDirect(px, py);
    }
    else if (strcmp(cmd, "DEMO") == 0) {
        DemoManager::runDemoSequence();
    }
    else if (strcmp(cmd, "DEMO000") == 0) {
        std::cout << "[Pilot] DEMO000" << std::endl;
        DemoManager::executeDemo000(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO001") == 0) {
        std::cout << "[Pilot] DEMO001" << std::endl;
        DemoManager::executeDemo001(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO111") == 0) {
        std::cout << "[Pilot] DEMO111" << std::endl;
        DemoManager::executeDemo111(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO112") == 0) {
        std::cout << "[Pilot] DEMO112" << std::endl;
        DemoManager::executeDemo112(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO021") == 0) {
        std::cout << "[Pilot] DEMO021" << std::endl;
        DemoManager::executeDemo021(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO031") == 0) {
        std::cout << "[Pilot] DEMO021" << std::endl;
        DemoManager::executeDemo021(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO131") == 0) {
        std::cout << "[Pilot] DEMO131" << std::endl;
        DemoManager::executeDemo131(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO132") == 0) {
        std::cout << "[Pilot] DEMO132" << std::endl;
        DemoManager::executeDemo132(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO121") == 0) {
        std::cout << "[Pilot] DEMO131" << std::endl;
        DemoManager::executeDemo131(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO122") == 0) {
        std::cout << "[Pilot] DEMO132" << std::endl;
        DemoManager::executeDemo132(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO041") == 0) {
        std::cout << "[Pilot] DEMO041" << std::endl;
        DemoManager::executeDemo041(px, py, pz);
    }
    else if (strcmp(cmd, "DO001") == 0) {
        std::cout << "[Pilot] DO001" << std::endl;
        DemoManager::executeDo001();
    }
    else if (strcmp(cmd, "DO031") == 0) {
        std::cout << "[Pilot] DO031" << std::endl;
        DemoManager::executeDo031();
    }
    else if (strcmp(cmd, "DO002") == 0) {
        std::cout << "[Pilot] DO002" << std::endl;
        DemoManager::executeDo002();
    }
    else if (strcmp(cmd, "DEMO102") == 0) {
        std::cout << "[Pilot] DEMO102" << std::endl;
        DemoManager::executeDemo102(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO002") == 0) {
        std::cout << "[Pilot] DEMO002" << std::endl;
        DemoManager::executeDemo002(px, py, pz);
    }
    else if (strcmp(cmd, "DEMO091") == 0) {
        std::cout << "[Pilot] DEMO091" << std::endl;
        DemoManager::executeDemo091(px, py, pz);
    }
    else if (strcmp(cmd, "CHASSIS_MOVE") == 0) {
        std::cout << "\n>>> [Pilot] 收到find移动指令 CHASSIS_MOVE" << std::endl;
        DemoManager::executeChassisAutoMove(px, py);
    }
    else if (num == 11 && (strcmp(cmd, "ARM0") == 0 || strcmp(cmd, "ARM1") == 0)) {
        int arm_id = (strcmp(cmd, "ARM0") == 0) ? 0 : 1;
        std::cout << "\n>>> [Pilot] 接收单步: " << cmd << " | X=" << px << " Y=" << py << " Z=" << pz << std::endl;
        g_arm.moveSmooth(arm_id, px, py, pz, zx, zy, zz, xx, xy, xz);
    }
    else if (num >= 3 && strcmp(cmd, "CH") == 0) {
        int channel = (int)px;
        if (channel >= 0 && channel <= 15) {
            int target_arm = (channel >= 7) ? 1 : 0;
            float calibrated_angle = py;
            if (target_arm == 0) {
                if (channel == 0) calibrated_angle = py + 20.0f;
                else if (channel == 1) calibrated_angle = py + 30.0f;
                else if (channel == 2) calibrated_angle = py + 20.0f;
                else if (channel == 3) calibrated_angle = py + 20.0f;
                else if (channel == 4) calibrated_angle = py + 12.0f;
                else if (channel == 5) calibrated_angle = py + 13.0f;
            }
            else if (target_arm == 1) {
                if (channel == 9) calibrated_angle = py + 18.0f;
                else if (channel == 10) calibrated_angle = py + 18.0f;
                else if (channel == 11) calibrated_angle = py + 18.0f;
                else if (channel == 12) calibrated_angle = py + 18.0f;
                else if (channel == 13) calibrated_angle = py + 18.0f;
                else if (channel == 14) calibrated_angle = py - 55.0f;
                else if (channel == 7) {
                    calibrated_angle = py;
                    g_arm.notifyCameraManualSet(7, calibrated_angle); 
                }
                else if (channel == 8) {
                    calibrated_angle = py;
                    g_arm.notifyCameraManualSet(8, calibrated_angle); 
                }
            }
            g_arm.setServoAngle(target_arm, channel, calibrated_angle);
            std::cout << "[Pilot] 收到直控指令 -> " << cmd << " " << channel << " 角度:" << py << " (已映射为:" << calibrated_angle << ")" << std::endl;
        }
    }
    else if (strcmp(cmd, "MR") == 0) {
        g_car.resetPosition();
        std::cout << "[Pilot] 底盘里程计已归零。" << std::endl;
    }
    else if (strcmp(cmd, "0") == 0) {
        g_car.emergencyStop();
        std::cout << "[Pilot] 底盘急停锁死！" << std::endl;
    }
    else if (strcmp(cmd, "M") == 0) {
        g_car.setAbsoluteTarget(px, py);
        std::cout << "[Pilot] 绝对移动目标 -> X:" << px << "cm, Y:" << py << "cm" << std::endl;
    }
    else if (strcmp(cmd, "MW") == 0) {
        g_car.moveRelative(px, 0);
        std::cout << "[Pilot] 相对移动 -> 前进 " << px << " cm" << std::endl;
    }
    else if (strcmp(cmd, "MS") == 0) {
        g_car.moveRelative(-px, 0);
        std::cout << "[Pilot] 相对移动 -> 后退 " << px << " cm" << std::endl;
    }
    else if (strcmp(cmd, "MD") == 0) {
        g_car.moveRelative(0, px);
        std::cout << "[Pilot] 相对移动 -> 向右平移 " << px << " cm" << std::endl;
    }
    else if (strcmp(cmd, "MA") == 0) {
        g_car.moveRelative(0, -px);
        std::cout << "[Pilot] 相对移动 -> 向左平移 " << px << " cm" << std::endl;
    }
    else if (strcmp(cmd, "MQ") == 0) {
        float deg = (num >= 2) ? px : 90.0f;
        g_car.turnRelative(-deg);
        std::cout << "[Pilot] 车身原地左转 " << deg << " 度" << std::endl;
    }
    else if (strcmp(cmd, "ME") == 0) {
        float deg = (num >= 2) ? px : 90.0f;
        g_car.turnRelative(deg);
        std::cout << "[Pilot] 车身原地右转 " << deg << " 度" << std::endl;
    }
    else if (strcmp(cmd, "TEST") == 0) {
        std::cout << "[Pilot] 收到 TEST 指令，准备执行物理层盲走验证..." << std::endl;
        g_car.blindTest();
    }
    else if (strcmp(cmd, "VEL") == 0) {
        if (px == 0 && py == 0 && pz == 0)
            g_car.stopVelocity();
        else
            g_car.setVelocity(px, py, pz);
    }
    else {
        std::cout << "[Pilot 警告] 收到未知或格式错误的指令: [" << cmd_str << "] 解析出CMD:[" << cmd << "]" << std::endl;
    }

    write(fd_, ack.c_str(), ack.length());
    std::cout << std::flush;
}

SerialRouter::SerialRouter() : fd_(-1) {}

bool SerialRouter::start()
{
    fd_ = initPort(SystemConfig::SERIAL_PORT_MONITOR);
    g_monitor_fd = fd_;
    return fd_ >= 0;
}

void SerialRouter::spinOnce()
{
    char buffer[256];
    int valread = read(fd_, buffer, sizeof(buffer) - 1);
    if (valread > 0)
    {
        buffer[valread] = '\0';
        rx_buffer_ += buffer;

        size_t pos;
        while ((pos = rx_buffer_.find_first_of("\r\n")) != std::string::npos)
        {
            std::string line = rx_buffer_.substr(0, pos);
            rx_buffer_.erase(0, pos + 1);
            dispatchCommand(line);
        }
    }
    else
    {
        usleep(10000);
    }
}