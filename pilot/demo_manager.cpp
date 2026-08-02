/**
 * @file demo_manager.cpp
 * @brief 宏指令动作组
 */
#include "demo_manager.h"
#include "arm_controller.h"
#include "chassis_controller.h"
#include "pilot_global.h"
#include "pilot_config.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <unistd.h>
#include <vector>

namespace DemoManager
{

    float g_cache_091_px = 0.0f;
    float g_cache_091_py = 0.0f;
    float g_cache_091_pz = 0.0f;
    bool g_has_cache_091 = false;
    // ==============小车===============//

    void executeChassisAutoMove(float px, float py)
    {
        std::thread([=]()
                    {
                        // 根据左臂 PnP 期望的最终抓取坐标
                        float target_x = -15.6f;
                        float target_y = 15.0f;
                        float forward_cm = target_x - px;
                        float right_cm = py - target_y;
                        std::cout << "\n>>> [底盘追踪] 当前物体: X=" << px << ", Y=" << py << std::endl;
                        std::cout << ">>> [底盘追踪] 期望到达: X=" << target_x << ", Y=" << target_y << std::endl;
                        std::cout << ">>> [底盘追踪] 执行相对移动 -> 前进 " << forward_cm << " cm, 向右 " << right_cm << " cm" << std::endl;
                        g_car.moveRelative(forward_cm, right_cm);
                        // 估算行驶时间，让底盘有充足的时间跑完
                        // 假设小车均速 15cm/s，外加 1.5 秒刹车缓冲时间
                        float max_dist = std::max(std::abs(forward_cm), std::abs(right_cm));
                        float move_time = max_dist / 15.0f + 1.5f;
                        usleep((int)(move_time * 1000000));

                        std::cout << ">>> [底盘追踪] 移动结束，向大脑回传完成信号..." << std::endl;
                        sendToMonitor("CHASSIS_DONE\r\n"); // 报告移动完毕，触发 Monitor 的下一步状态
                    })
            .detach();
    }

    //===============机械臂================//
    void runDemoSequence()
    {
        std::cout << "\n>>> [DEMO]  <<<" << std::endl;
        g_arm.setServoAngle(0, 6, Arm0_open);
        usleep(100000);
        g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
        usleep(1000000);
        g_arm.setServoAngle(1, 15, Arm1_open);
        usleep(1000000);
        g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
        usleep(1000000);
        std::cout << ">>> 测试序列执行完毕！ <<<\n"
                  << std::endl;
    }

    //用于测试标定
    void executeDemo130(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        g_arm.moveSmooth(1, px, py , 3 , 0 , 0 , -1 , -1 , 0 , 0);
                        usleep(2000000);
                    })
            .detach();
    }

    void executeDemo030(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        g_arm.moveSmooth(0, px, py , 3 , 0 , 0 , -1 , -1 , 0 , 0);
                        usleep(2000000);
                    })
            .detach();
    }

    //正式的
    void executeDemo000(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(500000);
                        // g_arm.moveSmooth(0, px-2 , py + 5.2f, -4.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                        g_arm.moveSmooth(0, px-1.0 , py + 6.0f, 1.0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px-1.0 , py + 6.0f, -2.8, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(800000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1000000);
                        // g_arm.moveSmooth(0, px-2 , py + 5.2f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000);
                        g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                        sendToMonitor("DEMO_DONE\r\n");
                    })
            .detach();
    }

    void executeDemo111(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        // 实际要去的三维坐标点
                        float arm_x = px - 2.0f;
                        float arm_y = py - 9.0f;
                        // 让手腕顺着从基座到目标点的向量方向延伸
                        float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
                        float f_xx = arm_x / length;
                        float f_xy = arm_y / length;
                        float f_xz = 0.0f;

                        g_arm.setServoAngle(1, 15, 110.0f);
                        usleep(1500000);
                        // 手腕是自然歪斜姿态
                        g_arm.moveSmooth(1, arm_x, arm_y, -2.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        // 依据实测数据：Y=-3 -> 190, Y=0 -> 180, Y=5 -> 173
                        // 这里的 arm_y 就是实际 Y 坐标 (py - 10.0)
                        float ch14_angle = 0.242f * arm_y * arm_y - 2.608f * arm_y + 180.0f;
                        if (ch14_angle > 220.0f)
                            ch14_angle = 220.0f;
                        if (ch14_angle < 90.0f)
                            ch14_angle = 90.0f;
                        float raw_physical_angle = ch14_angle - 55.0f;
                        g_arm.setServoAngle(1, 14, raw_physical_angle);
                        usleep(1500000);
                        std::cout << ">>> 执行完毕 <<<\n"
                                  << std::endl;
                        sendToMonitor("FIX_111\r\n");
                    })
            .detach();
    }

    void executeDemo112(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(1, px, py - 9.0f, 2, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        g_arm.moveSmooth(1, px, py - 0.5f, 2, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.setServoAngle(1, 15, Arm1_close);
                        usleep(1500000);
                        g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                        usleep(1000000);
                        sendToMonitor("DEMO_DONE\r\n");
                    })
            .detach();
    }

    void executeDemo021(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;



                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py, 4.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px, py, 1.0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(800000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1000000);
                        g_arm.moveSmooth(0, px, py + 6.0f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                        sendToMonitor("DEMO_DONE\r\n");
                    })
            .detach();
    }

    void executeDemo031(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py, 3.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px, py, 0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(800000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1000000);
                        g_arm.moveSmooth(0, px, py + 6.0f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                    })
        .detach();
    }
    void executeDemo131(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        // 实际要去的三维坐标点
                        float arm_x = px - 2;
                        float arm_y = py - 11.0f;
                        // 让手腕顺着从基座到目标点的向量方向延伸
                        float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
                        float f_xx = arm_x / length;
                        float f_xy = arm_y / length;
                        float f_xz = 0.0f;
                        g_arm.moveSmooth(1, -12, -10 , 8 , 0, 0, -1, -1, 0, 0); //停在一旁
                        usleep(1000000);
                        std::cout << "\n>>>开始执行demo131 <<<" << std::endl;
                        g_arm.setServoAngle(1, 15, 90.0f);
                        usleep(1500000);
                        // 手腕是自然歪斜姿态
                        g_arm.moveSmooth(1, arm_x, arm_y, -2.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        // 依据实测数据：Y=-3 -> 190, Y=0 -> 180, Y=5 -> 173
                        // 这里的 arm_y 就是实际 Y 坐标 (py - 10.0)
                        float ch14_angle = 0.242f * arm_y * arm_y - 2.608f * arm_y + 180.0f;
                        if (ch14_angle > 220.0f)
                            ch14_angle = 220.0f;
                        if (ch14_angle < 90.0f)
                            ch14_angle = 90.0f;
                        float raw_physical_angle = ch14_angle - 55.0f;
                        g_arm.setServoAngle(1, 14, raw_physical_angle);
                        usleep(1500000);
                        std::cout << ">>> 执行完毕 <<<\n"
                                  << std::endl;
                        sendToMonitor("FIX_131\r\n");
                    })
            .detach();
    }
    void executeDemo132(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = 0.0f, f_zy = 0.3f, f_zz = -0.916f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(1, px - 1, py - 9.0f, 0.2, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        std::cout << "\n>>>开始执行demo132 <<<" << std::endl;
                        g_arm.setServoAngle(1, 15, Arm1_open);
                        usleep(1500000);
                        g_arm.moveSmooth(1, px , py - 2.5f, 1.0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.setServoAngle(1, 15, Arm1_close);
                        usleep(1000000);
                        g_arm.moveSmooth(1, px , py - 4.2f, 2.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(500000);
                        g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                        usleep(1000000);
                        sendToMonitor("DEMO_DONE\r\n");
                    })
            .detach();
    }

    void executeDemo121(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        // 实际要去的三维坐标点
                        float arm_x = px - 2.0f;
                        float arm_y = py - 15.5f;
                        // 让手腕顺着从基座到目标点的向量方向延伸
                        float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
                        float f_xx = arm_x / length;
                        float f_xy = arm_y / length;
                        float f_xz = 0.0f;
                        g_arm.moveSmooth(1, -13, -10, 8, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);
                        g_arm.setServoAngle(1, 15, 90.0f);
                        usleep(1500000);
                        // 手腕是自然歪斜姿态
                        g_arm.moveSmooth(1, arm_x, arm_y, -2.0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        // 依据实测数据：Y=-3 -> 190, Y=0 -> 180, Y=5 -> 173
                        // 这里的 arm_y 就是实际 Y 坐标 (py - 10.0)
                        float ch14_angle = 0.242f * arm_y * arm_y - 2.608f * arm_y + 180.0f;
                        if (ch14_angle > 220.0f)
                            ch14_angle = 220.0f;
                        if (ch14_angle < 90.0f)
                            ch14_angle = 90.0f;
                        float raw_physical_angle = ch14_angle - 55.0f;
                        g_arm.setServoAngle(1, 14, raw_physical_angle);
                        usleep(1500000);
                        std::cout << ">>> 执行完毕 <<<\n"
                                  << std::endl;
                        sendToMonitor("FIX_121\r\n");
                    })
            .detach();
    }
    void executeDemo122(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = 0.0f, f_zy = 0.4f, f_zz = -0.916f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(1, px - 1, py - 10.0f, -1, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        std::cout << "\n>>>开始执行demo132 <<<" << std::endl;
                        g_arm.setServoAngle(1, 15, Arm1_open);
                        usleep(1500000);
                        g_arm.moveSmooth(1, px - 1, py - 3.0f, -1, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.setServoAngle(1, 15, Arm1_close);
                        usleep(1000000);
                        g_arm.moveSmooth(1, px - 1, py - 6.0f, 2, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                        usleep(1000000);
                    })
            .detach();
    }
    void executeDemo041(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        std::cout << "\n>>>开始执行demo041 <<<" << std::endl;
                        g_arm.setServoAngle(0, 6, 30.0f);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py, pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py, pz - 4.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.setServoAngle(0, 6, 200.0f);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py, pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        std::cout << ">>>  执行完毕！ <<<\n"
                                  << std::endl;
                        g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                    })
            .detach();
    }

    // 固定指令
    void executeDo001()
    {
        std::thread([]()
                    {       
            g_arm.moveSmooth(0, -20, -3 , 0, -0.1, 0, -1, -1, 0, 0); usleep(2000000); 
            g_arm.moveSmooth(0, -20, -3 , -5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.moveSmooth(0, -20, -3 , -7, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1000000);
            g_arm.moveSmooth(0, -15, 5 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000); })
            .detach();
    }

    // 换手
    void executeDo031()
    {
        std::thread([]()
        {    
            g_arm.setServoAngle(0, 6, Arm0_open); 
            usleep(1000000);
            g_arm.moveSmooth(1, -3, 0 , 11 , 0 , 0, -1, -1, 0, 0); 
            usleep(1000000);
            g_arm.moveSmooth(1, -3, 6 , 11 , 0 , 0, -1, -1, 0, 0); 
            usleep(1000000);
            g_arm.moveSmooth(1, -5, 14 , 7 , 0 , 0.2 , -0.916 , -1, 0, 0); 
            usleep(1000000);
            g_arm.moveSmooth(0, -8.5, -0.3 , 7.2 , 0 , 0, -1, -1, 0, 0);  //key
            usleep(2000000);
            g_arm.setServoAngle(0, 6, Arm0_close); 
            usleep(1000000);
            g_arm.setServoAngle(1, 15, (Arm1_close + Arm1_open)/2);
            usleep(800000);
            g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
            usleep(1500000);
            g_arm.moveSmooth(1, -3, 6 , 11 , 0 , 0, -1, -1, 0, 0); 
            usleep(1800000);
            g_arm.moveSmooth(1, -3, 0 , 11 , 0 , 0, -1, -1, 0, 0); 
            usleep(1800000);

            if (g_has_cache_091) {
                float px = g_cache_091_px;
                float py = g_cache_091_py;
                float pz = g_cache_091_pz;
                float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
                float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
                float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间

                g_arm.moveSmooth(1, -13, 0 , 11 , -0.1 , 0, -1, -1, 0, 0); 
                usleep(1000000);

                g_arm.moveSmooth(1, px_arm1 + 1.0 , py_arm1 - 3.2f , 9.5f, 0.1f , f_zy, f_zz, f_xx, f_xy, f_xz);usleep(2000000);
                g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                g_arm.moveSmooth(1, px_arm1 + 1.0 , py_arm1 - 1.2f, 6.0f, 0.1f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                g_arm.setServoAngle(1, 15, Arm1_close); 
            
                //开环代码
                //g_arm.moveSmooth(0, px -0.5f, py + 14.5f , 7.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                //g_arm.moveSmooth(0, px -0.5f, py + 10.5f , 6.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                //g_arm.moveSmooth(0, px -0.5f, py + 10.5f , 5.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
                //g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000); // 组装压实后，松开爪子

                //视觉闭环
                g_arm.moveSmooth(0, px -0.5f, py + 14.5f , 7.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                g_arm.moveSmooth(0, px -2.2f, py + 10.0f , 6.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                //停止移动，向 Monitor 请求拍照验核
                std::cout << ">>> [闭环] 右臂就位，请求上位机执行边缘校验..." << std::endl;
                sendToMonitor("CHECK_091\r\n");

                //g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                //g_arm.moveSmooth(1, px_arm1, py_arm1-1 , 9.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);

                //g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                //usleep(1000000);
                //g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁 
                //usleep(1000000);

            } else {
                std::cout << "\n>>> 没有发现 091 <<<" << std::endl;
            } 
        })
            .detach();
    }


    // 拼装动作（底）
    void executeDemo091(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        g_cache_091_px = px;
                        g_cache_091_py = py;
                        g_cache_091_pz = pz;
                        g_has_cache_091 = true;
                        float f_zx = -0.37f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        //g_arm.moveSmooth(0, px - 3.0f, py + 15.0f, 2.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        //usleep(2000000);
                        g_arm.moveSmooth(0, px - 3.0f, py + 17.5f, 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px - 3.0f, py + 16.5f, -0.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.moveSmooth(0, px - 3.0f, py + 16.5f, -1.2f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(500000);
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(800000);
                        g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
                        sendToMonitor("DEMO_DONE\r\n");
                    })
            .detach();
    }

    // 2. 新增 DEMO092: 负责微调，微调完再次请求检查
    void executeDemo092(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO092] 闭环微调执行中..." << std::endl;
            
            g_arm.moveSmooth(0, px -0.5f, py + 14.5f , 7.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px -1.5f, py + 10.0f , 7.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);

            // 执行完微调后，再次呼叫 Monitor
            sendToMonitor("CHECK_091\r\n");
        }).detach();
    }

    // 3. 新增 DEMO093: 闭环检验通过后的装配收尾
    void executeDemo093(float px, float py, float pz)
    {
        std::thread([=]() {
            std::cout << "\n>>> [DEMO093] 边缘检验通过，执行最终压实与复位！" << std::endl;
            float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;
            float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
            float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间

            g_arm.moveSmooth(0, px -1.5f, py + 9.0f , 6.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1000000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1500000); // 组装压实后，松开爪子

            g_arm.setServoAngle(1, 15, Arm1_open); usleep(1000000);
            g_arm.moveSmooth(1, px_arm1 + 0.5 , py_arm1 - 3.2f , 9.5f, 0.1f , f_zy, f_zz, f_xx, f_xy, f_xz);usleep(2000000);
            g_arm.moveSmooth(1, -13, 0 , 11 , -0.1 , 0, -1, -1, 0, 0); 
            usleep(1000000);
            g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁 
            usleep(1000000);
            g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
            usleep(1000000);
            sendToMonitor("DEMO_DONE\r\n");

        }).detach();
    }

    void executeDemo001(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
                        float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间
                        float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;


                        g_arm.moveSmooth(1, -13, 0 , 11 , -0.1 , 0, -1, -1, 0, 0); 
                        usleep(1000000);

                        g_arm.moveSmooth(1, px_arm1 + 1.0 , py_arm1 - 7.2f , 9.5f, 0.1f , f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                        g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                        g_arm.moveSmooth(1, px_arm1 + 1.0 , py_arm1 - 5.2f, 6.0f, 0.1f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                        g_arm.setServoAngle(1, 15, Arm1_close); 

                        // 拼ID=2物体
                        //视觉闭环
                        g_arm.moveSmooth(0, px + 1.5f, py + 8.5f , 8.5f, -0.05, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                        g_arm.moveSmooth(0, px + 1.5f, py + 6.0f , 8.5f, -0.05, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                        //停止移动，向 Monitor 请求拍照验核
                        std::cout << ">>> [闭环] 右臂就位，请求上位机执行边缘校验..." << std::endl;
                        sendToMonitor("CHECK_001\r\n");
                    })
            .detach();
    }

    void executeDemo001ADJ(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.05f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO001] 闭环微调执行中..." << std::endl;
            
            g_arm.moveSmooth(0, px + 0.5f, py + 8.5f , 8.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px + 0.5f, py + 6.0f , 8.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);

            // 执行完微调后，再次呼叫 Monitor
            sendToMonitor("CHECK_001\r\n");
        }).detach();
    }

    void executeDemo001DONE(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.05f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;
            float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
            float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间

            g_arm.moveSmooth(0, px + 0.5f, py + 5.0f , 5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1500000); // 组装压实后，松开爪子
            g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
            usleep(1000000);
            g_arm.setServoAngle(1, 15, Arm1_open); usleep(1000000);
            g_arm.moveSmooth(1, px_arm1 + 1.5 , py_arm1 - 3.2f , 9.5f, 0.0f , f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
            g_arm.moveSmooth(1, -13, 0 , 11 , -0.1 , 0, -1, -1, 0, 0); 
            usleep(1000000);
            g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁 
            usleep(1000000);
            sendToMonitor("DEMO_DONE\r\n");
        }).detach();
    }


    void executeDemo002(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
                        float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间
                        float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        // 拼ID=1物体
                        g_arm.moveSmooth(1, px_arm1 + 3.0, py_arm1 - 3.0 , 9.5f, -0.0f, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(1, px_arm1 + 0.5, py_arm1 - 3.0 , 9.0f, -0.0f, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);

                        //停止移动，向 Monitor 请求拍照验核
                        std::cout << ">>> [闭环] 左臂就位，请求上位机执行边缘校验..." << std::endl;
                        sendToMonitor("CHECK_002\r\n");
                    })
            .detach();
    }

    void executeDemo002ADJ(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.0f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;
            float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
            float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间

            std::cout << "\n>>> [DEMO002] 闭环微调执行中..." << std::endl;
            
            g_arm.moveSmooth(1, px_arm1 + 3.0, py_arm1 - 3.0, 9.5f, -0.0f, f_zy, f_zz, f_xx, f_xy, f_xz);
            usleep(2000000);
            g_arm.moveSmooth(1, px_arm1 + 0.5, py_arm1 - 3.0, 9.0f, -0.0f, f_zy, f_zz, f_xx, f_xy, f_xz);
            usleep(2000000);

            // 执行完微调后，再次呼叫 Monitor
            sendToMonitor("CHECK_002\r\n");
        }).detach();
    }

    void executeDemo002DONE(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.08f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;
            float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
            float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间
            
            g_arm.moveSmooth(1, px_arm1 , py_arm1 - 3.0, 7.0f, -0.08f, f_zy, f_zz, f_xx, f_xy, f_xz);
            usleep(1500000);
            g_arm.setServoAngle(1, 15, (Arm1_open + Arm1_close) / 2);
            usleep(500000); // 组装压实后，松开爪子
            g_arm.moveSmooth(1, px_arm1 , py_arm1 - 3.0, 9.0f, -0.08f, f_zy, f_zz, f_xx, f_xy, f_xz);
            usleep(1000000);
            g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁 
            usleep(1000000);
            g_arm.moveSmooth(1, px_arm1 , py_arm1 - 4.5, 6.0f, -0.08f, f_zy, f_zz, f_xx, f_xy, f_xz);
            usleep(1000000);
            // 再拼一个ID=2的物体
            // 视觉闭环
            g_arm.moveSmooth(0, px + 0.5f, py + 8.5f , 7.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px + 0.5f, py + 5.7f , 7.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            //停止移动，向 Monitor 请求拍照验核
            std::cout << ">>> [闭环] 右臂就位，请求上位机执行边缘校验..." << std::endl;
            sendToMonitor("CHECK_003\r\n");
   
        }).detach();
    }

        void executeDemo003ADJ(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.08f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO001] 闭环微调执行中..." << std::endl;
            
            g_arm.moveSmooth(0, px + 0.5f, py + 8.5f , 8.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(0, px + 0.5f, py + 5.7f , 8.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);

            // 执行完微调后，再次呼叫 Monitor
            sendToMonitor("CHECK_003\r\n");
        }).detach();
    }

    void executeDemo003DONE(float px, float py, float pz)
    {
        std::thread([=]() {
            float f_zx = -0.08f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;
            float px_arm1 = px + 1.2f;  // X 轴标定误差补偿
            float py_arm1 = py + 20.0f;  // Y 轴平移厘米，对齐物理空间

            g_arm.moveSmooth(0, px + 0.5f, py + 5.0f , 7.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);

            g_arm.setServoAngle(0, 6, Arm0_open); usleep(1500000); // 组装压实后，松开爪子
            g_arm.moveSmooth(0, -8, 10 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁
            usleep(1000000);
            g_arm.moveSmooth(1, -9, -7.5 , 8 , -0.1 , 0, -1, -1, 0, 0); //停在一旁 
            usleep(1000000);
            sendToMonitor("DEMO_DONE\r\n");

        }).detach();
    }

    // arm2是摄像头云台
    void executeDemo220()
    {
        std::thread([]()
                    {
                        std::cout << "\n>>> [巡航搜索] 调整云台视角 1 (DEMO220: Pan 43, Tilt 45) <<<" << std::endl;
                        g_arm.moveCameraSmooth(43.0f, 45.0f);
                        usleep(2000000);                   // 预留 1 秒钟给云台平滑移动和摄像头画面稳定
                        sendToMonitor("FIND_ACK_220\r\n"); // 动作完成，向上位机汇报！
                    })
            .detach();
    }

    void executeDemo221()
    {
        std::thread([]()
                    {
                        std::cout << "\n>>> [巡航搜索] 调整云台视角 2 (DEMO221: Pan 43, Tilt 30) <<<" << std::endl;
                        g_arm.moveCameraSmooth(43.0f, 20.0f);
                        usleep(2000000);                   // 预留 1 秒钟
                        sendToMonitor("FIND_ACK_221\r\n"); // 动作完成，向上位机汇报
                    })
            .detach();
    }
}