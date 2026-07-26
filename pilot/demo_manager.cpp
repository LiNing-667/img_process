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
        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
        usleep(1000000);
        g_arm.setServoAngle(1, 15, Arm1_open);
        usleep(1000000);
        g_arm.moveSmooth(1, -13, -10, 8, -0.1, 0, -1, -1, 0, 0);
        usleep(1000000);
        std::cout << ">>> 测试序列执行完毕！ <<<\n"
                  << std::endl;
    }

    void executeDemo000(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);

                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(1500000);
                        // g_arm.moveSmooth(0, px-2 , py + 5.2f, -4.5 , f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                        g_arm.moveSmooth(0, px - 0.2, py + 6.0f, -1.0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px - 0.2, py + 6.0f, -7.4, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1000000);
                        // g_arm.moveSmooth(0, px-2 , py + 5.2f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(1500000);
                        std::cout << ">>>  执行完毕！ <<<\n"
                                  << std::endl;
                        g_arm.moveSmooth(0, -13, 12, 5, -0.1, 0, -1, -1, 0, 0);
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
                        float arm_y = py - 10.0f;
                        // 让手腕顺着从基座到目标点的向量方向延伸
                        float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
                        float f_xx = arm_x / length;
                        float f_xy = arm_y / length;
                        float f_xz = 0.0f;
                        g_arm.moveSmooth(1, -13, -10, 8, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);
                        std::cout << "\n>>>开始执行demo111 <<<" << std::endl;
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
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(1, px, py - 9.0f, 1, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        std::cout << "\n>>>开始执行demo111 <<<" << std::endl;
                        g_arm.setServoAngle(1, 15, 130.0f);
                        usleep(1500000);
                        g_arm.moveSmooth(1, px - 1.6f, py - 0.0f, 1, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.setServoAngle(1, 15, 50.0f);
                        usleep(1000000);
                        std::cout << ">>> 执行完毕 <<<\n"
                                  << std::endl;
                        g_arm.moveSmooth(1, -13, -10, 8, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);
                    })
            .detach();
    }

    void executeDemo021(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        std::cout << "\n>>>开始执行demo000 <<<" << std::endl;
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py, 1.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px, py, -2.2, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py + 6.0f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        std::cout << ">>>  执行完毕！ <<<\n"
                                  << std::endl;
                        g_arm.moveSmooth(0, -13, 12, 5, -0.1, 0, -1, -1, 0, 0);
                    })
            .detach();
    }

    void executeDemo031(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        std::cout << "\n>>>开始执行demo000 <<<" << std::endl;
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py + 1.5, 1.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px, py + 1.5, -1.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1500000);
                        g_arm.moveSmooth(0, px, py + 6.0f, pz + 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1500000);
                        std::cout << ">>>  执行完毕！ <<<\n"
                                  << std::endl;
                        g_arm.moveSmooth(0, -13, 12, 5, -0.1, 0, -1, -1, 0, 0);
                    })
            .detach();
    }
    void executeDemo131(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
                        // 实际要去的三维坐标点
                        float arm_x = px - 2.0f;
                        float arm_y = py - 12.0f;
                        // 让手腕顺着从基座到目标点的向量方向延伸
                        float length = std::sqrt(arm_x * arm_x + arm_y * arm_y);
                        float f_xx = arm_x / length;
                        float f_xy = arm_y / length;
                        float f_xz = 0.0f;
                        g_arm.moveSmooth(1, -13, -10, 8, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);
                        std::cout << "\n>>>开始执行demo131 <<<" << std::endl;
                        g_arm.setServoAngle(1, 15, 90.0f);
                        usleep(1500000);
                        // 手腕是自然歪斜姿态
                        g_arm.moveSmooth(1, arm_x, arm_y, -3.0, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
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
                        g_arm.moveSmooth(1, px - 1, py - 3.1f, 0.2, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.setServoAngle(1, 15, Arm1_close);
                        usleep(1000000);
                        g_arm.moveSmooth(1, px - 1, py - 4.2f, 1.5, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.moveSmooth(1, -10, -8, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);
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
                        g_arm.moveSmooth(1, -10, -8, 5, -0.1, 0, -1, -1, 0, 0);
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
                        g_arm.moveSmooth(0, -15, 5, 5, -0.1, 0, -1, -1, 0, 0);
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
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.moveSmooth(1, -10, 0 ,8, -0.1, 0, -1, -1, 0, 0);usleep(1200000);
            // 顺序为: CH9=150, CH10=70, CH11=1, CH12=60, CH13=130, CH14=70
            std::vector<float> target_ch = {
                145.0f + 18.0f,  // CH9  
                80.0f + 18.0f,   // CH10 
                1.0f + 18.0f,    // CH11 
                70.0f + 18.0f,   // CH12 
                120.0f + 18.0f,  // CH13 
                70.0f - 55.0f    // CH14
            };
            // 调用同步平滑插值，设定 2.5 秒内缓慢移动到位
            g_arm.moveRawChannelsSmooth(1, target_ch, 1.5f); 
            usleep(500000); // 等待移动完成
            g_arm.moveSmooth(0, -12.1, -7.8 , 1.0, -0.1, 0, -1, -1, 0, 0); usleep(2300000); ////
            g_arm.setServoAngle(0, 6, Arm0_close); usleep(1000000); 
            g_arm.setServoAngle(1, 15, (Arm1_open + Arm1_close)/2); usleep(1000000);
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
            g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);
            g_arm.moveSmooth(1, -10, 0 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(2000000); 
            g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(1200000); 
            g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); 

            if (g_has_cache_091) {
                float px = g_cache_091_px;
                float py = g_cache_091_py;
                float pz = g_cache_091_pz;
                float f_zx = -0.20f, f_zy = 0.0f, f_zz = -1.0f;
                float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                float px_arm1 = px + 0.02f;  // X 轴极微小的标定误差补偿
                float py_arm1 = py + 18.2f;  // Y 轴平移 18.2 厘米，瞬间对齐物理空间！

                g_arm.moveSmooth(1, px_arm1 + 0.5 , py_arm1 - 3.2f , 8.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                g_arm.moveSmooth(1, px_arm1 + 0.5 , py_arm1 - 1.2f, 5.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);
                g_arm.setServoAngle(1, 15, Arm1_close); 
                
                g_arm.moveSmooth(0, px -0.3f, py + 14.5f , 3.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                g_arm.moveSmooth(0, px -0.3f, py + 10.5f , 2.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                g_arm.moveSmooth(0, px -0.3f, py + 10.5f , 1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(500000);
                g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000); // 组装压实后，松开爪子

                g_arm.setServoAngle(1, 15, Arm1_open); usleep(600000);
                g_arm.moveSmooth(1, px_arm1, py_arm1-1 , 9.0f, -0.20f, f_zy, f_zz, f_xx, f_xy, f_xz);usleep(1000000);

                g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);// 抬升并回归待命姿态
                g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(1000000);

            } else {
                std::cout << "\n>>> 没有发现 demo091 <<<" << std::endl;
            } })
            .detach();
    }

    // 临时调试用
    void executeDo002()
    {
        std::thread([]()
                    {
                        float f_zx = -0.37f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        std::cout << "\n>>> [DEMO091] ARM0 移动至组装目标 <<<" << std::endl;
                        g_arm.moveSmooth(0, -20, 4, 5, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, -22, -8, 5, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.setServoAngle(0, 6, (Arm0_close + Arm0_open) / 2);
                        usleep(800000);
                        g_arm.moveSmooth(0, -20, 4, 5, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1200000);
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(800000);

                        // 换手
                        g_arm.moveSmooth(1, -10, 0, 8, -0.1, 0, -1, -1, 0, 0);
                        usleep(1200000);
                        std::vector<float> target_ch = {145.0f + 18.0f, 80.0f + 18.0f, 1.0f + 18.0f, 70.0f + 18.0f, 120.0f + 18.0f, 70.0f - 55.0f};
                        g_arm.moveRawChannelsSmooth(1, target_ch, 1.5f);
                        usleep(1500000); // 等待移动完成
                        g_arm.moveSmooth(0, -11.8, -6, 2.2, -0.1, 0, -1, -1, 0, 0);
                        usleep(2300000);
                        g_arm.setServoAngle(0, 6, Arm0_close);
                        usleep(1000000);
                        g_arm.setServoAngle(1, 15, 50);
                        usleep(1000000);
                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);
                        g_arm.setServoAngle(1, 15, Arm1_close);
                        usleep(1000000);
                        g_arm.moveSmooth(1, -10, 0, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(2000000);
                        g_arm.moveSmooth(1, -13, -10, 9, -0.1, 0, -1, -1, 0, 0);
                        usleep(200000);
                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1200000);

                        // 拼ID=2物体
                        g_arm.moveSmooth(0, -17, 5, 6, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, -17, 3, 6, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, -17, 3, 2, -0.2, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(800000); // 组装压实后，松开爪子

                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000); // 抬升并回归待命姿态
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

                        std::cout << "\n>>> [DEMO091] ARM0 移动至组装目标 <<<" << std::endl;
                        g_arm.moveSmooth(0, px - 2.8f, py + 19.5f, -1.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px - 2.8f, py + 18.5f, -3.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(500000);
                        g_arm.moveSmooth(0, px - 2.8f, py + 18.5f, -5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(500000);
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(800000);
                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                    })
            .detach();
    }
    void executeDemo001(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float px_arm1 = px + 0.02f;
                        float py_arm1 = py + 18.2f;
                        float f_zx = -0.20f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        //   std::cout << "\n>>> [DEMO091] ARM0 移动至组装目标 <<<" << std::endl;
                        //   g_arm.moveSmooth(0, px -5.0f, py+0.5 , 4.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
                        //   g_arm.moveSmooth(0, px -5.0f, py+0.5 , 2.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(800000);
                        //   g_arm.setServoAngle(0, 6, (Arm0_close + Arm0_open)/2 ); usleep(800000);
                        //   g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1200000);
                        //   g_arm.setServoAngle(0, 6, Arm0_open); usleep(800000);
                        //
                        //   //换手
                        //   g_arm.moveSmooth(1, -10, 0 ,8, -0.1, 0, -1, -1, 0, 0);usleep(1200000);
                        //   std::vector<float> target_ch = {145.0f + 18.0f, 80.0f + 18.0f, 1.0f + 18.0f, 70.0f + 18.0f,  120.0f + 18.0f,  70.0f - 55.0f };
                        //   g_arm.moveRawChannelsSmooth(1, target_ch, 1.5f); usleep(1500000); // 等待移动完成
                        //   g_arm.moveSmooth(0, -12.5, -6 , 2.2, -0.1, 0, -1, -1, 0, 0); usleep(2300000);
                        //   g_arm.setServoAngle(0, 6, Arm0_close); usleep(1000000);
                        //   g_arm.setServoAngle(1, 15, Arm1_open); usleep(1000000);
                        //   g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0); usleep(1000000);
                        //   g_arm.setServoAngle(1, 15, Arm1_close); usleep(1000000);
                        //   g_arm.moveSmooth(1, -10, 0 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(2000000);
                        //   g_arm.moveSmooth(1, -13, -10 , 9, -0.1, 0, -1, -1, 0, 0);  usleep(200000);
                        //   g_arm.moveSmooth(0, -13 , 10 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(1200000);

                        // 拼ID=2物体
                        g_arm.moveSmooth(0, px + 1.0f, py + 8.5f, 6.2f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px + 1.0f, py + 5.8f, 5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px + 1.0f, py + 5.8f, 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(800000);
                        g_arm.moveSmooth(0, px + 1.0f, py + 5.8f, 0.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(500000);
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(800000); // 组装压实后，松开爪子

                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000); // 抬升并回归待命姿态
                    })
            .detach();
    }

    void executeDemo102(float px, float py, float pz)
    {
        std::thread([=]()
                    {
            float f_zx = -0.1f, f_zy = 0.0f, f_zz = -1.0f;
            float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

            std::cout << "\n>>> [DEMO102] 第二次拼装: ARM1 移动至目标 <<<" << std::endl;
            g_arm.moveSmooth(1, -13, -10 , 5, -0.1, 0, -1, -1, 0, 0);  usleep(1000000); 
            g_arm.moveSmooth(1, px +2.2f, py + 0.8f , pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2500000);
            g_arm.moveSmooth(1, px -0.5f, py + 0.8f , pz + 3.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);
            g_arm.moveSmooth(1, px -0.5f, py + 0.8f , pz - 0.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz); usleep(2000000);

            g_arm.setServoAngle(1, 15, 150.0f); usleep(1000000); 
            g_arm.moveSmooth(1, -13, -10 , 8, -0.1, 0, -1, -1, 0, 0);  usleep(1000000); })
            .detach();
    }

    void executeDemo002(float px, float py, float pz)
    {
        std::thread([=]()
                    {
                        float px_arm1 = px + 0.02f;
                        float py_arm1 = py + 18.2f;
                        float f_zx = -0.22f, f_zy = 0.0f, f_zz = -1.0f;
                        float f_xx = -1.0f, f_xy = 0.0f, f_xz = 0.0f;

                        // 拼ID=1物体
                        g_arm.moveSmooth(1, px_arm1 + 1.6, py_arm1 - 0.2, 8.5f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(3000000);
                        g_arm.moveSmooth(1, px_arm1 + 0.1, py_arm1 - 0.2, 7.2f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(1, px_arm1 + 0.1, py_arm1 - 0.2, 0.5f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.setServoAngle(1, 15, (Arm1_open + Arm1_close) / 2);
                        usleep(800000); // 组装压实后，松开爪子
                        
                        g_arm.moveSmooth(1, px_arm1 + 2.0, py_arm1 + 0.0, 6.5f, -0.10f, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(1000000);
                        g_arm.moveSmooth(1, -13, -10, 8, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000);

                        // 拼ID=2物体
                        g_arm.moveSmooth(0, px - 0.5f, py + 7.5f, 5.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(3000000);
                        g_arm.moveSmooth(0, px - 0.5f, py + 6.5f, 4.5f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(2000000);
                        g_arm.moveSmooth(0, px - 0.5f, py + 6.5f, 2.0f, f_zx, f_zy, f_zz, f_xx, f_xy, f_xz);
                        usleep(500000);
                        g_arm.setServoAngle(0, 6, Arm0_open);
                        usleep(800000); // 组装压实后，松开爪子

                        g_arm.moveSmooth(0, -13, 10, 5, -0.1, 0, -1, -1, 0, 0);
                        usleep(1000000); // 抬升并回归待命姿态
                    })
            .detach();
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