#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "dev_key.h"
#include "dev_motor.h"
#include "dev_mpu6050.h"
#include "dev_rc522.h"
#include "dev_led_ws2812.h"
#include "dev_screen.h"

static int system_mode = 0;
static int running = 1;

// 传感器数据轮询线程
void *sensor_thread_entry(void *arg) {
    mpu6050_data_t mpu_data;
    unsigned char card_id[4];

    while (running) {
        if (system_mode == 1) { // 模式 1：显示 MPU6050 数据并控制电机
            if (mpu6050_read(&mpu_data) == 0) {
                char disp_str[64];
                snprintf(disp_str, sizeof(disp_str), "Ax:%d Gy:%d", mpu_data.accel_x, mpu_data.gyro_x);
                screen_show_string(2, disp_str);
                
                // 倾斜联动电机
                int speed = mpu_data.accel_x / 300;
                motor_set_speed(1, speed);
                motor_set_speed(2, -speed);
            }
        } else if (system_mode == 2) { // 模式 2：射频卡识别与灯效
            if (rc522_read_card_id(card_id) == 0) {
                char disp_str[64];
                snprintf(disp_str, sizeof(disp_str), "ID:%02X%02X%02X%02X", card_id[0], card_id[1], card_id[2], card_id[3]);
                screen_show_string(2, disp_str);
                
                ws2812_set_color(0, 0, 255, 0); // 读到卡全亮绿灯
                ws2812_update();
            }
        }
        usleep(100000); // 100ms
    }
    return NULL;
}

int main(void) {
    printf("=== 龙芯2K0300能力测评综合测试程序 ===\n");

    // 1. 初始化所有模块
    key_init();
    motor_init();
    mpu6050_init("/dev/i2c-1");
    rc522_init("/dev/spidev1.0");
    ws2812_init();
    screen_init();

    screen_show_string(1, "Mode: Ready");

    // 2. 创建底层线程
    pthread_t thread_id;
    pthread_create(&thread_id, NULL, sensor_thread_entry, NULL);

    // 3. 主循环响应 Key 切换
    while (running) {
        if (key_read_state(1)) {
            system_mode = 1;
            screen_show_string(1, "Mode 1: MPU&Motor");
            ws2812_set_color(0, 255, 0, 0); ws2812_update(); // 红灯
            usleep(300000);
        } else if (key_read_state(2)) {
            system_mode = 2;
            screen_show_string(1, "Mode 2: RC522 Card");
            ws2812_set_color(0, 0, 0, 255); ws2812_update(); // 蓝灯
            usleep(300000);
        } else if (key_read_state(3)) {
            system_mode = 0;
            motor_set_speed(1, 0); motor_set_speed(2, 0);
            screen_show_string(1, "Mode 0: Stop");
            usleep(300000);
        }
        usleep(50000);
    }

    // 清理退出
    motor_deinit();
    key_deinit();
    return 0;
}