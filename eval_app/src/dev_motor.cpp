#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "dev_motor.h"

// 电机方向控制 GPIO 组（AIN1, AIN2, BIN1, BIN2）
static int dir_gpios[4] = {20, 21, 22, 23}; 

static void sysfs_write(const char *path, const char *val) {
    int fd = open(path, O_WRONLY);
    if (fd >= 0) { write(fd, val, sizeof(val)); close(fd); }
}

int motor_init(void) {
    // 初始化 PWM0 和 PWM1
    system("echo 0 > /sys/class/pwm/pwmchip0/export 2>/dev/null");
    system("echo 0 > /sys/class/pwm/pwmchip1/export 2>/dev/null");
    system("echo 1000000 > /sys/class/pwm/pwmchip0/pwm0/period");
    system("echo 1000000 > /sys/class/pwm/pwmchip1/pwm0/period");
    system("echo 0 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
    system("echo 0 > /sys/class/pwm/pwmchip1/pwm0/duty_cycle");
    system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");
    system("echo 1 > /sys/class/pwm/pwmchip1/pwm0/enable");
    return 0;
}

void motor_set_speed(int motor_id, int speed) {
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    long duty = (labs(speed) * 1000000L) / 100;
    char path[128], val[32];
    
    snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip%d/pwm0/duty_cycle", motor_id - 1);
    snprintf(val, sizeof(val), "%ld", duty);
    sysfs_write(path, val);
    
    // 设置方向 (略，在此调用 GPIO 写入)
}

void motor_deinit(void) {
    motor_set_speed(1, 0);
    motor_set_speed(2, 0);
}