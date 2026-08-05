#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "dev_key.h"

/* 假设 3 个按键分别映射为 GPIO 引脚（视具体设备树/Sysfs映射修改） */
static int key_gpios[3] = {10, 11, 12}; 

int key_init(void) {
    char path[64];
    for (int i = 0; i < 3; i++) {
        // 导出 GPIO
        int fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd >= 0) {
            dprintf(fd, "%d", key_gpios[i]);
            close(fd);
        }
        // 设置为输入方向
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", key_gpios[i]);
        fd = open(path, O_WRONLY);
        if (fd >= 0) {
            write(fd, "in", 2);
            close(fd);
        }
    }
    return 0;
}

int key_read_state(int key_num) {
    if (key_num < 1 || key_num > 3) return 0;
    char path[64], val;
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", key_gpios[key_num - 1]);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;
    read(fd, &val, 1);
    close(fd);
    return (val == '0') ? 1 : 0; // 低电平触发按下
}

void key_deinit(void) {}