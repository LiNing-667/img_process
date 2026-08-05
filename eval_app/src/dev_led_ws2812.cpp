#include <stdio.h>
#include "dev_led_ws2812.h"

typedef struct { unsigned char r, g, b; } pixel_t;
static pixel_t pixels[4];

int ws2812_init(void) { return 0; }

void ws2812_set_color(int index, unsigned char r, unsigned char g, unsigned char b) {
    if (index >= 0 && index < 4) {
        pixels[index].r = r;
        pixels[index].g = g;
        pixels[index].b = b;
    }
}

void ws2812_update(void) {
    // 基于 GPIO 翻转或 SPI 模拟产生 800kHz 归零码输出
}