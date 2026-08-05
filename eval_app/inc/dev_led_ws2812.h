#ifndef __DEV_LED_WS2812_H__
#define __DEV_LED_WS2812_H__

int ws2812_init(void);
void ws2812_set_color(int index, unsigned char r, unsigned char g, unsigned char b);
void ws2812_update(void);

#endif