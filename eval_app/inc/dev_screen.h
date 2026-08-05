#ifndef __DEV_SCREEN_H__
#define __DEV_SCREEN_H__

int screen_init(void);
void screen_show_string(int row, const char *str);
void screen_clear(void);

#endif