#ifndef __DEV_KEY_H__
#define __DEV_KEY_H__

int key_init(void);
int key_read_state(int key_num); // 返回 1 表示按下，0 表示松开
void key_deinit(void);

#endif