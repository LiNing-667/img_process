#ifndef __DEV_MOTOR_H__
#define __DEV_MOTOR_H__

int motor_init(void);
/* motor_id: 1 或 2; speed: -100 到 100 */
void motor_set_speed(int motor_id, int speed); 
void motor_deinit(void);

#endif