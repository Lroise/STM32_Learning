#ifndef __CONTROL_
#define __CONTROL_

#include "main.h"

// 声明控制循环函数，让 main.c 能找到它
void Control_Loop(void);

// 声明三个 PID 函数
int16_t Vertical(float Med,float Angle,float Gyro_X);
int16_t Velocity(int16_t Target_Speed,int16_t Encoder_left,int16_t Encoder_right);
int16_t Turn(int16_t RC,int16_t gyro_Z);

#endif