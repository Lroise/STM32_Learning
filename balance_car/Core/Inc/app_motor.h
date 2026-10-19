#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "stm32f1xx_hal.h"

void App_Motor_Init(void);
void App_Motor_Proc(void);
void App_Motor_SetOmega_L(float Omega);
void App_Motor_SetOmega_R(float Omega);

// ================= 新增：电机开关控制 =================
/**
  * @brief  开关电机
  * @param  On: 控制电机的开关，0 - 关闭，非0 - 开启
  * @note   在开启或关闭电机时，会对 PID 控制器进行复位，防止积分饱和起飞
  */
void App_Motor_Cmd(uint8_t On);

#endif