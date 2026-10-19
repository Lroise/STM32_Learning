#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

typedef struct
{
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float SP;           // 用户的设定值（目标速度）
    
    uint64_t t_k_1;     // 上次运行PID的时间（微秒）
    float err_k_1;      // 上次运行PID时的误差
    float err_int_k_1;  // 上次运行的积分值

    // ================= 新增：限幅变量 =================
    float UpperLimit;   // 积分及输出上限
    float LowerLimit;   // 积分及输出下限
} PID_TypeDef;

void  PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void  PID_ChangeSP(PID_TypeDef *PID, float SP);
float PID_Compute(PID_TypeDef *PID, float FB);

// ================= 新增：函数声明 =================
void  PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower);
void  PID_Reset(PID_TypeDef *PID);

#endif