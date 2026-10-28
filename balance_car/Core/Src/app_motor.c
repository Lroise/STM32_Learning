#include "app_motor.h"
#include "app_encoder.h"
#include "app_pwm.h"
#include "pid.h"

static uint8_t motor_enable = 0;

void App_Motor_Init(void) { }

void App_Motor_Cmd(uint8_t On)
{
    motor_enable = On; 
    if(On) TB6612_Enable(1); 
    else TB6612_Enable(0);
}

void App_Motor_Proc(void)
{
    // 什么都不做，完全交给上层控制
}

// 修改底层函数，让它直接接收最终的 PWM 值
void App_Motor_SetOmega_L(float Omega) 
{ 
    if(motor_enable) Motor_SetSpeed((int16_t)Omega, (int16_t)Omega); 
}
void App_Motor_SetOmega_R(float Omega) { }