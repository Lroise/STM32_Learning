#ifndef __APP_PWM_H
#define __APP_PWM_H

#include "main.h"   //引入HAL库基础头文件，uint8_t、int16_t、GPIO_PIN_SET这些定义都在这里

void App_PWM_Init(void);
void TB6612_Enable(uint8_t en);
void Motor_SetSpeed(int16_t left, int16_t right);

#endif
