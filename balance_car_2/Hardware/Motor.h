#ifndef __MOTOR_
#define __MOTOR_

#include "main.h"
/****************************************************************************
*定义外部接口函数
****************************************************************************/
void PWM_Init(void);
void Motor_Init(void);
uint16_t Num_Abs(int16_t P);
void Limit(int16_t *motoA,int16_t *motoB);
void Load(int16_t moto1,int16_t moto2);

#endif
