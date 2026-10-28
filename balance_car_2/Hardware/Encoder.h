#ifndef __ENCODER_
#define __ENCODER_

#include "main.h"
/****************************************************************************
*定义外部接口函数
****************************************************************************/
void Encoder_TIM4_Init(void);
void Encoder_TIM3_Init(void);
int16_t Read_Speed(uint8_t TIMX);

#endif
