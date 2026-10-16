#ifndef __APP_ENCODER_H
#define __APP_ENCODER_H

#include "stm32f1xx_hal.h"

/**
  * @brief  启动编码器模式
  * @note   必须在 MX_TIM1_Init() 和 MX_TIM2_Init() 之后调用
  */
void App_Encoder_Init(void);

/**
  * @brief  读取编码器计数值
  * @param  p_enc1: 左轮计数值指针
  * @param  p_enc2: 右轮计数值指针
  * @note   直接读 CNT 寄存器，16 位无符号值
  */
void App_Encoder_Update(uint16_t *p_enc1, uint16_t *p_enc2);

/**
  * @brief  获取左轮累计位置（单位：脉冲数）
  * @note   内部用 int16_t 差值自动处理 16 位计数器溢出
  */
float App_Encoder_GetPos_L(void);

/**
  * @brief  获取右轮累计位置（单位：脉冲数）
  * @note   内部用 int16_t 差值自动处理 16 位计数器溢出
  */
float App_Encoder_GetPos_R(void);

#endif