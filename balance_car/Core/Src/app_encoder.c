#include "app_encoder.h"
#include "tim.h"

// 编码器一圈总脉冲数 = 线数 × 减速比 × 4（四倍频）
#define ENCODER_PPR   11
#define GEAR_RATIO    30
#define PULSE_PER_REV (ENCODER_PPR * GEAR_RATIO * 4)

// 上一次的 CNT 值，用于算增量
static int16_t last_cnt_l = 0;
static int16_t last_cnt_r = 0;

// 累计位置（单位：脉冲数），用 float 防止溢出
static float pos_l = 0.0f;
static float pos_r = 0.0f;

/**
  * @brief  启动编码器模式
  */
void App_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // 记录初始 CNT，避免第一次调用 GetPos 时算出巨大增量
    last_cnt_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    last_cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    pos_l = 0.0f;
    pos_r = 0.0f;
}

/**
  * @brief  读取编码器计数值
  */
void App_Encoder_Update(uint16_t *p_enc1, uint16_t *p_enc2)
{
    *p_enc1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    *p_enc2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

/**
  * @brief  获取左轮累计位置（单位：脉冲数）
  * @note   取反修正方向：正转位置增加，反转位置减小
  */
float App_Encoder_GetPos_L(void)
{
    int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t diff = (int16_t)(last_cnt_l - now);   // 取反，修正方向
    last_cnt_l = now;
    pos_l += (float)diff;
    return pos_l;
}

/**
  * @brief  获取右轮累计位置（单位：脉冲数）
  * @note   取反修正方向：正转位置增加，反转位置减小
  */
float App_Encoder_GetPos_R(void)
{
    int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = (int16_t)(last_cnt_r - now);   // 取反，修正方向
    last_cnt_r = now;
    pos_r += (float)diff;
    return pos_r;
}