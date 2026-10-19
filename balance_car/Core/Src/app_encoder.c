#include "app_encoder.h"
#include "tim.h"
#include "math.h"    // 新增：用于计算 PI

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

// 新增：用于算速度的变量（左右轮分开，防止互相干扰）
static float last_pos_l_speed = 0.0f;
static float last_pos_r_speed = 0.0f;
static uint32_t last_time_l_speed = 0;
static uint32_t last_time_r_speed = 0;

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
    
    // 初始化速度计算的初始值
    last_pos_l_speed = 0.0f;
    last_pos_r_speed = 0.0f;
    last_time_l_speed = HAL_GetTick();
    last_time_r_speed = HAL_GetTick();
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

/**
  * @brief  获取左轮转速（单位：弧度/秒 rad/s）
  * @note   通过两次调用之间的位置差和时间差计算
  */
float App_Encoder_GetSpeed_L(void)
{
    // 1. 读当前位置
    float pos = App_Encoder_GetPos_L();
    
    // 2. 算位移增量
    float diff = pos - last_pos_l_speed;
    last_pos_l_speed = pos;
    
    // 3. 算时间差（秒）
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time_l_speed) * 0.001f;
    last_time_l_speed = now;
    
    if (dt <= 0.0f) dt = 0.001f; // 防止除 0
    
    // 4. 速度换算：脉冲/秒 -> 圈/秒 -> 弧度/秒
    float rev_per_sec = (diff / dt) / (float)PULSE_PER_REV;
    float speed = rev_per_sec * 2.0f * 3.1415926f;

    // ================= 新增：限幅 =================
    // 防止因为 1ms 定时误差导致速度瞬间算出天文数字
    if (speed > 50.0f)  speed = 50.0f;
    if (speed < -50.0f) speed = -50.0f;

    return speed;
}

/**
  * @brief  获取右轮转速（单位：弧度/秒 rad/s）
  * @note   通过两次调用之间的位置差和时间差计算
  */
float App_Encoder_GetSpeed_R(void)
{
    float pos = App_Encoder_GetPos_R();
    float diff = pos - last_pos_r_speed;
    last_pos_r_speed = pos;
    
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time_r_speed) * 0.001f;
    last_time_r_speed = now;
    
    if (dt <= 0.0f) dt = 0.001f;
    
    float rev_per_sec = (diff / dt) / (float)PULSE_PER_REV;
    float speed = rev_per_sec * 2.0f * 3.1415926f;

    // ================= 新增：限幅 =================
    if (speed > 50.0f)  speed = 50.0f;
    if (speed < -50.0f) speed = -50.0f;

    return speed;
}