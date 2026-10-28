#include "app_encoder.h"
#include "tim.h"
#include "math.h"

#define ENCODER_PPR   11
#define GEAR_RATIO    30
#define PULSE_PER_REV (ENCODER_PPR * GEAR_RATIO * 4) // 一圈总脉冲数 1320

static int16_t last_cnt_l = 0;
static int16_t last_cnt_r = 0;
static float pos_l = 0.0f;
static float pos_r = 0.0f;

static float last_pos_l_speed = 0.0f;
static float last_pos_r_speed = 0.0f;
static uint32_t last_time_l_speed = 0;
static uint32_t last_time_r_speed = 0;

void App_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动左轮编码器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // 启动右轮编码器

    last_cnt_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    last_cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    pos_l = 0.0f;
    pos_r = 0.0f;
    last_pos_l_speed = 0.0f;
    last_pos_r_speed = 0.0f;
    last_time_l_speed = HAL_GetTick();
    last_time_r_speed = HAL_GetTick();
}

void App_Encoder_Update(uint16_t *p_enc1, uint16_t *p_enc2)
{
    *p_enc1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    *p_enc2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

float App_Encoder_GetPos_L(void)
{
    int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
    int16_t diff = (int16_t)(last_cnt_l - now); // 计算脉冲增量，防止溢出
    last_cnt_l = now;
    pos_l += (float)diff;
    return pos_l;
}

float App_Encoder_GetPos_R(void)
{
    int16_t now  = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = (int16_t)(last_cnt_r - now);   
    last_cnt_r = now;
    pos_r += (float)diff;
    return pos_r;
}

float App_Encoder_GetSpeed_L(void)
{
    float pos = App_Encoder_GetPos_L();
    float diff = pos - last_pos_l_speed; // 两次读取的位置差
    last_pos_l_speed = pos;
    
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time_l_speed) * 0.001f; // 时间差
    last_time_l_speed = now;
    if (dt <= 0.0f) dt = 0.001f; 
    
    // 转速(rad/s) = (脉冲差 / 时间) / 1320 * 2PI
    float rev_per_sec = (diff / dt) / (float)PULSE_PER_REV;
    float speed = rev_per_sec * 2.0f * 3.1415926f;

    if (speed > 50.0f)  speed = 50.0f;   // 限幅
    if (speed < -50.0f) speed = -50.0f;
    return speed;
}

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

    if (speed > 50.0f)  speed = 50.0f;
    if (speed < -50.0f) speed = -50.0f;
    return -speed; // 右轮反向，所以这里要加负号
}