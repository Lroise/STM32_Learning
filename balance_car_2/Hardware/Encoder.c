#include "Encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Encoder_TIM4_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

void Encoder_TIM3_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t Read_Speed(uint8_t TIMX)
{
    int16_t Speed_Value;
    switch(TIMX)
    {
        case 4: 
            // 【关键修改】给左轮加上负号，强行反转方向，让它和右轮同号！
            Speed_Value = -__HAL_TIM_GET_COUNTER(&htim4); 
            __HAL_TIM_SET_COUNTER(&htim4, 0);
            break; 
        case 3: 
            // 右轮保持正向
            Speed_Value = __HAL_TIM_GET_COUNTER(&htim3); 
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            break; 
        default: 
            Speed_Value = 0;
    }
    return Speed_Value;
}