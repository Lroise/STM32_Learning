#include "Motor.h"
#include "main.h"  // 确保能识别CubeMX的宏定义

extern int16_t PWM_MAX, PWM_MIN;
extern TIM_HandleTypeDef htim2; // 引入CubeMX的定时器句柄

// 因为 CubeMX 已经初始化了引脚，这里留空即可
void Motor_Init(void)
{
}

void PWM_Init(void) 
{
    // 直接启动两路PWM输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

uint16_t Num_Abs(int16_t P)
{
    return P > 0 ? P : (-P);
}   

void Limit(int16_t *motoA, int16_t *motoB)
{
    if(*motoA > PWM_MAX) *motoA = PWM_MAX;
    if(*motoA < PWM_MIN) *motoA = PWM_MIN;
    if(*motoB > PWM_MAX) *motoB = PWM_MAX;
    if(*motoB < PWM_MIN) *motoB = PWM_MIN;
}

void Load(int16_t moto1, int16_t moto2)
{
    // 左轮控制 (PB0, PB1 和 TIM2_CH1)
    if(moto1 > 0) {
        HAL_GPIO_WritePin(MOTOR_L_AIN1_GPIO_Port, MOTOR_L_AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_GPIO_Port, MOTOR_L_AIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_L_AIN1_GPIO_Port, MOTOR_L_AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_AIN2_GPIO_Port, MOTOR_L_AIN2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Num_Abs(moto1));

    // 右轮控制 (PA5, PA4 和 TIM2_CH2)
    if(moto2 > 0) {
        HAL_GPIO_WritePin(MOTOR_R_BIN1_GPIO_Port, MOTOR_R_BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_GPIO_Port, MOTOR_R_BIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_R_BIN1_GPIO_Port, MOTOR_R_BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_BIN2_GPIO_Port, MOTOR_R_BIN2_Pin, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Num_Abs(moto2));
}