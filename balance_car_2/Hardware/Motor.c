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
    /* === 面试解析 ===
     * TIM2 的两个通道被配置为 PWM 模式。
     * 启动后，只要修改比较寄存器（CCR），就能改变引脚输出方波的占空比，从而控制电机转速。
     */
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
    /* === 面试解析 ===
     * 限幅保护：PID 计算出来的值可能会瞬间很大（比如倒地瞬间积分饱和）。
     * 如果直接把超大的值传给定时器，不仅会超出寄存器容量，还可能烧毁电机驱动。
     */
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
    /* === 面试解析 ===
     * 核心驱动逻辑：H桥控制。
     * 1. AIN1 和 AIN2 的高低电平组合决定电机正反转。
     * 2. 我们传入的 moto1 正负号代表了方向，所以速度值要取绝对值（Num_Abs）。
     */

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