#include "app_pwm.h"
#include "tim.h"
#include "gpio.h"

#define AIN1_GPIO       GPIOA
#define AIN1_PIN        GPIO_PIN_5
#define AIN2_GPIO       GPIOA
#define AIN2_PIN        GPIO_PIN_4
#define BIN1_GPIO       GPIOB
#define BIN1_PIN        GPIO_PIN_15
#define BIN2_GPIO       GPIOB
#define BIN2_PIN        GPIO_PIN_14
#define STBY_GPIO       GPIOB
#define STBY_PIN        GPIO_PIN_13

void TB6612_Enable(uint8_t en) {
    if(en == 1) HAL_GPIO_WritePin(STBY_GPIO, STBY_PIN, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(STBY_GPIO, STBY_PIN, GPIO_PIN_RESET);
}

void Motor_SetSpeed(int16_t left,int16_t right)
{
    if(left > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO, AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left);
    } else if(left < 0) {
        HAL_GPIO_WritePin(AIN1_GPIO, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO, AIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -left);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO, AIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    }

    if(right > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO, BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);
    } else if(right < 0) {
        HAL_GPIO_WritePin(BIN1_GPIO, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO, BIN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -right);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO, BIN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
}

void App_PWM_Init(void) {
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}