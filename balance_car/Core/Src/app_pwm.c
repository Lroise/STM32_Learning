#include "app_pwm.h"
#include "tim.h"    //CubeMX生成的定时器htim3定义在这里
#include "gpio.h"   //CubeMX生成的GPIO初始化代码在这里

//---------------------引脚宏定义---------------------

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

/**
 * @brief  TB6612 芯片使能/待机
 * @param  en: 1 使能(正常工作)  0 待机锁电机
 */
//驱动芯片总开关
void TB6612_Enable(uint8_t en)
{
    if(en == 1)
    {
        HAL_GPIO_WritePin(STBY_GPIO, STBY_PIN, GPIO_PIN_SET);  //输出高电平
    }
    else
    {
        HAL_GPIO_WritePin(STBY_GPIO, STBY_PIN, GPIO_PIN_RESET); //输出低电平
    }
}
/**
 * @brief  设置左右电机转速
 * @param  left  左电机速度：范围 -999 ~ +999，正数正转，负数反转，0停止
 * @param  right 右电机速度：范围 -999 ~ +999，正数正转，负数反转，0停止
 * @note   真值表和铁头山羊TB6612完全一致
 *         AIN1=1 AIN2=0 → 正转
 *         AIN1=0 AIN2=1 → 反转
 *         AIN1=0 AIN2=0 → 停止
 */
//核心电机调速函数
void Motor_SetSpeed(int16_t left,int16_t right)
{
	//========================左电机控制========================
		if(left > 0)    // 正数：左电机【正转】
	{
			HAL_GPIO_WritePin(AIN1_GPIO, AIN1_PIN, GPIO_PIN_SET);   //AIN1=1
			HAL_GPIO_WritePin(AIN2_GPIO, AIN2_PIN, GPIO_PIN_RESET); //AIN2=0
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left);    //设置PWMA占空比=left
	}
	else if(left < 0)  //负数：左电机【反转】
	{
			HAL_GPIO_WritePin(AIN1_GPIO, AIN1_PIN, GPIO_PIN_RESET); //AIN1=0
			HAL_GPIO_WritePin(AIN2_GPIO, AIN2_PIN, GPIO_PIN_SET);   //AIN2=1
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -left);    //取绝对值设置PWM
	}
	else    // left == 0：电机停止
	{
			HAL_GPIO_WritePin(AIN1_GPIO, AIN1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AIN2_GPIO, AIN2_PIN, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);        //PWM输出0
	}
	//========================右电机控制========================
		if(right > 0)    // 正转
	{
			HAL_GPIO_WritePin(BIN1_GPIO, BIN1_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BIN2_GPIO, BIN2_PIN, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);
	}
	else if(right < 0)  //反转
	{
			HAL_GPIO_WritePin(BIN1_GPIO, BIN1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN2_GPIO, BIN2_PIN, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -right);
	}
	else    //停止
	{
			HAL_GPIO_WritePin(BIN1_GPIO, BIN1_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BIN2_GPIO, BIN2_PIN, GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}

}
void App_PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

