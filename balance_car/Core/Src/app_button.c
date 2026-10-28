#include"app_button.h"
#include"app_pwm.h"
#include"app_motor.h"

// 引入主程序里的目标速度（虽然这里没直接用到，但为了链接不报错保留）
extern float targetOmega; 

//=====按键配置区=====
#define KEY1_PORT GPIOB
#define KEY1_PIN GPIO_PIN_1
//====================

// 按键状态变量
uint8_t key1_last = 1;            // 记录上一次的电平（松开为高电平 1）
uint32_t key1_cnt = 0;            // 消抖计数器
uint8_t key1_press_flag = 0;      // 标记：是否确认按键已经按下 0=没按下，1=按下

uint8_t motor_run_flag = 0;       // 电机运行标志位（0关，1开）

/**
  * @brief  按键 1 单击回调函数
  * @note   每按一次，电机状态翻转一次（开->关，关->开）
  */
static void Key1_Click(void)
{
    motor_run_flag = !motor_run_flag;      // 状态翻转
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 翻转 LED 指示
    
    if (motor_run_flag == 1)
    {
        // 开启电机总闸（底层使能）
        App_Motor_Cmd(1);   
    }
    else
    {
        // 关闭电机总闸
        App_Motor_Cmd(0);   
        targetOmega = 0.0f; // 清空目标速度
    }
}

void App_Button_Init(void)
{
    // CubeMX 已经在 gpio.c 里初始化了引脚，这里留空
}

/**
  * @brief  按键扫描函数（必须放在 while(1) 中不断调用）
  * @note   带消抖逻辑，防止按一下触发多次
  */
void App_Button_Proc(void)
{
    // 读取当前按键电平
    uint8_t key1_now = HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN);
    
    // 如果电平发生变化（比如按下变0，或松开变1）
    if(key1_now != key1_last)
    {
        key1_cnt = 2;              // 重置消抖计数器（给几次轮询的时间稳定）
        key1_last = key1_now;      // 更新记录的上一次电平
    }
    
    if(key1_cnt > 0) key1_cnt--;   // 每次调用递减，减到 0 说明电平稳定了
    
    // 如果计数器归零，且电平确实稳定了
    if(key1_cnt == 0)
    {
        // 1. 捕捉按下动作
        if(key1_now == 0 && key1_press_flag == 0)
        {
            key1_press_flag = 1;   // 标记为已按下
        }
        
        // 2. 捕捉松开动作（此时说明完成了一次完整的点击）
        if(key1_now == 1 && key1_press_flag == 1)
        {
            key1_press_flag = 0;   // 清除按下标记
            Key1_Click();          // 触发单击回调
        }
    }
}