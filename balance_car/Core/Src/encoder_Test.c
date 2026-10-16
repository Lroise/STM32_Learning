#include "encoder_test.h"
#include "app_encoder.h"
#include "app_button.h"     // 按键驱动
#include "app_pwm.h"        // 电机驱动
#include "usart.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 声明 main.c 里的串口打印函数，避免重复定义
void My_USART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);

// 上一次的位置，用来算 1ms 内的位移
static float last_pos_l = 0.0f;
static float last_pos_r = 0.0f;

/**
  * @brief  M 法测速测试 + 按键控制电机
  * @note   每 1ms 采样一次，每 50ms 打印一次，避免串口堵死
  *         每 1ms 调用一次 App_Button_Proc()，按键按下切换电机转/停
  *         调用后进入死循环，不会返回
  */
void Encoder_M_Method_Test(void)
{
    App_Encoder_Init();       // 启动 TIM1/TIM2 编码器模式

    uint32_t print_cnt = 0;   // 打印计数器，用来控制打印频率

    while(1)
    {
        App_Button_Proc();    // 按键扫描，必须周期调用
                              // 按键按下会调用 Motor_SetSpeed()，改变电机状态

        HAL_Delay(1);         // 采样周期 1ms

        // 读当前位置（累计脉冲数）
        float pos_l = App_Encoder_GetPos_L();
        float pos_r = App_Encoder_GetPos_R();

        // 这 1ms 内走了多少脉冲
        float M_l = pos_l - last_pos_l;
        float M_r = pos_r - last_pos_r;

        // 速度 = 位移 / 时间（1ms = 0.001s），单位：脉冲/秒
        float omega_l = M_l / 0.001f;
        float omega_r = M_r / 0.001f;

        // 更新上次位置，为下一轮做准备
        last_pos_l = pos_l;
        last_pos_r = pos_r;

        // 每 50ms 打印一次
        print_cnt++;
        if(print_cnt >= 50)
        {
            print_cnt = 0;
            // VOFA+ FireWater 协议要求 \r\n 结尾
            My_USART_Printf(&huart2, "%.2f,%.2f,%.2f,%.2f\r\n",
                            pos_l, pos_r, omega_l, omega_r);
        }
    }
}