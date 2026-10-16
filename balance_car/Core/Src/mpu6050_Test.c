#include "mpu6050_test.h"
#include "app_mpu6050.h"
#include "app_button.h"     // 按键驱动
#include "app_pwm.h"        // 电机驱动
#include "usart.h"
#include "main.h"
#include "task.h"           // PERIODIC 宏
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 声明 main.c 里的串口打印函数，避免重复定义
void My_USART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);

// 内部函数声明
static void USART2_Proc(void);

/**
  * @brief  MPU6050 测试
  * @note   每 10ms 读一次，打印 7 个物理量（加速度、温度、角速度）
  *         进去后进入死循环，不会返回
  */
void MPU6050_Test(void)
{
    App_MPU6050_Init();       // 初始化 MPU6050

    while(1)
    {
        App_MPU6050_Update(); // 更新传感器数据

        // 读加速度（单位：g）
        float ax = App_MPU6050_GetAx();
        float ay = App_MPU6050_GetAy();
        float az = App_MPU6050_GetAz();

        // 读温度（单位：摄氏度）
        float temperature = App_MPU6050_GetTemperature();

        // 读角速度（单位：度/秒）
        float gx = App_MPU6050_GetGx();
        float gy = App_MPU6050_GetGy();
        float gz = App_MPU6050_GetGz();

        // 打印 7 个值：ax、ay、az、温度、gx、gy、gz
        My_USART_Printf(&huart2, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                        ax, ay, az, temperature, gx, gy, gz);

        HAL_Delay(10);        // 10ms 打印一次
    }
}

/**
  * @brief  按键 + 电机 + MPU6050
  * @note   每 5ms 跑一次 Proc（陀螺仪积分算欧拉角），每 10ms 打印一次
  *         每 5ms 调用一次 App_Button_Proc()，按键按下切换电机转/停
  *         进去后进入死循环，不会返回
  */
void MPU6050_EularAngleTest(void)
{
    App_MPU6050_Init();       // 初始化 MPU6050

    while(1)
    {
        App_Button_Proc();    // 按键扫描，按键按下切换电机转/停
        App_MPU6050_Proc();   // MPU6050 进程，每 5ms 跑一次
        USART2_Proc();        // 打印函数，每 10ms 跑一次
    }
}

/**
  * @brief  每 10ms 把 MPU6050 数据发给电脑
  * @note   用 PERIODIC(10) 做定时，不阻塞
  */
static void USART2_Proc(void)
{
    PERIODIC(10)              // 每 10ms 执行一次，未到时间直接 return

    // 读加速度（单位：g）
    float ax = App_MPU6050_GetAx();
    float ay = App_MPU6050_GetAy();
    float az = App_MPU6050_GetAz();

    // 读温度（单位：摄氏度）
    float temperature = App_MPU6050_GetTemperature();

    // 读角速度（单位：度/秒）
    float gx = App_MPU6050_GetGx();
    float gy = App_MPU6050_GetGy();
    float gz = App_MPU6050_GetGz();

    // 读欧拉角（单位：度）
    float yaw   = App_MPU6050_GetYaw();
    float pitch = App_MPU6050_GetPitch();
    float roll  = App_MPU6050_GetRoll();

    // 打印 10 个值：ax、ay、az、温度、gx、gy、gz、yaw、pitch、roll
    // VOFA+ FireWater 协议要求 \r\n 结尾
    My_USART_Printf(&huart2,
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        ax, ay, az, temperature, gx, gy, gz, yaw, pitch, roll);
}