#ifndef __APP_MPU6050_H
#define __APP_MPU6050_H

#include "stm32f1xx_hal.h"

// 初始化 MPU6050
void App_MPU6050_Init(void);

// MPU6050 进程函数，每 5ms 调用一次
void App_MPU6050_Proc(void);

// 更新 MPU6050 数据
void App_MPU6050_Update(void);

// 获取加速度（单位：g）
float App_MPU6050_GetAx(void);
float App_MPU6050_GetAy(void);
float App_MPU6050_GetAz(void);

// 获取温度（单位：摄氏度）
float App_MPU6050_GetTemperature(void);

// 获取角速度（单位：度/秒）
float App_MPU6050_GetGx(void);
float App_MPU6050_GetGy(void);
float App_MPU6050_GetGz(void);

// 获取欧拉角（单位：度）
float App_MPU6050_GetYaw(void);
float App_MPU6050_GetPitch(void);
float App_MPU6050_GetRoll(void);

#endif