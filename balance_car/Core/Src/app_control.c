#include "app_control.h"
#include "app_pwm.h"
#include "app_encoder.h"
#include "app_mpu6050.h"
#include "app_motor.h"
#include "math.h"

#define Med_Angle      -1.10f     
#define Vertical_Kp    80.0f      // 【安全值】降到80
#define Vertical_Kd    1.0f       // 【安全值】慢慢来
#define Velocity_Kp    0.0f      
#define Velocity_Ki    0.0f       

void App_Control_Init(void) { }

void App_Control_Proc(void)
{
    float Pitch  = App_MPU6050_GetPitch();
    float Gyro_X = App_MPU6050_GetGx(); 
    
    if (Pitch > 30.0f || Pitch < -30.0f) 
    {
        Motor_SetSpeed(0, 0);
        return; 
    }

    float Vertical_out = Vertical_Kp * (Pitch - Med_Angle) + Vertical_Kd * Gyro_X;

    // 死区补偿调回 80
    if (Vertical_out > 0.0f && Vertical_out < 80.0f)  Vertical_out = 80.0f;
    if (Vertical_out < 0.0f && Vertical_out > -80.0f) Vertical_out = -80.0f;

    // 【核心保护 1】最大输出绝对不超过 400！直接限制死电流
    if(Vertical_out > 400.0f) Vertical_out = 400.0f;
    if(Vertical_out < -400.0f) Vertical_out = -400.0f;

    // 【核心保护 2】输出平滑过滤（防止瞬间PWM突变）
    static float last_out = 0.0f;
    Vertical_out = last_out * 0.6f + Vertical_out * 0.4f; // 平滑
    last_out = Vertical_out;

    Motor_SetSpeed((int16_t)Vertical_out, (int16_t)Vertical_out);
}