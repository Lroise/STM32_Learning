#include "Control.h"
#include "main.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "Encoder.h"
#include "Motor.h"
#include "uart.h"
#include "Buzzer.h" // 【新增】引入蜂鸣器头文件

// ---- 变量定义区 ----
float Roll,Pitch,Yaw;       
short gyrox,gyroy,gyroz;    
short aacx,aacy,aacz;       

int16_t Get_EncoderL,Get_EncoderR;        
int16_t MOTO1,MOTO2;                      
int16_t PWM_MAX = 6200,PWM_MIN  = -6200 ; 

int16_t Vertical_out;               
int16_t Velocity_out;               
int16_t Turn_out;                   

extern uint8_t Fore,Back,Left,Right;
extern float Distance; // 【新增】引入超声波距离变量

int16_t Target_Speed = 0;               
int16_t Turn_Speed = 0;                 

// 【核心参数】（直立环，保持原样没动）
float Med_Angle   = -1.0;   
float Vertical_Kp = 300.0;   
float Vertical_Kd = -0.6;    

// 【速度环参数】（保持原样没动）
float Velocity_Kp = 40.0;    
float Velocity_Ki = 2.0;     

// 【转向环参数】（新增：开启转向）
float Turn_Kp     = 15.0;    // 转向比例
float Turn_Kd     = 0.5;     // 转向微分

int16_t test_cnt = 0;

void Control_Loop(void)
{
    test_cnt++; 
    
    int16_t PWM_OUT;                              
    
    // 1. 采集数据
    Get_EncoderL = Read_Speed(4);             
    Get_EncoderR = Read_Speed(3);            
    mpu_dmp_get_data(&Pitch,&Roll,&Yaw);      
    MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);  

    // 倒地保护
    if (Roll > 35.0 || Roll < -35.0) {
        Load(0, 0); 
        return;     
    }

    // =========================================================================
    // 【新功能 1：自动刹车避障】
    // =========================================================================
    if (Distance > 0.0 && Distance < 15.0) { // 如果前方15cm内有障碍物
        Target_Speed = 0; // 强制目标速度归零，刹车
        Buzzer_On();      // 蜂鸣器报警
    } else {
        Buzzer_Off();     // 解除报警
    }

    // =========================================================================
    // 【新功能 2：蓝牙遥控速度指令】
    // =========================================================================
    if (Fore == 1)      Target_Speed = 5;   // 收到前进
    else if (Back == 1) Target_Speed = -5;  // 收到后退
    else                Target_Speed = 0;   // 没收到指令，原地保持

    // =========================================================================
    // 【新功能 3：蓝牙遥控转向指令】
    // =========================================================================
    if (Left == 1)       Turn_Speed = -10;  // 左转
    else if (Right == 1) Turn_Speed = 10;   // 右转
    else                 Turn_Speed = 0;    // 回正

    // 2. 计算直立环输出
    Vertical_out = Vertical(Med_Angle,Roll,gyrox);                   
    
    // 3. 计算速度环输出
    Velocity_out = Velocity(Target_Speed,Get_EncoderL,Get_EncoderR); 
    
    // 4. 计算转向环输出（利用 Z 轴角速度）
    Turn_out = Turn(Turn_Speed, gyroz);

    // 5. 最终叠加：基础动力 + 左右差速
    PWM_OUT = Vertical_out + Velocity_out;      
    
    // 【修改】左轮加上转向力，右轮减去转向力（实现差速转弯）
    MOTO1 = PWM_OUT + Turn_out;                   
    MOTO2 = PWM_OUT - Turn_out;                   
    
    Limit(&MOTO1,&MOTO2);                       
    Load(MOTO1,MOTO2);                          
}

int16_t Vertical(float Med_Angle,float Angle,float Gyro_X)
{
    int16_t PWM_Out;
    PWM_Out = Vertical_Kp * (Angle-Med_Angle) - Vertical_Kd * Gyro_X;
    return PWM_Out;
}

int16_t Velocity(int16_t Target_Speed,int16_t Encoder_left,int16_t Encoder_right)
{
    static int16_t Pwm_Out, Encoder_Err, Encoder_S, Encoder_Err_Lowout, Encoder_Err_Lowout_last;
    float a = 0.5; 
     
    Encoder_Err = (Encoder_left + Encoder_right) - Target_Speed; 
    
    Encoder_Err_Lowout = a * Encoder_Err + (1 - a) * Encoder_Err_Lowout_last; 
    Encoder_Err_Lowout_last = Encoder_Err_Lowout;                        
    
    Encoder_S += Encoder_Err_Lowout;                                       
    Encoder_S = Encoder_S > 10000 ? 10000 : (Encoder_S < -10000 ? -10000 : Encoder_S); 
    if(Roll > 40 || Roll < -40) Encoder_S = 0;                                           
    
    Pwm_Out = Velocity_Kp * Encoder_Err_Lowout + Velocity_Ki * Encoder_S; 
    
    if (Pwm_Out > 4000)  Pwm_Out = 4000;
    if (Pwm_Out < -4000) Pwm_Out = -4000;
    
    return Pwm_Out;
}

int16_t Turn(int16_t RC,int16_t gyro_Z)
{
    int16_t PWM_out;
    // 【修改】转向公式：P * 目标转向速度 + D * 实际Z轴旋转速度
    PWM_out = Turn_Kp * RC + Turn_Kd * gyro_Z;
    return PWM_out;
}