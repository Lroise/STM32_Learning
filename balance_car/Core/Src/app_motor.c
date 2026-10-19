#include "app_motor.h"
#include "pid.h"
#include "app_encoder.h"
#include "app_pwm.h"

static PID_TypeDef pid_motor_l;  // 左电机调速系统的PID控制器
static PID_TypeDef pid_motor_r;  // 右电机调速系统的PID控制器

// ================= 新增：全局电机使能标志 =================
static uint8_t motor_enable = 0; // 0: 关，1: 开

/**
  * @brief  初始化左右电机调速系统
  */
void App_Motor_Init(void)
{
    // 铁头山羊给的参数：Kp=0.5, Ki=7, Kd=0
    PID_Init(&pid_motor_l, 0.5, 7, 0);
    PID_Init(&pid_motor_r, 0.5, 7, 0);

    // 配置速度环输出的最大/最小电压限制
    // 限制输出的电压范围，防止积分饱和导致电机起飞
    PID_LimitConfig(&pid_motor_l, +8.4f, -8.4f);
    PID_LimitConfig(&pid_motor_r, +8.4f, -8.4f);
}

/**
  * @brief  开关电机
  * @param  On: 控制电机的开关，0 - 关闭，非0 - 开启
  * @note   在开启或关闭电机时，会对 PID 控制器进行复位，防止历史积分导致起飞
  */
void App_Motor_Cmd(uint8_t On)
{
    // 记录当前电机开关状态
    motor_enable = On; 

    if(On)
    {
        // 开启 TB6612 使能
        TB6612_Enable(1); 
        // 启动瞬间必须复位 PID 历史积分，防止“积分饱和”导致一启动就飞车
        PID_Reset(&pid_motor_l);
        PID_Reset(&pid_motor_r);
    }
    else
    {
        // 关闭 TB6612 使能
        TB6612_Enable(0);
    }
}

/**
  * @brief  电机调速系统的进程函数
  * @note   每间隔 1ms 执行一次
  */
void App_Motor_Proc(void)
{
    // 假设你已经有了一个微秒级的定时或者直接用 1ms 的 Tick
    static uint32_t last_tick = 0;
    if (HAL_GetTick() - last_tick < 1) return; 
    last_tick = HAL_GetTick();

    // 如果电机没开启，强制输出 0 并直接退出
    // 这样就算你用手去拨动轮子，产生速度反馈，PID 也不会被唤醒乱转
    if (motor_enable == 0) 
    {
        Motor_SetSpeed(0, 0); 
        return; 
    }

    // #1. 通过编码器获取左右电机旋转的角速度
    float omega_l = App_Encoder_GetSpeed_L();
    float omega_r = App_Encoder_GetSpeed_R();

    // #2. 计算PID控制器的输出
    float ua_l = PID_Compute(&pid_motor_l, omega_l);
    float ua_r = PID_Compute(&pid_motor_r, omega_r);

    // #3. 将电压Ua设置为电机两端 (假设电池 7.4V)
    float vbat = 7.4f; 
    
    // 计算左右电机PWM信号的占空比 (目标范围 -100 到 100)
    float duty_l = ua_l / vbat * 100.0f;
    float duty_r = ua_r / vbat * 100.0f;

    // 限幅
    if(duty_l > 100.0f) duty_l = 100.0f;
    if(duty_l < -100.0f) duty_l = -100.0f;
    if(duty_r > 100.0f) duty_r = 100.0f;
    if(duty_r < -100.0f) duty_r = -100.0f;

    // 注意：你的 Motor_SetSpeed 接收的是 -999 ~ 999，所以要乘 9.99
    Motor_SetSpeed((int16_t)(duty_l * 9.99f), (int16_t)(duty_r * 9.99f));
}

/**
  * @brief  用来设置左电机的目标转速
  */
void App_Motor_SetOmega_L(float Omega)
{
    PID_ChangeSP(&pid_motor_l, Omega);
}

/**
  * @brief  用来设置右电机的目标转速
  */
void App_Motor_SetOmega_R(float Omega)
{
    PID_ChangeSP(&pid_motor_r, Omega);
}