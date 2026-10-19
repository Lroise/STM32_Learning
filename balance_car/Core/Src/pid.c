#include "pid.h"

/**
  * @brief  对PID控制器进行初始化
  */
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
    PID->SP = 0.0f;
    
    PID->t_k_1 = 0;
    PID->err_k_1 = 0.0f;
    PID->err_int_k_1 = 0.0f;

    // ================= 新增：初始化限幅默认值为无穷大 =================
    PID->UpperLimit = +3.4e+38f;
    PID->LowerLimit = -3.4e+38f;
}

/**
  * @brief  改变设定值SP
  */
void PID_ChangeSP(PID_TypeDef *PID, float SP)
{
    PID->SP = SP;
}

// ================= 新增：设置限幅 =================
/**
  * @brief  设置PID控制器门限
  * @param  Upper: 上限值
  * @param  Lower: 下限值
  */
void PID_LimitConfig(PID_TypeDef *PID, float Upper, float Lower)
{
    PID->UpperLimit = Upper;
    PID->LowerLimit = Lower;
}

/**
  * @brief  执行一次PID运算
  * @param  FB: 反馈值（当前的实际速度）
  * @retval PID控制器的计算结果
  */
float PID_Compute(PID_TypeDef *PID, float FB)
{
    float err = PID->SP - FB;  // 计算误差
    
    // 获取微秒时间 (HAL库用 __HAL_TIM_GET_COUNTER 或者 DWT，这里为了简便暂时用 ms 代替，但乘了 1000)
    uint64_t t_k = HAL_GetTick() * 1000; 
    
    float deltaT = (t_k - PID->t_k_1) * 1.0e-6f;
    
    // 防止第一次计算或时间差过小导致微分爆炸
    if(deltaT <= 0.0f || deltaT > 0.1f) deltaT = 0.001f; 

    // ================= 新增：首次运行/复位保护 =================
    float err_dev = 0.0f;
    float err_int = 0.0f;
    
    if(PID->t_k_1 != 0)
    {
        // 微分项
        err_dev = (err - PID->err_k_1) / deltaT;
        
        // 积分项（梯形积分）
        err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;
    }

    // 计算 P、I、D 输出
    float COp = PID->Kp * err;
    float COi = PID->Ki * err_int;
    float COd = PID->Kd * err_dev;
    
    // 总输出
    float CO = COp + COi + COd;
    
    // 更新历史值
    PID->t_k_1 = t_k;
    PID->err_k_1 = err;
    PID->err_int_k_1 = err_int;

    // ================= 新增：输出限幅 =================
    if(CO > PID->UpperLimit) CO = PID->UpperLimit;
    if(CO < PID->LowerLimit) CO = PID->LowerLimit;

    // ================= 新增：积分限幅（抗积分饱和） =================
    if(PID->err_int_k_1 > PID->UpperLimit) PID->err_int_k_1 = PID->UpperLimit;
    if(PID->err_int_k_1 < PID->LowerLimit) PID->err_int_k_1 = PID->LowerLimit;

    return CO;
}

// ================= 新增：复位 PID 状态 =================
/**
  * @brief  对PID控制器进行复位（清零历史积分和误差）
  * @note   在按键启动电机时调用，防止历史积分导致起飞
  */
void PID_Reset(PID_TypeDef *PID)
{
    PID->t_k_1 = 0;
    PID->err_k_1 = 0.0f;
    PID->err_int_k_1 = 0.0f;
}