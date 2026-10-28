#include "echo.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

uint16_t echo_sta = 0;  
uint16_t echo_dval = 0;        
float Distance;

static void echo_recv_timeout(void)
{
    if(echo_sta & 0X40)  
    {
        echo_sta |= 0X80;   
    }
}

void echo_init(void)                                         
{
    TIM_IC_InitTypeDef tim_ic_init_handle;

    // 因为 CubeMX 配置了 TIM1_CH1，这里直接配置输入捕获通道的参数
    tim_ic_init_handle.ICPolarity = TIM_ICPOLARITY_RISING;      
    tim_ic_init_handle.ICSelection = TIM_ICSELECTION_DIRECTTI;  
    tim_ic_init_handle.ICPrescaler = TIM_ICPSC_DIV1;            
    tim_ic_init_handle.ICFilter = 0x03;                         
    HAL_TIM_IC_ConfigChannel(&htim1, &tim_ic_init_handle, TIM_CHANNEL_1);  
    
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);           
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);           
}

// 注意：这个回调函数是弱定义，直接覆盖即可
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)            
    {
        echo_recv_timeout();		
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)     
    {
        if (!(echo_sta & 0X40))      
        {
            echo_sta = 0X40;           
            __HAL_TIM_SET_COUNTER(&htim1, 0);   						
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
        }
        else if (echo_sta & 0X40 && !(echo_sta & 0X80)) 
        {
            echo_dval = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  
            echo_sta |= 0X80;        															
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
        }
    }
}

void echo_value_get(void)                          
{                                             
    echo_sta = 0;    
    echo_dval = 0;
    
    HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_SET);    			
    delay_us(20);  																							
    HAL_GPIO_WritePin(ULTRA_TRIG_GPIO_Port, ULTRA_TRIG_Pin, GPIO_PIN_RESET);				
    
    uint32_t timeout = 0;
    while(!(echo_sta&0X80) && timeout < 30000)  
    {
        timeout++;
        delay_us(1);
    }
    
    if(echo_sta&0X80 && echo_dval > 0)
    {
        Distance = (echo_dval * 0.017f) / 100.0f;  
        echo_sta = 0;                          
    }
    else
    {
        Distance = 0.0f;  
    }
}