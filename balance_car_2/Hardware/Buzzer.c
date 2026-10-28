#include "Buzzer.h"

void Buzzer_Init()                  // ·äÃùÆ÷³õÊ¼»¯ 
{
    __HAL_RCC_GPIOB_CLK_ENABLE();                      
    GPIO_InitTypeDef gpio_init_struct;                 

    gpio_init_struct.Pin = GPIO_PIN_4;                 
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;       
    gpio_init_struct.Pull = GPIO_PULLUP;               
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     
    HAL_GPIO_Init(GPIOB, &gpio_init_struct); 
    
}

void Buzzer_On()
{
    HAL_GPIO_WritePin(GPIOB, Buzzer, GPIO_PIN_RESET); 
}

void Buzzer_Off()
{
    HAL_GPIO_WritePin(GPIOB, Buzzer, GPIO_PIN_SET);   
}