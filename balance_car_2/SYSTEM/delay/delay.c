#include "delay.h"

static uint32_t g_fac_us = 0;

void delay_init(uint16_t sysclk)
{
    g_fac_us = sysclk;                                  
}

// 【关键修改】废弃 SysTick 延时，改用纯软件死循环。绝对安全，绝不卡死！
void delay_us(uint32_t n_us)
{
    uint32_t i;
    for(i = 0; i < n_us * 12; i++) 
    {
        __NOP(); 
    }
}

void delay_ms(uint16_t n_ms)
{
    delay_us((uint32_t)(n_ms * 1000));                   
}