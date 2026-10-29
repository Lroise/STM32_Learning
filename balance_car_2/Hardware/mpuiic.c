#include "mpuiic.h"
#include "main.h"

// 自定义微秒延时
static void MPU_Delay_us(uint32_t us)
{
    volatile uint32_t i;
    for(i = 0; i < us * 12; i++) { __NOP(); }
}

void MPU_IIC_Delay(void)
{
    // 【关键修改】从 10us 改为 2us，极速读取数据，配合主循环的 2ms 延时，彻底消灭控制滞后！
    MPU_Delay_us(2); 
}

void MPU6050_IIC_SDA_IO_OUT(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_PIN_10; // SDA 是 PB10
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP; 
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
}

void MPU6050_IIC_SDA_IO_IN(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = GPIO_PIN_10; // SDA 是 PB10
    gpio_init_struct.Mode = GPIO_MODE_INPUT; 
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
}

void MPU_IIC_Init(void)
{   
    GPIO_InitTypeDef gpio_init_struct;
    __HAL_RCC_GPIOB_CLK_ENABLE();                       
    gpio_init_struct.Pin = GPIO_PIN_10 | GPIO_PIN_11;     
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);  
    
    MPU_IIC_SDA_H();
    MPU_IIC_SCL_H();
    HAL_Delay(100); 
    
    // 【关键】发送 9 个时钟脉冲，强行解除 IIC 总线死锁状态！
    for(int i = 0; i < 9; i++) {
        MPU_IIC_SCL_L();
        MPU_IIC_Delay();
        MPU_IIC_SCL_H();
        MPU_IIC_Delay();
        /* === 面试解析 ===
         * 经典的工程防呆操作。如果单片机意外复位而传感器还在拉低 SDA，总线会死锁。
         * 强行发 9 个脉冲，让从机把最后一个字节（8位数据+1位ACK）发完，释放总线。
         */
    }
}

void MPU_IIC_Start(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU_IIC_SDA_H();
    MPU_IIC_SCL_H();
    MPU_IIC_Delay();
    MPU_IIC_SDA_L();
    MPU_IIC_Delay();
    MPU_IIC_SCL_L();               
}

void MPU_IIC_Stop(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU_IIC_SCL_L();
    MPU_IIC_SDA_L();
    MPU_IIC_Delay();
    MPU_IIC_SCL_H();  
    MPU_IIC_SDA_H();              
    MPU_IIC_Delay();
}

u8 MPU_IIC_Wait_Ack(void)
{
    u8 ucErrTime=0;
    MPU6050_IIC_SDA_IO_IN(); 
    
    MPU_IIC_SDA_H();  
    MPU_IIC_Delay();  
    MPU_IIC_SCL_H(); 
    MPU_IIC_Delay(); 
    while(MPU_READ_SDA)
    {
        ucErrTime++;
        MPU_IIC_Delay(); 
        if(ucErrTime > 250)
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_SCL_L();                  
    return 0;  
}

void MPU_IIC_Ack(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU_IIC_SCL_L();
    MPU_IIC_SDA_L();
    MPU_IIC_Delay();
    MPU_IIC_SCL_H();
    MPU_IIC_Delay();
    MPU_IIC_SCL_L();
}

void MPU_IIC_NAck(void)
{
    MPU6050_IIC_SDA_IO_OUT();
    MPU_IIC_SCL_L();
    MPU_IIC_SDA_H();
    MPU_IIC_Delay();
    MPU_IIC_SCL_H();
    MPU_IIC_Delay();
    MPU_IIC_SCL_L();
} 

void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
    MPU6050_IIC_SDA_IO_OUT();
    MPU_IIC_SCL_L();      
    for(t=0; t<8; t++)
    {              
        if(txd & 0x80) MPU_IIC_SDA_H();
        else MPU_IIC_SDA_L();
        txd <<= 1; 	  
        MPU_IIC_SCL_H();
        MPU_IIC_Delay(); 
        MPU_IIC_SCL_L();	
        MPU_IIC_Delay();
    } 
}

u8 MPU_IIC_Read_Byte(unsigned char ack)
{
    unsigned char i,receive=0;
    MPU6050_IIC_SDA_IO_IN(); 
    for(i=0; i<8; i++ )
    {
        MPU_IIC_SCL_L(); 
        MPU_IIC_Delay();
        MPU_IIC_SCL_H();
        receive <<= 1;
        if(MPU_READ_SDA) receive++;   
        MPU_IIC_Delay(); 
    }
    if (!ack) MPU_IIC_NAck();         
    else MPU_IIC_Ack();          
    return receive;
}