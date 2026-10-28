#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

#define u8 uint8_t
#define u16 uint16_t

// 【强制确认】SCL 是 PB11，SDA 是 PB10
#define MPU_IIC_SCL_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define MPU_IIC_SCL_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)

#define MPU_IIC_SDA_H()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define MPU_IIC_SDA_L()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)

#define MPU_READ_SDA      HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)

void MPU_IIC_Delay(void);               
void MPU_IIC_Init(void);                
void MPU_IIC_Start(void);               
void MPU_IIC_Stop(void);                
void MPU_IIC_Send_Byte(u8 txd);         
u8 MPU_IIC_Read_Byte(unsigned char ack);
u8 MPU_IIC_Wait_Ack(void);              
void MPU_IIC_Ack(void);                 
void MPU_IIC_NAck(void);                
void MPU6050_IIC_SDA_IO_OUT(void);
void MPU6050_IIC_SDA_IO_IN(void);

#endif