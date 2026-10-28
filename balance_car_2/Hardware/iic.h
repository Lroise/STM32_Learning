#ifndef __IIC_H
#define __IIC_H

#include "stdio.h"
#include "main.h"
#include "delay.h"
/****************************************************************************
*IO操作
****************************************************************************/
#define IIC_SCL_H()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define IIC_SCL_L()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
                          
#define IIC_SDA_H()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)   
#define IIC_SDA_L()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)

#define IIC_SDA_R         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) /* 读取SDA */

/****************************************************************************
*外部接口函数
****************************************************************************/
void iicbus_init(void);            /* 初始化IIC的IO口 */
void iicbus_start(void);           /* 发送IIC开始信号 */
void iicbus_stop(void);            /* 发送IIC停止信号 */
void iicbus_ack(void);             /* IIC发送ACK信号 */
void iicbus_nack(void);            /* IIC不发送ACK信号 */
uint8_t iicbus_wait_ack(void);     /* IIC等待ACK信号 */
void iicbus_send_byte(uint8_t txd);/* IIC发送一个字节 */
uint8_t iicbus_read_byte(unsigned char ack);/* IIC读取一个字节 */

#endif
