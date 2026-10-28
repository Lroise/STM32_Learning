#ifndef _UART_H
#define _UART_H
#include "stdio.h"

#include "delay.h"
#include "main.h"


#define USART_REC_LEN           200     //定义最大接收字节数 200
/******************************************************************************************/
/* 外部接口函数*/
void usart_init(uint32_t baudrate);
/******************************************************************************************/
#endif
