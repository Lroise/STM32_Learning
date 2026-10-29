#include "uart.h"
#include "oled.h"
#include "main.h"

// 【修改说明】引入 CubeMX 生成的串口句柄，原名是 uart1_handle，现统一使用 huart1
extern UART_HandleTypeDef huart1;

uint8_t g_usart_rx_buf[USART_REC_LEN];     // 接收缓冲,最大USART_REC_LEN个字节
uint8_t usart_len = 0;
uint8_t buffer[1];         
// 【修改说明】这里是所有 Fore/Back/Left/Right 变量的唯一实体定义处，其它文件用 extern
uint8_t Fore, Back, Left, Right; 

/****************************************************************************
*函数名：fputc
*功  能：串口发送函数，重定向 printf
****************************************************************************/
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/****************************************************************************
*函数名：usart_init
*输  入：baudrate（此参数留作兼容，实际波特率由 CubeMX 配置）
*功  能：开启串口中断接收
****************************************************************************/
void usart_init(uint32_t baudrate)
{
    // 底层 MX_USART1_UART_Init() 已经做好了，这里只需要开启中断接收
    HAL_UART_Receive_IT(&huart1, (uint8_t *)buffer, 1); 
}

/****************************************************************************
*函数名：HAL_UART_RxCpltCallback
*输  入：huart:串口句柄
*功  能：串口数据接收回调函数，蓝牙数据处理在这里进行
****************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)                    
    {
        g_usart_rx_buf[0] = buffer[0]; // 只取最新单字节
        
        // 【修改说明】删除了 HAL_UART_Transmit 和 while 等待 TC 标志，防止阻塞中断导致死机！
        /* 
         * === 面试解析 ===
         * 这一步极其关键！很多新手会在串口接收中断里用 printf 打印收到的字符。
         * 这会导致极严重的阻塞，导致平衡车主循环卡死直接摔倒。
         * 面试时可以说：我知道中断服务函数（ISR）必须尽量短小快，所以我只做标志位判断，绝不在这里做打印或延时。
         */
        
        if (g_usart_rx_buf[0] == '1') { 
            Fore=0; Back=0; Left=0; Right=0; 
            OLED_ShowString(10,1,(uint8_t *)"Stop   ",8);
        }    
        else if (g_usart_rx_buf[0] == '2') { 
            Fore=1; Back=0; Left=0; Right=0; // 注意这里你原代码逻辑反了，我改成正向
            OLED_ShowString(10,1,(uint8_t *)"Foward ",8);
        }
        else if (g_usart_rx_buf[0] == '3') { 
            Fore=0; Back=1; Left=0; Right=0; 
            OLED_ShowString(10,1,(uint8_t *)"Back   ",8);
        }
        else if (g_usart_rx_buf[0] == '4') { 
            Fore=0; Back=0; Left=0; Right=1; 
            OLED_ShowString(10,1,(uint8_t *)"TurnRight",8);
        }
        else if (g_usart_rx_buf[0] == '5') { 
            Fore=0; Back=0; Left=1; Right=0; 
            OLED_ShowString(10,1,(uint8_t *)"TurnLeft ",8);
        }
        else {
            Fore=0; Back=0; Left=0; Right=0;
        }
        
        // 重新开启接收中断，等待下一个字节
        HAL_UART_Receive_IT(&huart1, (uint8_t *)buffer, 1);
        
        /* 
         * === 面试解析 ===
         * 为什么这行代码必须写在最后面？
         * 因为 HAL 库的接收中断是“单次”的。进了一次回调，接收就关闭了。
         * 必须在这个函数退出前，重新调用它把接收打开，否则小车就只能响应一次遥控指令。
         */
    }
}