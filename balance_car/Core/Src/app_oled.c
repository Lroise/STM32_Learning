#include "app_oled.h"
#include <string.h>

// ================== 引脚宏定义 (PB8 和 PB9) ==================
#define SCL_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define SCL_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define SDA_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define SDA_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)

// ================== 极简 6x8 字库 ==================
// 包含: 0-9, A-Z, a-z, 标点符号，足够调试用
const unsigned char F6x8[][6] = {
    /* 0 */ 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00,
    /* 1 */ 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00,
    /* 2 */ 0x42, 0x61, 0x51, 0x49, 0x46, 0x00,
    /* 3 */ 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00,
    /* 4 */ 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00,
    /* 5 */ 0x27, 0x45, 0x45, 0x45, 0x39, 0x00,
    /* 6 */ 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00,
    /* 7 */ 0x01, 0x71, 0x09, 0x05, 0x03, 0x00,
    /* 8 */ 0x36, 0x49, 0x49, 0x49, 0x36, 0x00,
    /* 9 */ 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00,
    /* : */ 0x00, 0x36, 0x36, 0x00, 0x00, 0x00,
    /* . */ 0x00, 0x60, 0x60, 0x00, 0x00, 0x00,
    /* - */ 0x08, 0x08, 0x08, 0x08, 0x08, 0x00,
    /* + */ 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00,
    /* P */ 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00,
    /* i */ 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00,
    /* t */ 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00,
    /* c */ 0x38, 0x44, 0x44, 0x44, 0x20, 0x00,
    /* h */ 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00,
    /* G */ 0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00,
    /* y */ 0x1C, 0xA0, 0xA0, 0xA0, 0x7C, 0x00,
    /* r */ 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00,
    /* o */ 0x38, 0x44, 0x44, 0x44, 0x38, 0x00,
    /* X */ 0x63, 0x14, 0x08, 0x14, 0x63, 0x00,
    /* R */ 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00,
    /* u */ 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00,
    /* n */ 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00,
    /*   */ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// 获取字符在字库中的索引
static uint8_t Get_Char_Index(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c == ':') return 10;
    if (c == '.') return 11;
    if (c == '-') return 12;
    if (c == '+') return 13;
    if (c == 'P') return 14;
    if (c == 'i') return 15;
    if (c == 't') return 16;
    if (c == 'c') return 17;
    if (c == 'h') return 18;
    if (c == 'G') return 19;
    if (c == 'y') return 20;
    if (c == 'r') return 21;
    if (c == 'o') return 22;
    if (c == 'X') return 23;
    if (c == 'R') return 24;
    if (c == 'u') return 25;
    if (c == 'n') return 26;
    return 27; // 空格或其他字符
}

// ================== IIC 底层驱动 (同之前) ==================
static void IIC_Delay(void) { for(volatile int i = 0; i < 20; i++); }
static void IIC_Start(void) { SDA_H(); SCL_H(); IIC_Delay(); SDA_L(); IIC_Delay(); SCL_L(); IIC_Delay(); }
static void IIC_Stop(void) { SDA_L(); SCL_H(); IIC_Delay(); SDA_H(); IIC_Delay(); }
static void IIC_SendByte(uint8_t dat) {
    for(int i = 0; i < 8; i++) {
        if(dat & 0x80) SDA_H(); else SDA_L();
        dat <<= 1;
        SCL_H(); IIC_Delay(); SCL_L(); IIC_Delay();
    }
    SDA_H(); SCL_H(); IIC_Delay(); SCL_L(); IIC_Delay();
}
static void OLED_WR_CMD(uint8_t cmd) {
    IIC_Start(); IIC_SendByte(0x78); IIC_SendByte(0x00); IIC_SendByte(cmd); IIC_Stop();
}
static void OLED_WR_DATA(uint8_t data) {
    IIC_Start(); IIC_SendByte(0x78); IIC_SendByte(0x40); IIC_SendByte(data); IIC_Stop();
}

// ================== 功能实现 ==================
void App_OLED_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    HAL_Delay(100);

    OLED_WR_CMD(0xAE); OLED_WR_CMD(0x00); OLED_WR_CMD(0x10); OLED_WR_CMD(0x40);
    OLED_WR_CMD(0x81); OLED_WR_CMD(0xCF); OLED_WR_CMD(0xA1); OLED_WR_CMD(0xC8);
    OLED_WR_CMD(0xA6); OLED_WR_CMD(0xA8); OLED_WR_CMD(0x3F); OLED_WR_CMD(0xD3);
    OLED_WR_CMD(0x00); OLED_WR_CMD(0xD5); OLED_WR_CMD(0x80); OLED_WR_CMD(0xD9);
    OLED_WR_CMD(0xF1); OLED_WR_CMD(0xDA); OLED_WR_CMD(0x12); OLED_WR_CMD(0xDB);
    OLED_WR_CMD(0x40); OLED_WR_CMD(0x8D); OLED_WR_CMD(0x14); OLED_WR_CMD(0xAF);
    
    App_OLED_Clear();
}

void App_OLED_Clear(void)
{
    for(uint8_t i = 0; i < 8; i++) 
    {
        OLED_WR_CMD(0xB0 + i);
        OLED_WR_CMD(0x00); OLED_WR_CMD(0x10); 
        for(uint8_t n = 0; n < 128; n++) OLED_WR_DATA(0x00); 
    }
}

void App_OLED_Test(void)
{
    for(uint8_t i = 0; i < 8; i++) 
    {
        OLED_WR_CMD(0xB0 + i); OLED_WR_CMD(0x00); OLED_WR_CMD(0x10); 
        for(uint8_t n = 0; n < 128; n++) OLED_WR_DATA((i % 2 == 0) ? 0xFF : 0x00); 
    }
}

// 【新增】显示字符串函数
// x: 0~127, y: 0~7 (页)
void App_OLED_ShowString(uint8_t x, uint8_t y, char *str)
{
    uint8_t i = 0;
    while(str[i] != '\0' && x < 128)
    {
        uint8_t idx = Get_Char_Index(str[i]);
        
        OLED_WR_CMD(0xB0 + y);                    // 设置页地址
        OLED_WR_CMD(((x & 0xF0) >> 4) | 0x10);    // 设置列高地址
        OLED_WR_CMD((x & 0x0F));                  // 设置列低地址
        
        for(int j = 0; j < 6; j++)
        {
            OLED_WR_DATA(F6x8[idx][j]);
        }
        
        x += 6; // 每个字符宽 6 像素
        i++;
    }
}