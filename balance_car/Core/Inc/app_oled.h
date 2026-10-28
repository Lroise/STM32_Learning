#ifndef __APP_OLED_H
#define __APP_OLED_H

#include "main.h"
#include <stdint.h>

void App_OLED_Init(void);
void App_OLED_Clear(void);
void App_OLED_Test(void);
void App_OLED_ShowString(uint8_t x, uint8_t y, char *str); // 【新增】

#endif