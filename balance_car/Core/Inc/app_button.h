#ifndef __APP_BUTTON_H      //头文件保护
#define __APP_BUTTON_H

#include"stm32f1xx_hal.h"
#include"app_pwm.h"

void App_Button_Init(void);      //按键模块初始化
void App_Button_Proc(void);      //按键扫描主逻辑

#endif