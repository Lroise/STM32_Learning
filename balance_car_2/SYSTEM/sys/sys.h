#ifndef __SYS_H
#define __SYS_H

#include "stm32f1xx.h"

/*函数申明*******************************************************************************************/
void stm32_sys_temClock_Config(uint32_t plln);                                              /* 系统时钟初始化函数 */
void Error_Handler(void);                                                                   /*  错误函数 */

#endif











