#ifndef __TASK_H
#define __TASK_H

#include "stm32f1xx_hal.h"

// 用 HAL 库自带的毫秒计时
#define GetTick()  HAL_GetTick()

/**
  * @brief  周期性任务宏
  * @note   PERIODIC(T)：如果未到下次执行时间，return；
  *         到了就执行，并把下次执行时间往后推 T 毫秒。
  *         用法：
  *         void Task(void)
  *         {
  *             PERIODIC(5)   // 每 5ms 执行一次
  *             // 任务代码
  *         }
  */
#define PERIODIC(T)                        \
    static uint32_t nxt = 0;               \
    if(GetTick() < nxt) return;            \
    nxt += (T);

#endif