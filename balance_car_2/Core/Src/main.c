/* USER CODE BEGIN Header */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "Control.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "echo.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include "sys.h"
#include "delay.h"
#include "Buzzer.h" 
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
extern int16_t test_cnt; 
extern float Roll;       
extern int16_t Get_EncoderL, Get_EncoderR; 
extern int16_t Velocity_out; 
extern float Distance; 
/* USER CODE END PV */

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  
  /* USER CODE BEGIN 2 */
  
  delay_init(72); 
  OLED_Init();
  OLED_ShowString(1, 1, (uint8_t *)"System Init...", 16);
  
  Motor_Init();
  PWM_Init();
  Encoder_TIM3_Init();
  Encoder_TIM4_Init();
  echo_init(); 
  Buzzer_Init(); 

  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); 

  OLED_ShowString(1, 3, (uint8_t *)"Start DMP...  ", 16);
  uint8_t mpu_err = mpu_dmp_init(); 
  
  if(mpu_err == 0) 
  {
      OLED_ShowString(1, 3, (uint8_t *)"MPU OK!       ", 16);
  } 
  else 
  {
      OLED_ShowString(1, 3, (uint8_t *)"DMP ERR: ", 16);
      OLED_ShowNum(75, 3, mpu_err, 2, 16); 
      while(1); 
  }
  
  HAL_Delay(1000);
  OLED_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static int oled_cnt = 0;
  static int echo_cnt = 0; // 【新增】测距计数器
  while (1)
  {
      /* USER CODE BEGIN 3 */
      
      // 1. 极速执行控制环（绝对不被干扰）
      Control_Loop(); 
      
      // 2. 测距降频：每 50 次循环（大概 100ms）测一次距离
      echo_cnt++;
      if (echo_cnt >= 50) {
          echo_cnt = 0;
          echo_value_get();
      }
      
      // 3. 屏幕刷新
      oled_cnt++;
      if (oled_cnt >= 200) 
      {
          oled_cnt = 0;
          
          OLED_ShowString(1, 1, (uint8_t *)"Roll:", 16);
          if (Roll < 0) {
              OLED_ShowString(48, 1, (uint8_t *)"-", 16);
              OLED_ShowNum(56, 1, (uint32_t)(-Roll), 3, 16);
          } else {
              OLED_ShowString(48, 1, (uint8_t *)"+", 16);
              OLED_ShowNum(56, 1, (uint32_t)(Roll), 3, 16);
          }

          OLED_ShowString(1, 3, (uint8_t *)"Dist:", 16);
          OLED_ShowNum(48, 3, (uint32_t)(Distance * 100), 3, 16);
          OLED_ShowString(80, 3, (uint8_t *)"cm", 16);
      }
      
      HAL_Delay(2); 
      
      /* USER CODE END 3 */
  }
}

// 下面的配置函数等保持你原样...
void SystemClock_Config(void) { /* 保持原样，省略 */ }