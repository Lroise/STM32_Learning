/* USER CODE BEGIN Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "app_button.h"       
#include "app_pwm.h"          
#include "app_encoder.h"      
#include "app_motor.h"        
#include "app_mpu6050.h"      
#include "app_control.h"      
#include "app_oled.h"         
#include "task.h"
#include <stdio.h>            
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
extern uint8_t motor_run_flag; 
float targetOmega = 0.0f;      
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  
  /* USER CODE BEGIN 2 */
  App_PWM_Init();
  App_Encoder_Init();
  App_MPU6050_Init();
  App_Motor_Init();
  App_Button_Init();
  App_Control_Init();
  
  App_OLED_Init();
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
      
      // =====================================================================
      // 【防炸核心：严格锁定 5ms 控制周期】
      // 绝对不允许空转狂算 PID，必须和 MPU6050 的 5ms 采样同步！
      // =====================================================================
      static uint32_t last_ctrl = 0;
      if (HAL_GetTick() - last_ctrl >= 5) 
      {
          last_ctrl = HAL_GetTick();
          
          App_MPU6050_Proc(); 
          App_Button_Proc();
          App_Control_Proc();
      }
      
      // =====================================================================
      // 屏幕刷新：每 100ms 刷新一次，避免频繁刷屏导致 CPU 卡顿
      // =====================================================================
      static uint32_t last_oled = 0;
      if (HAL_GetTick() - last_oled >= 100) 
      {
          last_oled = HAL_GetTick();
          char buf[16];
          
          sprintf(buf, "P: %+.1f   ", App_MPU6050_GetPitch());
          App_OLED_ShowString(0, 0, buf); 
          
          sprintf(buf, "Gx: %+.1f   ", App_MPU6050_GetGx());
          App_OLED_ShowString(0, 2, buf); 
          
          sprintf(buf, "Run: %d    ", motor_run_flag);
          App_OLED_ShowString(0, 4, buf); 
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}