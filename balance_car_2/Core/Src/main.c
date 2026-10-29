/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
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

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int16_t test_cnt; 
extern float Roll;       
extern int16_t Get_EncoderL, Get_EncoderR; 
extern int16_t Velocity_out; 
extern float Distance; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	int oled_cnt=0;
		/* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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
  /* === 面试解析 ===
   * 为什么这里要禁用外部中断�?
   * 因为接下来要初始�? MPU6050 �? DMP 固件，需要极速且纯净的软�? I2C 时序�?
   * 如果不关闭，此时超声波或串口中断打断时序，会导致 DMP 初始化直接失败�??
   */

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
      /* === 面试解析 ===
       * 死循环保护�?�如�? MPU6050 损坏或接线松动，程序死在这里，防止小车乱跑伤人�??
       */
  }
  
  HAL_Delay(1000);
  OLED_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
      // 1. 极�?�执行控制环（绝对不被干扰）
      Control_Loop(); 
      
//      // 2. 测距降频：每 50 次循环（大概 100ms）测�?次距�?
//      echo_cnt++;
//      if (echo_cnt >= 50) {
//          echo_cnt = 0;
//          echo_value_get();
//      }
      
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
      /* === 面试解析 ===
       * 这个 while(1) 架构非常优秀。采用了“时间片轮询”的思想�?
       * 控制环（Control_Loop）是�?高优先级，每次必执行�?
       * 屏幕和超声波是�?�时大户，利用计数器进行降频（分频）执行，确保不会拖慢控制环�?
       * �?后的 HAL_Delay(2) 既给�? CPU 喘息的空闲，又固定了系统的基�?节拍�?500Hz）�??
       */
      
  /* USER CODE END 3 */
		}	
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
