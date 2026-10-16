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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_button.h"
#include "app_pwm.h"
#include "app_encoder.h"
#include "encoder_test.h"
#include "mpu6050_test.h"     // MPU6050 测试
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TB6612 电机方向引脚，与手写接线保持一致 */
#define AIN1_PIN  GPIO_PIN_5
#define AIN1_PORT GPIOA
#define AIN2_PIN  GPIO_PIN_4
#define AIN2_PORT GPIOA
#define BIN1_PIN  GPIO_PIN_15
#define BIN1_PORT GPIOB
#define BIN2_PIN  GPIO_PIN_14
#define BIN2_PORT GPIOB

/* 编码器一圈总脉冲数 = 线数 × 减速比 × 4（四倍频）
 * ENCODER_PPR：编码器线数
 * GEAR_RATIO ：减速比
 */
#define ENCODER_PPR   11
#define GEAR_RATIO    30
#define PULSE_PER_REV (ENCODER_PPR * GEAR_RATIO * 4)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void My_USART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  简易串口打印函数，支持可变参数
  * @param  huart: 串口句柄
  * @param  fmt:   格式化字符串
  * @note   把格式化后的字符串发到串口，供 VOFA+ 显示
  */
void My_USART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
  uint8_t buf[128];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf((char*)buf, sizeof(buf), fmt, ap);   // 安全版，限制长度
  va_end(ap);
  if(len > 0)
  {
    HAL_UART_Transmit(huart, buf, (uint16_t)len, 100);      // 超时改成 100ms
  }
}

/**
  * @brief  TIM4 定时中断回调，10ms 触发一次
  * @param  htim: 定时器句柄
  * @note   只翻转 LED，指示中断在跑
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4)
  {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);      // 翻转 LED，指示中断运行
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  App_Button_Init();                            // 按键初始化
  App_PWM_Init();                               // PWM 初始化（电机驱动）
  TB6612_Enable(1);                             // 使能 TB6612 驱动（电机驱动）
  HAL_TIM_Base_Start_IT(&htim4);                // 启动 TIM4 10ms 中断

  // Encoder_M_Method_Test();                   // 测速 + 按键 + 电机 + MPU6050（先注释掉）
  MPU6050_EularAngleTest();                     // 按键 + 电机 + MPU6050
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */