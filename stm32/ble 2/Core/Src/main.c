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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 协议定义
#define CMD_HEADER     0xAA
#define CMD_LED_RED    0x01
#define CMD_LED_GREEN  0x02
#define CMD_LED_BLUE   0x03
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 接收缓冲�?
uint8_t receiveData[256];
uint8_t txData[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief UART空闲中断回调函数 - 处理�?帧完整数�?
 * @param huart UART句柄
 * @param Size 接收到的数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1) {
        // 回显接收到的数据（调试用�?
        HAL_UART_Transmit_DMA(&huart1, receiveData, Size);
        
        // �?查帧�?
        if (receiveData[0] == CMD_HEADER) {
            // �?查长度字节（�?2个字节应等于总长度）
            if (receiveData[1] == Size) {
                // 计算校验和（除最后一个字节外的所有字节之和）
                uint8_t sum = 0;
                for (int i = 0; i < Size - 1; i++) {
                    sum += receiveData[i];
                }
                
                // 校验和验�?
                if (sum == receiveData[Size - 1]) {
                    // 解析数据：从索引2�?始，每两个字节为�?�? [引脚ID, 状�?�]
                    for (int i = 2; i < Size - 1; i += 2) {
                        uint8_t pinId = receiveData[i];
                        uint8_t pinState = receiveData[i + 1];
                      
                        // 转换状�?��??
                        GPIO_PinState state = (pinState == 0x00) ? GPIO_PIN_RESET : GPIO_PIN_SET;
                        
                        // 根据引脚ID控制对应LED
                        // 如果 LED_xxx 未定义，请改为直接端口操�?
                        switch (pinId) {
                            case CMD_LED_RED:
                                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, state);
                                break;
                            case CMD_LED_GREEN:
                                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, state);
                                break;
                            case CMD_LED_BLUE:
                                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state);
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
        }
        
        // 重新启动DMA接收（等待下�?帧数据）
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
        // 禁用DMA半满中断（避免不必要的回调）
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    // 启动DMA空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receiveData, sizeof(receiveData));
    // 禁用DMA半满中断
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
        /* 可以添加 LED 闪烁等错误指�? */
    }
}
/* USER CODE END 4 */

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
