/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/_intsup.h>
#include "AkieImage.h"
#include "ssd1306.hpp"
#include "st7789.hpp"
#include "compImage.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ReciveSize 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SSD1306 i2cScreen(&hi2c1);
ST7789 spiScreen(&hspi6);

volatile uint8_t val = 0;
__attribute__((section(".ram_d1"))) 
volatile char mSerialReciveBuffer[ReciveSize] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void mProcess_SerialReceiveData(char* data, uint16_t Size);
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

  /* USER CODE BEGIN 1 */
  /*重要！！！恢复中断可用*/
  __enable_irq();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_TIM17_Init();
  MX_SPI6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  memset((uint8_t*)mSerialReciveBuffer, 0xff, ReciveSize);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, ReciveSize);
  printf("\033[35mThis is a run in QSPI Flash`s Application, Execute method is XIP\n\033[31mCall \"resetMem\"to clean usart1 recive memory, \"Exit\" to exit Application\033[0m\n");

  i2cScreen.Init();
  spiScreen.Init();
  LCD_Backlight_ON;
  spiScreen.SetColor(0xFF2070CF);
  spiScreen.SetBackColor(0xFF000000);
  spiScreen.Clear();
  spiScreen.FillRect(10, 10, 100, 100);
  spiScreen.SetColor(0xFFFF0000);
  spiScreen.FillRect(50, 50, 100, 100);
  spiScreen.SetColor(0xFF00FF00);
  spiScreen.FillRect(90, 90, 100, 100);
  spiScreen.SetColor(0xFF0000FF);
  spiScreen.FillRect(130, 130, 100, 100);
  spiScreen.SetColor(0xFF00FFFF);
  spiScreen.CopyBuffer(10, 10, 128, 128, (uint16_t*)gImage_Akie000);
  spiScreen.CopyBuffer(148, 10, 128, 128, (uint16_t*)gImage_Akie001);
  i2cScreen.ClearBuffer();
  i2cScreen.DrawRect(20, 20, 32, 32, 1);
  i2cScreen.FillRect(64, 30, 32, 32, 1);
  i2cScreen.UpdateScreen();
  TIM17_Delay_Ms(2000);
  i2cScreen.ClearBuffer();
  i2cScreen.UpdateScreen();
  TIM17_Delay_Ms(100);
  i2cScreen.DrawString(0, 0, "Akie~", 1);
  i2cScreen.DrawString(0, 16, "Happy Birthday", 1);
  i2cScreen.DrawNumber(0, 32, 20160126, 1);
  i2cScreen.DrawFloat(0, 48, 0.767f, 5, 3, 1);
  i2cScreen.UpdateScreen();
  spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_CompImage);
  TIM17_Delay_Ms(2000);
  spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie004);
  TIM17_Delay_Ms(2000);
  spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie005);
  TIM17_Delay_Ms(2000);
  spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie008);
  spiScreen.DrawString(0, 0, "gImage_Akie006");
  spiScreen.DrawNumber(0, 16, 123);
  spiScreen.DrawFloat(0, 32, 10.345, 8, 4);
  // HAL_UART_Receive_IT(&huart1, (uint8_t*)mSerialReciveBuffer, ReciveSize);
  // HAL_UART_Receive_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, ReciveSize);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // for循环大概一个时钟周期，但该用TIM还得用TIM，HAL_Delay这玩意阻塞的
    // for (volatile int i = 0; i < 48000000; i++);
    // TIM17_Delay_Ms(1000);
    if(val == 1)
    break;
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart->Instance == USART1)
//   {
//     if(strncmp(mSerialReciveBuffer, "gImage_Akie002", ReciveSize) == 0)
//     {
//       CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie002);
//     }
//     else if (strncmp(mSerialReciveBuffer, "gImage_Akie003", ReciveSize) == 0) 
//     {
//       CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie003);
//     }
//     else {
//       HAL_UART_Transmit_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, ReciveSize);
//     }
//     // memset(mSerialReciveBuffer, 0, ReciveSize);
//     HAL_UART_Receive_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, ReciveSize);
//   }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"\033[33mInterrupt Call\033[0m\n", 25);
  if(huart->Instance == USART1)
  {
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, Size);
    mProcess_SerialReceiveData((char*)mSerialReciveBuffer, Size);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, ReciveSize);
  }
}

//经过测试，指令只有在非优化或优化等级1才能用，优化等级过高会将这玩意优化掉
void mProcess_SerialReceiveData(char* data, uint16_t Size)
{
  if(strncmp(data, "gImage_Akie004", Size) == 0)
    {
      spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie004);
    }
  else if (strncmp(data, "gImage_Akie005", Size) == 0)
    {
      spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie005);
    }
  else if(strncmp(data, "gImage_Akie006", Size) == 0)
    {
      spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie006);
    }
  else if (strncmp(data, "gImage_Akie007", Size) == 0)
    {
      spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie007);
    }
  else if (strncmp(data, "gImage_Akie008", Size) == 0)
    {
      spiScreen.CopyBuffer(0, 0, 320, 240, (uint16_t*)gImage_Akie008);
    }
  else if (strncmp(data, "resetMem", Size) == 0) {
      memset((uint8_t*)mSerialReciveBuffer, 255, ReciveSize);
    }
  else if (strncmp(data, "Exit", Size) == 0) {
      HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"Application Exit\n", 18);
      LCD_Backlight_OFF;
      HAL_Delay(1);
      HAL_UART_MspDeInit(&huart1);
      HAL_SPI_MspDeInit(&hspi6);
      SCB_CleanDCache();
      SCB_InvalidateICache();
      __HAL_RCC_USART1_FORCE_RESET();
      __HAL_RCC_DMA1_FORCE_RESET();
      HAL_Delay(1);
      __HAL_RCC_USART1_RELEASE_RESET();
      __HAL_RCC_DMA1_RELEASE_RESET();
      __disable_irq();
      val = 1;
    }
  else {
      HAL_UART_Transmit_DMA(&huart1, (uint8_t*)mSerialReciveBuffer, Size);
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
