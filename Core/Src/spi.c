/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
#include "stm32h7xx_hal.h"
#include "tim.h"
#include <stdio.h>

uint16_t ForgColor = 0x0000;
uint16_t BackColor = 0x0000;
uint16_t LCD_Buff[1024];
/* USER CODE END 0 */

SPI_HandleTypeDef hspi6;

/* SPI6 init function */
void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_02DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI6)
  {
  /* USER CODE BEGIN SPI6_MspInit 0 */

  /* USER CODE END SPI6_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI6 clock enable */
    __HAL_RCC_SPI6_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**SPI6 GPIO Configuration
    PA15 (JTDI)     ------> SPI6_NSS
    PG14     ------> SPI6_MOSI
    PG13     ------> SPI6_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_SPI6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI6_MspInit 1 */

  /* USER CODE END SPI6_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI6)
  {
  /* USER CODE BEGIN SPI6_MspDeInit 0 */

  /* USER CODE END SPI6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI6_CLK_DISABLE();

    /**SPI6 GPIO Configuration
    PA15 (JTDI)     ------> SPI6_NSS
    PG14     ------> SPI6_MOSI
    PG13     ------> SPI6_SCK
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_13);

  /* USER CODE BEGIN SPI6_MspDeInit 1 */

  /* USER CODE END SPI6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void LCD_WriteCommand(uint8_t lcd_command)
{
   LCD_DC_Command;
   HAL_SPI_Transmit(&hspi6, &lcd_command, 1, 1000);
}

void LCD_WriteData_8bit(uint8_t lcd_data)
{
   LCD_DC_Data;
   HAL_SPI_Transmit(&hspi6, &lcd_data, 1, 1000);
}

void LCD_WriteData_16bit(uint16_t lcd_data)
{
  uint8_t lcd_data_buff[2];
  LCD_DC_Data;
  lcd_data_buff[0] = lcd_data>>8;
  lcd_data_buff[1] = lcd_data;
	HAL_SPI_Transmit(&hspi6, lcd_data_buff, 2, 1000);
}

void LCD_WriteBuff(uint16_t *DataBuff, uint16_t DataSize)
{
	LCD_DC_Data;
  hspi6.Init.DataSize = SPI_DATASIZE_16BIT;
  HAL_SPI_Init(&hspi6);		
	
	HAL_SPI_Transmit(&hspi6, (uint8_t *)DataBuff, DataSize, 1000);
	
	hspi6.Init.DataSize 	= SPI_DATASIZE_8BIT;
  HAL_SPI_Init(&hspi6);	
}

void LCD_SetAddress(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)		
{
	LCD_WriteCommand(0x2a);
	LCD_WriteData_16bit(x1);
	LCD_WriteData_16bit(x2);

	LCD_WriteCommand(0x2b);
	LCD_WriteData_16bit(y1);
	LCD_WriteData_16bit(y2);

	LCD_WriteCommand(0x2c);
}

void LCD_SetColor(uint32_t Color)
{
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);
	ForgColor 	= (uint16_t)(Red_Value | Green_Value | Blue_Value);
}

void LCD_SetBackColor(uint32_t Color)
{
	uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
	Red_Value   = (uint16_t)((Color&0x00F80000)>>8);
	Green_Value = (uint16_t)((Color&0x0000FC00)>>5);
	Blue_Value  = (uint16_t)((Color&0x000000F8)>>3);
	BackColor 	= (uint16_t)(Red_Value | Green_Value | Blue_Value);  	
}

void LCD_SPI_CloseTransfer(SPI_HandleTypeDef *hspi)
{
  uint32_t itflag = hspi->Instance->SR;

  __HAL_SPI_CLEAR_EOTFLAG(hspi);
  __HAL_SPI_CLEAR_TXTFFLAG(hspi);

  /* Disable SPI peripheral */
  __HAL_SPI_DISABLE(hspi);

  /* Disable ITs */
  __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | SPI_IT_FRE | SPI_IT_MODF));

  /* Disable Tx DMA Request */
  CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

  /* Report UnderRun error for non RX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
  {
    if ((itflag & SPI_FLAG_UDR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
      __HAL_SPI_CLEAR_UDRFLAG(hspi);
    }
  }

  /* Report OverRun error for non TX Only communication */
  if (hspi->State != HAL_SPI_STATE_BUSY_TX)
  {
    if ((itflag & SPI_FLAG_OVR) != 0UL)
    {
      SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
      __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
  }

  /* SPI Mode Fault error interrupt occurred -------------------------------*/
  if ((itflag & SPI_FLAG_MODF) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
    __HAL_SPI_CLEAR_MODFFLAG(hspi);
  }

  /* SPI Frame error interrupt occurred ------------------------------------*/
  if ((itflag & SPI_FLAG_FRE) != 0UL)
  {
    SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
    __HAL_SPI_CLEAR_FREFLAG(hspi);
  }

  hspi->TxXferCount = (uint16_t)0UL;
  hspi->RxXferCount = (uint16_t)0UL;
}

HAL_StatusTypeDef LCD_SPI_WaitOnFlagUntilTimeout(SPI_HandleTypeDef *hspi, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Tickstart, uint32_t Timeout)
{
   /* Wait until flag is set */
   while ((__HAL_SPI_GET_FLAG(hspi, Flag) ? SET : RESET) == Status)
   {
      /* Check for the Timeout */
      if ((((HAL_GetTick() - Tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
      {
         return HAL_TIMEOUT;
      }
   }
   return HAL_OK;
}

HAL_StatusTypeDef LCD_SPI_Transmit(SPI_HandleTypeDef *hspi,uint16_t pData, uint32_t Size)
{
   uint32_t    tickstart;  
   uint32_t    Timeout = 1000;
   uint32_t    LCD_pData_32bit;
   uint32_t    LCD_TxDataCount;
   HAL_StatusTypeDef errorcode = HAL_OK;

	/* Check Direction parameter */
	assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE_2LINES_TXONLY(hspi->Init.Direction));

	/* Process Locked */
	__HAL_LOCK(hspi);

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	if (hspi->State != HAL_SPI_STATE_READY)
	{
		errorcode = HAL_BUSY;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	if ( Size == 0UL)
	{
		errorcode = HAL_ERROR;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	/* Set the transaction information */
	hspi->State       = HAL_SPI_STATE_BUSY_TX;
	hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

	LCD_TxDataCount   = Size;
	LCD_pData_32bit   = (pData<<16)|pData;

	/*Init field not used in handle to zero */
	hspi->pRxBuffPtr  = NULL;
	hspi->RxXferSize  = (uint16_t) 0UL;
	hspi->RxXferCount = (uint16_t) 0UL;
	hspi->TxISR       = NULL;
	hspi->RxISR       = NULL;

	/* Configure communication direction : 1Line */
	if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
	{
		SPI_1LINE_TX(hspi);
	}

	MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, 0);

	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(hspi);

	if (hspi->Init.Mode == SPI_MODE_MASTER)
	{
		 /* Master transfer start */
		 SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	}

	/* Transmit data in 16 Bit mode */
	while (LCD_TxDataCount > 0UL)
	{
		/* Wait until TXP flag is set to send data */
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP))
		{
			if ((hspi->TxXferCount > 1UL) && (hspi->Init.FifoThreshold > SPI_FIFO_THRESHOLD_01DATA))
			{
				*((__IO uint32_t *)&hspi->Instance->TXDR) = (uint32_t )LCD_pData_32bit;
				LCD_TxDataCount -= (uint16_t)2UL;
			}
			else
			{
				*((__IO uint16_t *)&hspi->Instance->TXDR) =  (uint16_t )pData;
				LCD_TxDataCount--;
			}
		}
		else
		{
			/* Timeout management */
			if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
			{
				/* Call standard close procedure with error check */
				LCD_SPI_CloseTransfer(hspi);

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
				hspi->State = HAL_SPI_STATE_READY;
				return HAL_ERROR;
			}
		}
	}

	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_SR_TXC, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}

	SET_BIT((hspi)->Instance->CR1 , SPI_CR1_CSUSP);
	/* µÈ´ýSPI¹ÒÆð */
	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_SUSP, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}
	LCD_SPI_CloseTransfer(hspi);   /* Call standard close procedure with error check */

	SET_BIT((hspi)->Instance->IFCR , SPI_IFCR_SUSPC);


	/* Process Unlocked */
	__HAL_UNLOCK(hspi);

	hspi->State = HAL_SPI_STATE_READY;

	if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
	{
		return HAL_ERROR;
	}
	return errorcode;
}

HAL_StatusTypeDef LCD_SPI_TransmitBuffer (SPI_HandleTypeDef *hspi, uint16_t *pData, uint32_t Size)
{
   uint32_t    tickstart;  
   uint32_t    Timeout = 1000;
   uint32_t    LCD_TxDataCount;
   HAL_StatusTypeDef errorcode = HAL_OK;

	/* Check Direction parameter */
	assert_param(IS_SPI_DIRECTION_2LINES_OR_1LINE_2LINES_TXONLY(hspi->Init.Direction));

	/* Process Locked */
	__HAL_LOCK(hspi);

	/* Init tickstart for timeout management*/
	tickstart = HAL_GetTick();

	if (hspi->State != HAL_SPI_STATE_READY)
	{
		errorcode = HAL_BUSY;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	if ( Size == 0UL)
	{
		errorcode = HAL_ERROR;
		__HAL_UNLOCK(hspi);
		return errorcode;
	}

	/* Set the transaction information */
	hspi->State       = HAL_SPI_STATE_BUSY_TX;
	hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

	LCD_TxDataCount   = Size;                // ´«ÊäµÄÊý¾Ý³¤¶È

	/*Init field not used in handle to zero */
	hspi->pRxBuffPtr  = NULL;
	hspi->RxXferSize  = (uint16_t) 0UL;
	hspi->RxXferCount = (uint16_t) 0UL;
	hspi->TxISR       = NULL;
	hspi->RxISR       = NULL;

	/* Configure communication direction : 1Line */
	if (hspi->Init.Direction == SPI_DIRECTION_1LINE)
	{
		SPI_1LINE_TX(hspi);
	}

	MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, 0);

	/* Enable SPI peripheral */
	__HAL_SPI_ENABLE(hspi);

	if (hspi->Init.Mode == SPI_MODE_MASTER)
	{
		 /* Master transfer start */
		 SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
	}

	/* Transmit data in 16 Bit mode */
	while (LCD_TxDataCount > 0UL)
	{
		/* Wait until TXP flag is set to send data */
		if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP))
		{
			if ((LCD_TxDataCount > 1UL) && (hspi->Init.FifoThreshold > SPI_FIFO_THRESHOLD_01DATA))
			{
				*((__IO uint32_t *)&hspi->Instance->TXDR) = *((uint32_t *)pData);
				pData += 2;
				LCD_TxDataCount -= 2;
			}
			else
			{
				*((__IO uint16_t *)&hspi->Instance->TXDR) = *((uint16_t *)pData);
				pData += 1;
				LCD_TxDataCount--;
			}
		}
		else
		{
			/* Timeout management */
			if ((((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY)) || (Timeout == 0U))
			{
				/* Call standard close procedure with error check */
				LCD_SPI_CloseTransfer(hspi);

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_TIMEOUT);
				hspi->State = HAL_SPI_STATE_READY;
				return HAL_ERROR;
			}
		}
	}

	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_SR_TXC, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}

	SET_BIT((hspi)->Instance->CR1 , SPI_CR1_CSUSP);
	if (LCD_SPI_WaitOnFlagUntilTimeout(hspi, SPI_FLAG_SUSP, RESET, tickstart, Timeout) != HAL_OK)
	{
		SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FLAG);
	}
	LCD_SPI_CloseTransfer(hspi);   /* Call standard close procedure with error check */

	SET_BIT((hspi)->Instance->IFCR , SPI_IFCR_SUSPC);


	/* Process Unlocked */
	__HAL_UNLOCK(hspi);

	hspi->State = HAL_SPI_STATE_READY;

	if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
	{
		return HAL_ERROR;
	}
	return errorcode;
}

void SPI_LCD_Init(void)
{   
  	TIM17_Delay_Ms(10);
 	LCD_WriteCommand(0x36);
	LCD_WriteData_8bit(0x00);

	LCD_WriteCommand(0x3A);
	LCD_WriteData_8bit(0x05);

 	LCD_WriteCommand(0xB2);			
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x0C); 
	LCD_WriteData_8bit(0x00); 
	LCD_WriteData_8bit(0x33); 
	LCD_WriteData_8bit(0x33); 			

	LCD_WriteCommand(0xB7);
	LCD_WriteData_8bit(0x35);

	LCD_WriteCommand(0xBB);
	LCD_WriteData_8bit(0x19);

	LCD_WriteCommand(0xC0);
	LCD_WriteData_8bit(0x2C);

	LCD_WriteCommand(0xC2);
	LCD_WriteData_8bit(0x01);

	LCD_WriteCommand(0xC3);
	LCD_WriteData_8bit(0x12);
				
	LCD_WriteCommand(0xC4);
	LCD_WriteData_8bit(0x20);

	LCD_WriteCommand(0xC6);
	LCD_WriteData_8bit(0x0F);

	LCD_WriteCommand(0xD0);
	LCD_WriteData_8bit(0xA4);
	LCD_WriteData_8bit(0xA1);

	LCD_WriteCommand(0xE0);
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2B);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x54);
	LCD_WriteData_8bit(0x4C);
	LCD_WriteData_8bit(0x18);
	LCD_WriteData_8bit(0x0D);
	LCD_WriteData_8bit(0x0B);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0xE1);
	LCD_WriteData_8bit(0xD0);
	LCD_WriteData_8bit(0x04);
	LCD_WriteData_8bit(0x0C);
	LCD_WriteData_8bit(0x11);
	LCD_WriteData_8bit(0x13);
	LCD_WriteData_8bit(0x2C);
	LCD_WriteData_8bit(0x3F);
	LCD_WriteData_8bit(0x44);
	LCD_WriteData_8bit(0x51);
	LCD_WriteData_8bit(0x2F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x1F);
	LCD_WriteData_8bit(0x20);
	LCD_WriteData_8bit(0x23);

	LCD_WriteCommand(0x21);

	LCD_WriteCommand(0x11);
  	TIM17_Delay_Ms(120);

	LCD_WriteCommand(0x29);

  	LCD_WriteCommand(0x36);
  	LCD_WriteData_8bit(0x70);
}

void LCD_Clear(void)
{
	LCD_SetAddress(0,0,320-1,240-1);
	
	LCD_DC_Data;	
	hspi6.Init.DataSize = SPI_DATASIZE_16BIT;
	HAL_SPI_Init(&hspi6);		
	
	LCD_SPI_Transmit(&hspi6, BackColor, 320 * 240);

	hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
	HAL_SPI_Init(&hspi6);
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
  	LCD_SetAddress( x, y, x+width-1, y+height-1);
	LCD_DC_Data;
  	hspi6.Init.DataSize = SPI_DATASIZE_16BIT;
  	HAL_SPI_Init(&hspi6);
  	LCD_SPI_Transmit(&hspi6, ForgColor, width*height);
	// printf("%x\n",ForgColor);

	hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  	HAL_SPI_Init(&hspi6);
}

void LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)
{  
   uint8_t   disChar;
   uint16_t  Xaddress = x;
   uint16_t  Yaddress = y;
   uint16_t  i=0,j=0,m=0;
   uint16_t  BuffCount = 0;
   uint16_t  Buff_Height = 0;

   Buff_Height = (sizeof(LCD_Buff)/2) / height;

	for(i = 0; i <height; i++)
	{
		for(j = 0; j <(float)width/8; j++)  
		{
			disChar = *pImage;

			for(m = 0; m < 8; m++)
			{ 
				if(disChar & 0x01)	
				{		
            		LCD_Buff[BuffCount] =  ForgColor;
				}
				else		
				{		
					LCD_Buff[BuffCount] = BackColor;
				}
				disChar >>= 1;
				Xaddress++;
				BuffCount++;  
				if( (Xaddress - x)==width)
				{											 
					Xaddress = x;				                 
					break;
				}
			}	
			pImage++;			
		}
      if( BuffCount == Buff_Height*width)
      {
         BuffCount = 0;
         LCD_SetAddress( x, Yaddress , x+width-1, Yaddress+Buff_Height-1);
         LCD_WriteBuff(LCD_Buff,width*Buff_Height);

         Yaddress = Yaddress+Buff_Height;
      }     
      if( (i+1)== height )
      {
         LCD_SetAddress( x, Yaddress , x+width-1,i+y);
         LCD_WriteBuff(LCD_Buff,width*(i+1+y-Yaddress)); 
      }
	}	
}

void LCD_CopyBuffer(uint16_t x, uint16_t y,uint16_t width,uint16_t height,uint16_t *DataBuff)
{
	LCD_SetAddress(x,y,x+width-1,y+height-1);

	LCD_DC_Data;
    hspi6.Init.DataSize = SPI_DATASIZE_16BIT;
    HAL_SPI_Init(&hspi6);		
	
	LCD_SPI_TransmitBuffer(&hspi6, DataBuff,width * height) ;
	
//	HAL_SPI_Transmit(&hspi5, (uint8_t *)DataBuff, (x2-x1+1) * (y2-y1+1), 1000) ;
    hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&hspi6);		
}
/* USER CODE END 1 */
