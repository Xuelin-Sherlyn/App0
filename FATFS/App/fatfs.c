/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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
#include "fatfs.h"
#include <stdio.h>

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  FRESULT fre=f_mount(&SDFatFS,SDPath,1);

	switch (fre)
	{
	case FR_OK:
		printf("\033[32msd mount ok\n\r\033[0m");
		break;
	case FR_DISK_ERR:
		printf("\033[31msd mount fail: FR_DISK_ERR\033[0m\r\n");
		break;
	case FR_INT_ERR:
		printf("\033[31msd mount fail: FR_INT_ERR\033[0m\r\n");
		break;
	case FR_NOT_READY:
		printf("\033[31msd mount fail: FR_NOT_READY\033[0m\r\n");
		break;
	case FR_NO_FILE:
		printf("\033[31msd mount fail: FR_NO_FILE\033[0m\r\n");
		break;
	case FR_NO_PATH:
		printf("\033[31msd mount fail: FR_NO_PATH\033[0m\r\n");
		break;
	case FR_INVALID_NAME:
		printf("\033[31msd mount fail: FR_INVALID_NAME\033[0m\r\n");
		break;
	case FR_DENIED:
		printf("\033[31msd mount fail: FR_DENIED\033[0m\r\n");
		break;
	case FR_EXIST:
		printf("\033[31msd mount fail: FR_EXIST\033[0m\r\n");
		break;
	case FR_INVALID_OBJECT:
		printf("\033[31msd mount fail: FR_INVALID_OBJECT\033[0m\r\n");
		break;
	case FR_WRITE_PROTECTED:
		printf("\033[31msd mount fail: FR_WRITE_PROTECTED\033[0m\r\n");
		break;
	case FR_INVALID_DRIVE:
		printf("\033[31msd mount fail: FR_INVALID_DRIVE\033[0m\r\n");
		break;
	case FR_NOT_ENABLED:
		printf("\033[31msd mount fail: FR_NOT_ENABLED\033[0m\r\n");
		break;
	case FR_NO_FILESYSTEM:
		printf("\033[31msd mount fail: FR_NO_FILESYSTEM\r\n人话：文件系统错误或者掉盘了\033[0m\r\n");
		break;
	case FR_MKFS_ABORTED:
		printf("\033[31msd mount fail: FR_MKFS_ABORTED\033[0m\r\n");
		break;
	case FR_TIMEOUT:
		printf("\033[31msd mount fail: FR_TIMEOUT\033[0m\r\n");
		break;
	case FR_LOCKED:
		printf("\033[31msd mount fail: FR_LOCKED\033[0m\r\n");
		break;
	case FR_NOT_ENOUGH_CORE:
		printf("\033[31msd mount fail: FR_NOT_ENOUGH_CORE\033[0m\r\n");
		break;
	case FR_TOO_MANY_OPEN_FILES:
		printf("\033[31msd mount fail: FR_TOO_MANY_OPEN_FILES\033[0m\r\n");
		break;
	case FR_INVALID_PARAMETER:
		printf("\033[31msd mount fail: FR_INVALID_PARAMETER\033[0m\r\n");
		break;
  	}
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
