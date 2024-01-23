/*
 * microSD.c
 *
 *  Created on: Dec 10, 2023
 *      Author: Lukas
 */

#include "spi.h"
#include "stm32f3xx_hal.h"
#include "microSD.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include "string.h"
#include <stdio.h>
#include "ff.h"

FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fresult;
char buff[100];



extern void plik_SD_init(void)
{

	fresult = f_mount(&fs, "/", 1);
	if(fresult != FR_OK)
	{
	  	printf("Error in mounting SD CARD\r\n");
	  	Error_Handler();
	}


  	fresult = f_open(&fil, "BME_280/Data.txt", FA_OPEN_ALWAYS | FA_WRITE);
  	if(fresult != FR_OK)
  	{
  		printf("Error in creating a file\r\n");
  		Error_Handler();
  	}

	fresult = f_close(&fil);
	if(fresult != FR_OK)
	{
		printf("Error in closing a file\r\n");
		Error_Handler();
	}
}

extern void write_data_SD(const char *buff)
{
	UINT bytes_written;

  	fresult = f_open(&fil, "BME_280/Data.txt", FA_OPEN_ALWAYS | FA_WRITE);
  	if(fresult != FR_OK)
  	{
  		printf("Error in opening a file\r\n");
  		Error_Handler();
  	}

  	fresult = f_lseek(&fil, f_size(&fil));
  	if(fresult != FR_OK)
  	{
  		printf("Error in moving cursor to an end of the file\r\n");
  		Error_Handler();
  	}

	fresult = f_write(&fil, buff, strlen(buff), &bytes_written);
	if(fresult != FR_OK)
	{
		printf("Error in writing data to file\r\n");
		Error_Handler();
	}

	fresult = f_close(&fil);
	if(fresult != FR_OK)
	{
		printf("Error in closing a file\r\n");
		Error_Handler();
	}
	HAL_Delay(100);
}








