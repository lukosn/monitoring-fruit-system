/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280_defs.h"
#include "bme280.h"
#include <stdio.h>
#include "lcd_i2c.h"
#include <stdbool.h>
#include "microSD.h"
#include "fatfs_sd.h"
#include "string.h"
#include "ff.h"
#include "akcelerometr.h"
#include "math.h"

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
struct lcd_disp disp;
char buffer[100];
bool power;
extern struct bme280_data data;
volatile uint16_t push_counter;
RTC_TimeTypeDef time;
RTC_DateTypeDef date;
uint8_t rx_data[6];
struct axis axis;
struct axis_g axis_g;
float a_axis, a_total;
int timer;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void power_on(void)
{
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	timer = time.Seconds % 5;

	if(timer == 1)
	{
		BME280_read_print_data();

		sprintf((char *)disp.line_1, "Temperatura: %0.2f C",data.temperature);
		sprintf((char *)disp.line_2, "Cisnienie: %0.1f hPa",data.pressure*0.01);
		sprintf((char *)disp.line_3, "Wilgotnosc: %0.2f %%",data.humidity);
		lcd_display(&disp);

		sprintf(buffer, "%02d:%02d:%02d, Temperatura: %0.2f C, Cisnienie: %0.2f hPa, Wilgotnosc: %0.2f %% \r\n",
				time.Hours, time.Minutes, time.Seconds, data.temperature, data.pressure*0.01, data.humidity);
		write_data_SD(buffer);
		HAL_Delay(1000);
	}

	acc_data();

}

void acc_data(void)
{
	acc_read(0x32);
	axis.X_axis = ((rx_data[1]<<8) | rx_data[0]);
	axis.Y_axis = ((rx_data[3]<<8) | rx_data[2]);
	axis.Z_axis = ((rx_data[5]<<8) | rx_data[4]);

	axis_g.X_g = axis.X_axis*.0078;
	axis_g.Y_g = axis.Y_axis*.0078;
	axis_g.Z_g = axis.Z_axis*.0078;
	a_axis = powf(axis_g.X_g,2) + powf(axis_g.Y_g,2) + powf(axis_g.Z_g,2);
	a_total = powf(a_axis, 0.5) - 1;

	if(a_total > 1)
	{
	sprintf(buffer, "%02d:%02d:%02d, Nastapilo uderzenie z przyspieszeniem: %0.2fg \r\n",
			time.Hours, time.Minutes, time.Seconds, a_total);
	write_data_SD(buffer);
	}
	HAL_Delay(100);
}

void power_off(void)
{
	  lcd_clear(&disp);
	  RTC_TimeTypeDef new_time = {0};
	  HAL_RTC_SetTime(&hrtc, &new_time, RTC_FORMAT_BIN);
}

void acc_write(uint8_t addr, uint8_t value)
{
	uint8_t data[2];
	data[0] = addr | 0x40;
	data[1] = value;
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, data, 2, 100);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}

void acc_read(uint8_t addr)
{
	addr |= 0x80;
	addr |= 0x40;
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Receive(&hspi2, rx_data, 6, 100);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}

void acc_init(void)
{
	acc_write(0x31, 0x01);
	acc_write(0x2d, 0x00);
	acc_write(0x2d, 0x08);
}

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  BME_280_init();
  acc_init();



  disp.addr = (0x27 << 1);
  disp.bl = true;
  lcd_init(&disp);

  HAL_Delay(500);
  SD_disk_initialize(0);
  HAL_Delay(500);

  plik_SD_init();
  HAL_Delay(200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(push_counter)
	  {
	  case 0:
		  power_off();
		  break;
	  case 1:
		  power_on();
		  break;
	  default:
		  break;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == power_button_Pin)
	{
		push_counter ++;
		if(push_counter >= 2)
		{
			push_counter = 0;
		}
	}
}
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
