/*
 * akcelerometr.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Lukas
 */

#include "akcelerometr.h"
#include "spi.h"
#include <stdio.h>
#include "stm32f3xx_hal.h"





/*
void acc_write(uint8_t addr, uint8_t value)
{
	uint8_t data[2];
	data[0] = addr | 0x40;
	data[1] = value;
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, data, 2, 100);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}


void acc_read(uint8_t addr, uint8_t *rec_data)
{
	addr = 0x40 | 0x80;
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Receive(&hspi2, rec_data, 6, 100);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}


void acc_init(void)
{
	acc_write(0x31, 0x01);
	acc_write(0x2d, 0x00);
	acc_write(0x2d, 0x08);
}

void acc_data(void)
{
	uint8_t data[6];
	axis.X_axis = ((data[1]<<8) | data[0]);
	axis.Y_axis = ((data[3]<<8) | data[2]);
	axis.Z_axis = ((data[5]<<8) | data[4]);

	axis_g.X_g = axis.X_axis*.0078;
	axis_g.Y_g = axis.Y_axis*.0078;
	axis_g.Z_g = axis.Z_axis*.0078;
	printf("X_G: %0.2f, Y_G: %0.2f, Z_G: %0.2f \r\n", axis_g.X_g, axis_g.Y_g, axis_g.Z_g);
}

*/

void dev_ID(uint8_t addr, uint8_t *ID)
{
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
	HAL_SPI_Receive(&hspi2, ID, 1, 100);
	HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}
