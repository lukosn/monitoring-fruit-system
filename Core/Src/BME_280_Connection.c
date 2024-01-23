/*
 * BME_280_Connection.c
 *
 *  Created on: Dec 27, 2023
 *      Author: Lukas
 */


#include "BME_280_Connection.h"
#include "bme280.h"
#include "i2c.h"
#include <stdio.h>

static struct bme280_dev bme;
struct bme280_data data;
float temp, pres, hum;


int8_t BME_280_init(void)
{
	int8_t result = BME280_OK;
	uint8_t settings;

	bme.dev_id = (BME280_I2C_ADDR_PRIM<<1);
	bme.intf = BME280_I2C_INTF;
	bme.read = i2c_read;
	bme.write = i2c_write;
	bme.delay_ms = delay_ms;

	result = bme280_init(&bme);

	if(result != BME280_OK)
	{
		return -1;
	}

	bme.settings.filter = BME280_FILTER_COEFF_16;
	bme.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;
	bme.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme.settings.osr_h = BME280_OVERSAMPLING_1X;



	if(result == BME280_OK)
	{
		settings = 	BME280_OSR_PRESS_SEL;
		settings |= BME280_OSR_TEMP_SEL;
		settings |= BME280_OSR_HUM_SEL;
		settings |= BME280_STANDBY_SEL;
		settings |= BME280_FILTER_SEL;

		result = bme280_set_sensor_settings(settings, &bme);
		result = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme);
	}

	return result;
}

int8_t BME280_read_print_data(void)
{
	int8_t result;

		bme.delay_ms(100);
		result = bme280_get_sensor_data(BME280_ALL, &data, &bme);

		temp = data.temperature;
		pres = data.pressure*0.01;
		hum = data.humidity;

		printf("Temperature: %0.2f C, Pressure:  %0.2f hPa, Humidity:  %0.2f %%\r\n",temp, pres, hum);

	return result;
}



void delay_ms(uint32_t period)
{
	HAL_Delay(period);
}

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t size)
{
	int8_t rslt = 0;
	HAL_I2C_Mem_Read(&hi2c2, dev_id, reg_addr, 1, reg_data, size, 100);
	return rslt;
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t size)
{
	int8_t rslt = 0;
	HAL_I2C_Mem_Write(&hi2c2, dev_id, reg_addr, 1, reg_data, size, 100);
	return rslt;
}




