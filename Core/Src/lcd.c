/*
 * lcd.c
 *
 *  Created on: Oct 16, 2023
 *      Author: Lukas
 */


#include "lcd_i2c.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"

void lcd_init(struct lcd_disp *lcd)
{
	uint8_t xpin = 0;

	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	HAL_Delay(40);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(5);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(1);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);

	lcd_write(lcd->addr, INIT_4_BIT_MODE, xpin);

	lcd_write(lcd->addr, UNDERLINE_OFF_BLINK_OFF, xpin);

	lcd_clear(lcd);
}

void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t tx_data[4];

	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	HAL_I2C_Master_Transmit(&HI2C_DEF, addr, tx_data, 4, 100);
	HAL_Delay(5);
}

void lcd_display(struct lcd_disp * lcd)
{
	uint8_t xpin = 0, i = 0;

	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcd_clear(lcd);

	lcd_write(lcd->addr, FIRST_CHAR_LINE_1, xpin);

	while(lcd->line_1[i])
	{
		lcd_write(lcd->addr, lcd->line_1[i], (xpin | RS_PIN));
		i++;
	}

	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_2, xpin);

	while(lcd->line_2[i])
	{
		lcd_write(lcd->addr, lcd->line_2[i], (xpin | RS_PIN));
		i++;
	}


	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_3, xpin);

	while(lcd->line_3[i])
	{
		lcd_write(lcd->addr, lcd->line_3[i], (xpin | RS_PIN));
		i++;
	}

	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_4, xpin);

	while(lcd->line_4[i])
	{
		lcd_write(lcd->addr, lcd->line_4[i], (xpin | RS_PIN));
		i++;
	}
}

void lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	lcd_write(lcd->addr, CLEAR_LCD, xpin);
}
