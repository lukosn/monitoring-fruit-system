/*
 * BME_280_Connection.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Lukas
 */

#ifndef INC_BME_280_CONNECTION_H_
#define INC_BME_280_CONNECTION_H_

#include <stdint.h>

int8_t BME_280_init(void);
void delay_ms(uint32_t period);
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t size);
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t size);
int8_t BME280_read_print_data(void);

#endif /* INC_BME_280_CONNECTION_H_ */
