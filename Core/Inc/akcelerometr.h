/*
 * akcelerometr.h
 *
 *  Created on: Nov 8, 2023
 *      Author: Lukas
 */
#include <stdio.h>
#include <stdint.h>

#ifndef INC_AKCELEROMETR_H_
#define INC_AKCELEROMETR_H_

/*
void acc_write(uint8_t addr, uint8_t value);
void acc_read(uint8_t addr, uint8_t *rec_data);
void acc_init(void);
void acc_data(void);
*/
void dev_ID(uint8_t addr, uint8_t *ID);


struct axis
{
	int16_t X_axis;
	int16_t Y_axis;
	int16_t Z_axis;
};

struct axis_g
{
	float X_g;
	float Y_g;
	float Z_g;
};



#endif /* INC_AKCELEROMETR_H_ */
