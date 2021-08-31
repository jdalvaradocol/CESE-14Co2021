/*
 * isl29023.h
 *
 *  Created on: 13/06/2021
 *      Author: jdalvarado
 */

#ifndef ISL29023_H_
#define ISL29023_H_

#include "sapi.h"

#define ISL29023_ADDR 	0x44
#define I2C 			I2C0

#define CMD1 			0
#define CMD2 			1
#define DATALSB 		2
#define DATAMSB 		3

void ISL29023_init(uint8_t addr);
float ISL29023_read(uint8_t addr);

#endif /* ACTIVIDADES_GLCD_INC_ISL29023_H_ */
