/*
 * STH21.h
 *
 *  Created on: 14/06/2021
 *      Author: jdalvarado
 */

#ifndef SHT21_H_
#define SHT21_H_

#include "sapi.h"

#define STH21_ADDR 	0x40	// I2C device address
#define I2C 		I2C0

#define  POLYNOMIAL 0x131;  // P(x)=x^8+x^5+x^4+1 = 100110001

//==============================================================================
#define TRIGGER_T_MEASUREMENT_HM 	0XE3   	// command trig. temp meas. hold master
#define TRIGGER_RH_MEASUREMENT_HM 	0XE5  	// command trig. hum. meas. hold master
#define TRIGGER_T_MEASUREMENT_NHM 	0XF3  	// command trig. temp meas. no hold master
#define TRIGGER_RH_MEASUREMENT_NHM 	0XF5 	// command trig. hum. meas. no hold master
#define USER_REGISTER_W 			0XE6	// command writing user register
#define USER_REGISTER_R 			0XE7    // command reading user register
#define SOFT_RESET 					0XFE    // command soft reset
//==============================================================================
// HOLD MASTER - SCL line is blocked (controlled by sensor) during measurement
// NO HOLD MASTER - allows other I2C communication tasks while sensor performing
// measurements.
//==============================================================================
uint16_t SHT21_readSensor_hm(uint8_t command, uint8_t addr);
//==============================================================================
// reads SHT21 with hold master operation mode
// input:	temp/hum command
// return:	temp/hum raw data (16bit scaled)
//==============================================================================
float SHT21_CalcRH(uint16_t rh);
//==============================================================================
// calculates the relative humidity
// input:  rh:	 humidity raw value (16bit scaled)
// return:		 relative humidity [%RH] (float)
//==============================================================================
float SHT21_CalcT(uint16_t t);
//==============================================================================
// calculates the temperature
// input:  t: 	temperature raw value (16bit scaled)
// return:		relative temperature [°C] (float)
//==============================================================================
uint8_t SHT21_CRC_Checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum);
//==============================================================================
// CRC-8 checksum for error detection
// input:  data[]       checksum is built based on this data
//         no_of_bytes  checksum is built for n bytes of data
//         checksum     expected checksum
// return:              1 			   = checksum does not match
//                      0              = checksum matches
//==============================================================================
float SHT21_getHumidity(uint8_t addr);
//==============================================================================
// calls humidity measurement with hold master mode
//==============================================================================
float SHT21_getTemperature(uint8_t addr);
//==============================================================================
// calls temperature measurement with hold master mode
//==============================================================================
void SHT21_init(uint8_t addr);
//==============================================================================
// performs a soft reset, delays 15ms
//==============================================================================
uint8_t SHT21_getSerialNumber(uint8_t return_sn, uint8_t addr);
//==============================================================================
// returns electronical identification code depending of selected memory
// location

#endif /* ACTIVIDADES_GLCD_INC_STH21_H_ */
