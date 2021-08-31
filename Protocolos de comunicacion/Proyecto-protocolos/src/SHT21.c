/*
 * STH21.c
 *
 *  Created on: 14/06/2021
 *      Author: jdalvarado
 */

#include "../../Proyecto_protocolos/inc/SHT21.h"

//==============================================================================
// PUBLIC
//==============================================================================

float SHT21_getHumidity(uint8_t addr) {
	uint16_t result; 	// return variable

	result = SHT21_readSensor_hm(TRIGGER_RH_MEASUREMENT_NHM, addr);

	return SHT21_CalcRH(result);
}


float SHT21_getTemperature(uint8_t addr) {
	uint16_t result; 	// return variable

	result = SHT21_readSensor_hm(TRIGGER_T_MEASUREMENT_NHM, addr);

	return SHT21_CalcT(result);
}

void SHT21_init(uint8_t addr)
{
	char reg[1] ;

	reg[0] = SOFT_RESET;


	i2cWrite ( I2C , addr,  reg, 1, ON);

	delay(15);	// wait for SHT to reset
}

uint8_t SHT21_getSerialNumber(uint8_t return_sn, uint8_t addr)
{

	uint8_t serialNumber[8];

	char reg[1] ;
	char data[8];

	reg[0] = 0xFA;
	reg[1] = 0x0F;

	// read memory location 1
	i2cWrite ( I2C , addr,  reg, 2, ON);

	i2cRead  ( I2C , addr, data, 8, ON);

	serialNumber[5] = data[0];	// read SNB_3
	serialNumber[4] = data[2];  // read SNB_2
	serialNumber[3] = data[4];	// read SNB_1
	serialNumber[2] = data[6];	// read SNB_0

	// read memory location 2

	reg[0] = 0xFC;
	reg[1] = 0xC9;

	i2cWrite ( I2C , addr,  reg, 2, ON);

	i2cRead  ( I2C , addr, data, 6, ON);

	serialNumber[1] = data[0];	// read SNC_1
	serialNumber[0] = data[1];  // read SNC_0
	serialNumber[7] = data[3];	// read SNA_1
	serialNumber[6] = data[4];	// read SNA_0

	return serialNumber[return_sn];
}

//==============================================================================
// PRIVATE
//==============================================================================

uint16_t SHT21_readSensor_hm(uint8_t command, uint8_t addr) {
	uint8_t checksum;
	uint8_t data[2];
	uint16_t result;
	uint8_t n = 0;
	uint8_t d;

	if(command == TRIGGER_RH_MEASUREMENT_HM || command == TRIGGER_RH_MEASUREMENT_NHM) d = 30;
	if(command == TRIGGER_T_MEASUREMENT_HM || command == TRIGGER_T_MEASUREMENT_NHM) d = 85;

	i2cWrite ( I2C , addr, &command, 1, ON);

	delay(d);

	i2cRead  ( I2C , addr, data, 3, ON);

	result = (data[0] << 8);
	result += data[1];
	checksum = data[2];

	return result;
}

float SHT21_CalcRH(uint16_t rh)
{

	rh &= ~0x0003;	// clean last two bits

  	return (-6.0 + 125.0/65536 * (float)rh); // return relative humidity
}

float SHT21_CalcT(uint16_t t) {

	t &= ~0x0003;	// clean last two bits

	return (-46.85 + 175.72/65536 * (float)t);
}

uint8_t SHT21_CRC_Checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum) {
	uint8_t crc = 0;
  	uint8_t byteCtr;

 	 //calculates 8-Bit checksum with given polynomial

  	for (byteCtr = 0; byteCtr < no_of_bytes; ++byteCtr)
 	{
  		crc ^= (data[byteCtr]);

  		for (uint8_t bit = 8; bit > 0; --bit)
  		{
 		   if (crc & 0x80)
 		   {
 			   crc = (crc << 1) ^ POLYNOMIAL;
 		   }
 		   else
 		   {
 			   crc = (crc << 1);
 		   }
 	   }
 	 }

  	if (crc != checksum) return 1;
 	 else return 0;
}
