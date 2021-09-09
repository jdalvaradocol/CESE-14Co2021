/*
 * BMP180.h
 *
 *  Created on: 16/06/2021
 *      Author: jdalvarado
 */

#ifndef BMP180_H_
#define BMP180_H_

#include "sapi.h"

#define BMP180_ADDR 	0x77
#define I2C 			    0


/* resolutions */
typedef enum
{
	BMP180_ULTRALOWPOWER, 		// low power             mode, oss0
	BMP180_STANDARD, 			// standard              mode, oss1
	BMP180_HIGHRES,       		// high resolution       mode, oss2
	BMP180_ULTRAHIGHRES   		// ultra high resolution mode, oss3
}BMP180_RESOLUTION;

/* calibration registers */
typedef enum
{
  BMP180_CAL_AC1_REG = 0xAA,  //ac1 pressure    computation
  BMP180_CAL_AC2_REG = 0xAC,  //ac2 pressure    computation
  BMP180_CAL_AC3_REG = 0xAE,  //ac3 pressure    computation
  BMP180_CAL_AC4_REG = 0xB0,  //ac4 pressure    computation
  BMP180_CAL_AC5_REG = 0xB2,  //ac5 temperature computation
  BMP180_CAL_AC6_REG = 0xB4,  //ac6 temperature computation
  BMP180_CAL_B1_REG  = 0xB6,  //b1  pressure    computation
  BMP180_CAL_B2_REG  = 0xB8,  //b2  pressure    computation
  BMP180_CAL_MB_REG  = 0xBA,  //mb
  BMP180_CAL_MC_REG  = 0xBC,  //mc  temperature computation
  BMP180_CAL_MD_REG  = 0xBE   //md  temperature computation
}BMP180_CAL_REG;

typedef struct
{
  int16_t  bmpAC1;
  int16_t  bmpAC2;
  int16_t  bmpAC3;
  uint16_t bmpAC4;
  uint16_t bmpAC5;
  uint16_t bmpAC6;

  int16_t  bmpB1;
  int16_t  bmpB2;

  int16_t  bmpMB;
  int16_t  bmpMC;
  int16_t  bmpMD;
}BMP180_CAL_COEFF;

#define BMP180_GET_ID_REG             0x00   //device id register
#define BMP180_GET_VERSION_REG        0xD1   //device version register

#define BMP180_SOFT_RESET_REG         0xE0   //soft reset register
#define BMP180_SOFT_RESET_CTRL        0xB6   //soft reset control

#define BMP180_START_MEASURMENT_REG   0xF4   //start measurment  register
#define BMP180_READ_ADC_MSB_REG       0xF6   //read adc msb  register
#define BMP180_READ_ADC_LSB_REG       0xF7   //read adc lsb  register
#define BMP180_READ_ADC_XLSB_REG      0xF8   //read adc xlsb register

/* BMP180_START_MEASURMENT_REG controls */
#define BMP180_GET_TEMPERATURE_CTRL   0x2E   //get temperature control
#define BMP180_GET_PRESSURE_OSS0_CTRL 0x34   //get pressure oversampling 1 time/oss0 control
#define BMP180_GET_PRESSURE_OSS1_CTRL 0x74   //get pressure oversampling 2 time/oss1 control
#define BMP180_GET_PRESSURE_OSS2_CTRL 0xB4   //get pressure oversampling 4 time/oss2 control
#define BMP180_GET_PRESSURE_OSS3_CTRL 0xF4   //get pressure oversampling 8 time/oss3 control

/* misc */
#define BMP180_CHIP_ID                0x00   //id number

#define BMP180_ERROR                  255    //returns 255, if communication error is occurred

/* to store calibration coefficients */

bool     BMP180_init						(uint8_t addr);
int32_t  BMP180_getPressure					(uint8_t addr);                               	//in Pa
float    BMP180_getTemperature				(uint8_t addr);                            		//in °C
int32_t  BMP180_getSeaLevelPressure			(uint8_t addr, int16_t trueAltitude); 			//in Pa, by default true altitude id 115 meters
void     BMP180_softReset					(uint8_t addr);
uint8_t  BMP180_readFirmwareVersion			(uint8_t addr);
uint8_t  BMP180_readDeviceID				(uint8_t addr);
bool     BMP180_readCalibrationCoefficients	(uint8_t addr);
uint16_t BMP180_readRawTemperature			(uint8_t addr);
uint32_t BMP180_readRawPressure				(uint8_t addr);
int32_t  BMP180_computeB5					(int32_t UT);
int32_t BMP180_altitude					(uint8_t addr);

uint8_t  BMP180_read8						(uint8_t addr, uint8_t reg);
uint16_t BMP180_read16						(uint8_t addr, uint8_t reg);
uint16_t BMP180_read16_data					(uint8_t addr, uint8_t reg);
bool     BMP180_write8						(uint8_t addr, uint8_t reg, uint8_t control);

#endif /* ACTIVIDADES_PROYECTO_PROTOCOLOS_INC_BMP180_H_ */
