/*
 * BMP180.c
 *
 *  Created on: 16/06/2021
 *      Author: jdalvarado
 */


#include "BMP180.h"
#include "math.h"

#define		resolution		BMP180_ULTRALOWPOWER

BMP180_CAL_COEFF calCoeff;

bool BMP180_init(uint8_t addr)
{

  if (BMP180_read8(addr , BMP180_GET_ID_REG) != BMP180_CHIP_ID)
  return false; //safety check, make sure the sensor is connected

  return BMP180_readCalibrationCoefficients(addr);                         //safety check, make sure all coefficients are valid
}


/**************************************************************************/
/*
    getPressure()
    Calculates compensated pressure, in Pa
    NOTE:
    - resolutin 1Pa with accuracy ±150Pa at range 30,000Pa..110,000Pa
*/
/**************************************************************************/

int32_t BMP180_getPressure(uint8_t addr)
{
  int32_t  UT       = 0;
  int32_t  UP       = 0;
  int32_t  B3       = 0;
  int32_t  B5       = 0;
  int32_t  B6       = 0;
  int32_t  X1       = 0;
  int32_t  X2       = 0;
  int32_t  X3       = 0;
  int32_t  pressure = 0;
  uint32_t B4       = 0;
  uint32_t B7       = 0;

  UT = BMP180_readRawTemperature(addr);                                            //read uncompensated temperature, 16-bit
  if (UT == BMP180_ERROR) return BMP180_ERROR;                          //error handler, collision on i2c bus

  UP = BMP180_readRawPressure(addr);                                               //read uncompensated pressure, 19-bit
  if (UP == BMP180_ERROR) return BMP180_ERROR;                          //error handler, collision on i2c bus

  B5 = BMP180_computeB5(UT);

  /* pressure calculation */
  B6 = B5 - 4000;
  X1 = ((int32_t) calCoeff.bmpB2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t) calCoeff.bmpAC2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t) calCoeff.bmpAC1 * 4 + X3) << resolution) + 2) / 4;

  X1 = ((int32_t) calCoeff.bmpAC3 * B6) >> 13;
  X2 = ((int32_t) calCoeff.bmpB1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t) calCoeff.bmpAC4 * (X3 + 32768L)) >> 15;
  B7 = (UP - B3) * (50000UL >> resolution);

  if (B4 == 0) return BMP180_ERROR;                                     //safety check, avoiding division by zero

  if   (B7 < 0x80000000) pressure = (B7 * 2) / B4;
  else                   pressure = (B7 / B4) * 2;

  X1 = pow((pressure >> 8), 2);
  X1 = (X1 * 3038L) >> 16;
  X2 = (-7357L * pressure) >> 16;

  return pressure = pressure + ((X1 + X2 + 3791L) >> 4);
}

/**************************************************************************/
/*
    getTemperature()
    Calculates compensated temperature, in °C
    NOTE:
    - resolution 0.1°C with accuracy ±1.0°C at range 0°C..+65°C
*/
/**************************************************************************/
float BMP180_getTemperature(uint8_t addr)
{
  int16_t rawTemperature = BMP180_readRawTemperature(addr);

  if (rawTemperature == BMP180_ERROR) return BMP180_ERROR;                                       //error handler, collision on i2c bus
  return (float)(((rawTemperature) + 8) >> 4) / 10;
}

/**************************************************************************/
/*
    getSeaLevelPressure()
    Converts current pressure to sea level pressure at specific true
    altitude, in Pa
    NOTE:
    - true altitude is the actual elevation above sea level, to find out
      your current true altitude do search with google earth or gps
    - see level pressure is commonly used in weather reports & forecasts
      to compensate current true altitude
    - for example, we know that a sunny day happens if the current sea
      level pressure is 250Pa above the average sea level pressure of
      101325 Pa, so by converting the current pressure to sea level &
      comparing it with an average sea level pressure we can instantly
      predict the weather conditions
*/
/**************************************************************************/
int32_t BMP180_getSeaLevelPressure(uint8_t addr, int16_t trueAltitude)
{
  int32_t pressure = BMP180_getPressure(addr);

  if (pressure == BMP180_ERROR) return BMP180_ERROR;
  return (pressure / pow(1.0 - (float)trueAltitude / 44330, 5.255));
}

/**************************************************************************/
/*
    softReset()
    Soft reset
    NOTE:
    - performs the same sequence as power on reset
*/
/**************************************************************************/
void BMP180_softReset(uint8_t addr)
{
  BMP180_write8(addr, BMP180_SOFT_RESET_REG, BMP180_SOFT_RESET_CTRL);
}

/**************************************************************************/
/*
    readFirmwareVersion()
    Reads ML & AL Version
    NOTE:
    - ML version is LSB, 4-bit..0-bit
    - AL version is MSB, 7-bit..5-bit
*/
/**************************************************************************/
uint8_t BMP180_readFirmwareVersion(uint8_t addr)
{
  return BMP180_read8(addr, BMP180_GET_VERSION_REG);
}

/**************************************************************************/
/*
    readDeviceID()
    Reads chip ID
*/
/**************************************************************************/
uint8_t BMP180_readDeviceID(uint8_t addr)
{
  if (BMP180_read8(addr, BMP180_GET_ID_REG) == BMP180_CHIP_ID) return 180;
  return false;
}

/**************************************************************************/
/*
    readCalibrationCoefficients()
    Reads factory calibration coefficients from E2PROM
    NOTE:
    - every sensor module has individual calibration coefficients
    - before first temperature & pressure calculation master have to read
      calibration coefficients from 176-bit E2PROM
*/
/**************************************************************************/
bool BMP180_readCalibrationCoefficients(uint8_t addr)
{
  int32_t value = 0;

  calCoeff.bmpAC1 = 0;
  calCoeff.bmpAC2 = 0;
  calCoeff.bmpAC3 = 0;
  calCoeff.bmpAC4 = 0;
  calCoeff.bmpAC5 = 0;
  calCoeff.bmpAC6 = 0;
  calCoeff.bmpB1  = 0;
  calCoeff.bmpB2  = 0;
  calCoeff.bmpMB  = 0;
  calCoeff.bmpMC  = 0;
  calCoeff.bmpMD  = 0;

  for (uint8_t reg = BMP180_CAL_AC1_REG; reg <= BMP180_CAL_MD_REG; reg=reg+2)
  {
    value = BMP180_read16_data(addr, reg);

    if (value == BMP180_ERROR) return false; //error handler, collision on i2c bus

    switch (reg)
    {
      case BMP180_CAL_AC1_REG:               //used for pressure computation
        calCoeff.bmpAC1 = value;
        break;

      case BMP180_CAL_AC2_REG:               //used for pressure computation
        calCoeff.bmpAC2 = value;
        break;

      case BMP180_CAL_AC3_REG:               //used for pressure computation
        calCoeff.bmpAC3 = value;
        break;

      case BMP180_CAL_AC4_REG:               //used for pressure computation
        calCoeff.bmpAC4 = value;
        break;

      case BMP180_CAL_AC5_REG:               //used for temperature computation
        calCoeff.bmpAC5 = value;
        break;

      case BMP180_CAL_AC6_REG:               //used for temperature computation
        calCoeff.bmpAC6 = value;
        break;

      case BMP180_CAL_B1_REG:                //used for pressure computation
        calCoeff.bmpB1 = value;
        break;

      case BMP180_CAL_B2_REG:                //used for pressure computation
        calCoeff.bmpB2 = value;
        break;

      case BMP180_CAL_MB_REG:                //???
        calCoeff.bmpMB = value;
        break;

      case BMP180_CAL_MC_REG:                //used for temperature computation
        calCoeff.bmpMC = value;
        break;

      case BMP180_CAL_MD_REG:                //used for temperature computation
        calCoeff.bmpMD = value;
        break;
    }
  }

  return true;
}

/**************************************************************************/
/*
    readRawTemperature()
    Reads raw/uncompensated temperature value, 16-bit
*/
/**************************************************************************/
uint16_t BMP180_readRawTemperature(uint8_t addr)
{
  /* send temperature measurement command */
  if (BMP180_write8(addr, BMP180_START_MEASURMENT_REG, BMP180_GET_TEMPERATURE_CTRL) != true) return BMP180_ERROR; //error handler, collision on i2c bus

  /* set measurement delay */
  delay(5);

  /* read result */
  return BMP180_read16(addr, BMP180_READ_ADC_MSB_REG);                                                            //reads msb + lsb
}

/**************************************************************************/
/*
    readRawPressure()
    Reads raw/uncompensated pressure value, 19-bits
*/
/**************************************************************************/
uint32_t BMP180_readRawPressure(uint8_t addr)
{
  uint8_t  regControl  = 0;
  uint32_t rawPressure = 0;

  /* convert resolution to register control */
  switch (resolution)
  {
    case BMP180_ULTRALOWPOWER:                    //oss0
      regControl = BMP180_GET_PRESSURE_OSS0_CTRL;
      break;

    case BMP180_STANDARD:                         //oss1
      regControl = BMP180_GET_PRESSURE_OSS1_CTRL;
      break;

    case BMP180_HIGHRES:                          //oss2
      regControl = BMP180_GET_PRESSURE_OSS2_CTRL;
      break;

    case BMP180_ULTRAHIGHRES:                     //oss3
      regControl = BMP180_GET_PRESSURE_OSS3_CTRL;
      break;
  }

  /* send pressure measurement command */
  if (BMP180_write8(addr, BMP180_START_MEASURMENT_REG, regControl) != true) return BMP180_ERROR; //error handler, collision on i2c bus

  /* set measurement delay */
  switch (resolution)
  {
    case BMP180_ULTRALOWPOWER:
      delay(5);
      break;

    case BMP180_STANDARD:
      delay(8);
      break;

    case BMP180_HIGHRES:
      delay(14);
      break;

    case BMP180_ULTRAHIGHRES:
      delay(26);
      break;
  }

  /* read result msb + lsb */
  rawPressure = BMP180_read16(addr, BMP180_READ_ADC_MSB_REG);        //16-bits
  if (rawPressure == BMP180_ERROR) return BMP180_ERROR; //error handler, collision on i2c bus

  /* read result xlsb */
  rawPressure <<= 8;
  rawPressure |= BMP180_read8(addr, BMP180_READ_ADC_XLSB_REG);       //19-bits

  rawPressure >>= (8 - resolution);

  return rawPressure;
}

/**************************************************************************/
/*
    BMP180_computeB5()
    Computes B5 value
    NOTE:
    - to compensate raw/uncompensated temperature
    - also used for compensated pressure calculation
*/
/**************************************************************************/
int32_t BMP180_computeB5(int32_t UT)
{
  int32_t X1 = ((UT - (int32_t) calCoeff.bmpAC6) * (int32_t) calCoeff.bmpAC5) >> 15;
  int32_t X2 = ((int32_t) calCoeff.bmpMC << 11) / (X1 + (int32_t) calCoeff.bmpMD);

  return X1 + X2;
}

/**************************************************************************/
/*
    Calculating absolute altitude
*/
/**************************************************************************/

double BMP180_altitude(uint8_t addr)
{

	double K 	 	= 44330;
	double E 		= 1/5.255;
	double altitude = 0;
	double P0 		= 101325;
 // double P0 = BMP180_getSeaLevelPressure(BMP180_ADDR, 1618 );

	double P = BMP180_getPressure(BMP180_ADDR);

	altitude = K * (1.0 - pow((P/P0),E));

	return altitude;
}

/**************************************************************************/
/*
    read8()
    Reads 8-bit value over I2C
*/
/**************************************************************************/
uint8_t BMP180_read8(uint8_t addr, uint8_t reg)
{

	uint8_t value = 0;

	i2cWrite ( I2C , addr,   &reg, 1, OFF);

	i2cRead  ( I2C , addr, &value, 1, ON);

	return value;

}

/**************************************************************************/
/*
    read16()
    Reads 16-bits value over I2C
*/
/**************************************************************************/
uint16_t BMP180_read16(uint8_t addr, uint8_t reg)
{
	uint16_t value = 0;
	char data[2];

	i2cWrite ( I2C , addr,  &reg, 1, OFF);

	i2cRead  ( I2C , addr, data, 1, ON);

	value  =  data[0]<< 8 + data[1];

	return value;
}

uint16_t BMP180_read16_data(uint8_t addr, uint8_t reg)
{
	uint16_t value = 0;
	char data[2];

	i2cWrite ( I2C , addr,  &reg, 1, OFF);

	i2cRead  ( I2C , addr, &data[0], 1, ON);

	i2cWrite ( I2C , addr,  &reg+1, 1, OFF);

	i2cRead  ( I2C , addr, &data[1], 1, ON);

	value  =  data[0]<< 8 + data[1];

	return value;
}


/**************************************************************************/
/*
    BMP180_write8()
    Writes 8-bits value over I2C
*/
/**************************************************************************/
bool BMP180_write8(uint8_t addr, uint8_t reg, uint8_t control)
{
	bool_t  state;
	char data[2];

	data[0]=reg;
	data[1]=control;

	state = i2cWrite ( I2C , addr,  data, 2, ON);

	return state;

}
