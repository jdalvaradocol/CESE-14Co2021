/*
 * isl29023.c
 *
 *  Created on: 13/06/2021
 *      Author: jdalvarado
 */

#include "../../Proyecto_protocolos/inc/isl29023.h"

#define ISL29023_ADDR 0x44

#define CMD1 0
#define CMD2 1
#define DATALSB 2
#define DATAMSB 3

void ISL29023_init(uint8_t addr)
{
  unsigned char i2cdata[2];

  i2cdata[0] = CMD1;
  i2cdata[1] = 0b10100000;

  i2cWrite( I2C , addr, i2cdata, 2 , ON); // writes the config (0x02) address with the sample rate

  i2cdata[0] = CMD2;
  i2cdata[1] = 0b00000011;

  i2cWrite( I2C , addr, i2cdata, 2 , ON); // writes the config (0x02) address with the sample rate

}

/*
 *  Reads the lux value back from the sensor.
*/

float ISL29023_read(uint8_t addr)
{
  unsigned char  i2cdata;
  unsigned int light;
  float lux;

  i2cdata = DATAMSB;

  i2cWrite( I2C , addr, &i2cdata, 1, OFF); // writes the config (0x02) address with the sample rate

  i2cRead ( I2C , addr, &i2cdata, 1,  ON); // read MSB

  light = ((unsigned int)i2cdata)<<8;

  i2cdata = DATALSB;

  i2cWrite( I2C , addr, &i2cdata, 1, OFF); // writes the config (0x03) address with the sample rate

  i2cRead ( I2C , addr, &i2cdata, 1,  ON); // read LSB

  light |= i2cdata;

  // this is a bit lame, ideally use data read back from the device
  // to scale accordingly
  lux = (64000 * (float)light)/65536;
  return lux;
 }
