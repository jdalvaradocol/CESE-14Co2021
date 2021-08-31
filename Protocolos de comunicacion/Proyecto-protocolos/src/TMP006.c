/*
 * TMP006.c
 *
 *  Created on: 13/06/2021
 *      Author: jdalvarado
 */


#include "../../Proyecto_protocolos/inc/TMP006.h"

#include "math.h"

/** TMP006 Config method.
 *  Used for configuring the sensor with desired samples.
 */
void TMP006_init(uint8_t addr, uint16_t samples)
{
    char    data[2];
 
    data[0]   = TMP006_CONFIG;
    data[1]   = samples | TMP006_CFG_MODEON | TMP006_CFG_DRDYEN;
 
    i2cWrite( I2C , addr, data, 2 , ON); // writes the config (0x02) address with the sample rate
  
}
 
/** TMP006 Read Raw Temperature method.
 *  Used for getting the raw data off the chip.
 */
int16_t TMP006_readRawDieTemperature(uint8_t addr)
{
 
  char reg[1] ;
  char data[2];
  
  reg[0] = TMP006_TAMB;
  
  i2cWrite ( I2C , addr,  reg, 1, OFF);
  i2cRead  ( I2C , addr, data, 2, ON);
   
  int16_t raw = (data[0] << 8) | data[1] ;
 
  raw >>= 2;
  return raw;
}
 
/** TMP006 Read Raw Voltage method.
 *  Used for reading the raw voltage from the thermopile.
 */
int16_t TMP006_readRawVoltage(uint8_t addr)
{  
  char reg[1];
  char data[2];
  
  reg[0] = TMP006_VOBJ;
  
  i2cWrite ( I2C , addr,  reg, 1, OFF);
  i2cRead  ( I2C , addr, data, 2, ON);
   
  int16_t raw = (data[0] << 8) | data[1] ;
  
  return raw;
}
 
/** TMP006 Read Object Temperature(C) method.
 *  Used for calculating the object temperature in celcius.
 */
double TMP006_readObjTempC(uint8_t addr)
{
  double Tdie = TMP006_readRawDieTemperature(addr);
  double Vobj = TMP006_readRawVoltage(addr);
  Vobj *= 156.25;  // 156.25 nV per LSB
  Vobj /= 1000000000; // nV -> V
  Tdie *= 0.03125; // convert to celsius
  Tdie += 273.15; // convert to kelvin
 
  // Equations for calculating temperature found in section 5.1 in the user guide
  double tdie_tref = Tdie - TMP006_TREF;
  double S = (1 + TMP006_A1*tdie_tref + 
      TMP006_A2*tdie_tref*tdie_tref);
  S *= TMP006_S0;
  S /= 10000000;
  S /= 10000000;
 
  double Vos = TMP006_B0 + TMP006_B1*tdie_tref + 
    TMP006_B2*tdie_tref*tdie_tref;
 
  double fVobj = (Vobj - Vos) + TMP006_C2*(Vobj-Vos)*(Vobj-Vos);
 
  double Tobj = sqrt(sqrt(Tdie * Tdie * Tdie * Tdie + fVobj/S));
 
  Tobj -= 273.15; // Kelvin -> *C
  return Tobj;
}
 
/** TMP006 Read Object Temperature(F) method.
 *  Used for calculating the object temperature in fahrenheit.
 */
double TMP006_readObjTempF(uint8_t addr)
{
  double Tobj = TMP006_readObjTempC(addr);
  Tobj = Tobj * 9.0/5.0 + 32.0; // convert to fahrenheit
  return Tobj;
}
 
/** TMP006 Read Die Temperature(C) method.
 *  Used for calculating the die temperature in celcius.
 */
double TMP006_readDieTempC(uint8_t addr)
{
  double Tdie = TMP006_readRawDieTemperature(addr);
  Tdie *= 0.03125; // convert to celsius
  return Tdie;
}
 
/** TMP006 Read Die Temperature(F) method.
 *  Used for calculating the die temperature in fahrenheit.
 */
double TMP006_readDieTempF(uint8_t addr)
{
  double Tdie = TMP006_readDieTempC(addr);
  Tdie = Tdie * 9.0/5.0 + 32.0; // convert to fahrenheit
  return Tdie;
}
