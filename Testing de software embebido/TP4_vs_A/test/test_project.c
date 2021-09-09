#include "unity.h"
#include "mock_i2c.h"
#include <stdbool.h>
#include "sapi.h"
#include "LCD.h"
#include "BMP180.h"

uint16_t ledsvirtuales;

void setUp (void)
{
    LedsInit(&ledsvirtuales);
}
void tearDown (void)
{
    
}
/* prueba para verificar si la LCD se inicializa correctamente. */
void test_init_LCD(void)
{
    bool valid =  0;
    valid = LCD_Init();
    TEST_ASSERT_TRUE(valid);
}
/* prueba para verificar si la LCD se envia un string correctamente. */
void test_init_LCD_SendString(void)
{
    bool    valid =  0;
   	char    Buffer[20];
    int	    Temperatura = 50;

    sprintf(Buffer, "Temperatura = %2d",Temperatura);
    valid = LCD_SendString(0,0,Buffer);
    TEST_ASSERT_TRUE(valid);
}
/* prueba para verificar si el sensor BMp180 se inicializa correctamente. */
void test_init_BMP180(void)
{
    bool valid =  0;
    uint8_t reg = 0x00, value = 0x00;

    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   reg, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR, value, 1, ON, true);

    for ( reg = BMP180_CAL_AC1_REG; reg <= BMP180_CAL_MD_REG; reg+=2)
    {
        i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   reg, 1, OFF, true);
        i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR, value, 1,  ON, true);
        i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,  reg+1, 1, OFF, true);
        i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR, value, 1,  ON, true);
    }

    valid = BMP180_init(BMP180_ADDR);
    TEST_ASSERT_TRUE(valid);
}
/* prueba para verificar si el sensor BMP180 realiza la medicion de altura. */
void test_init_BMP180_altitude(void)
{
    int32_t altitude =  0;
    uint8_t reg = 0x00, value = 0x00;

    // BMP180_readRawTemperature - BMP180_write8  
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR, BMP180_START_MEASURMENT_REG, 1, OFF, true);
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR, BMP180_GET_TEMPERATURE_CTRL, 1, ON, true);
  
    // BMP180_readRawTemperature - BMP180_write16 
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   BMP180_READ_ADC_MSB_REG, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);

    // BMP180_readRawPressure - BMP180_write8
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   BMP180_START_MEASURMENT_REG, 1, OFF, true);
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR, BMP180_GET_PRESSURE_OSS0_CTRL, 1, ON, true);

    // BMP180_readRawPressure - BMP180_read16 
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   BMP180_READ_ADC_MSB_REG, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);

    // BMP180_readRawPressure - BMP180_write8
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,      BMP180_READ_ADC_XLSB_REG, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                         value, 1, ON, true);

    altitude = BMP180_altitude(BMP180_ADDR);

    TEST_ASSERT_EQUAL_INT32( 44330,altitude);
}
/* prueba para verificar si el sensor BMP180 realiza la medicion de presion. */
void test_init_BMP180_getPressure(void)
{
    int32_t altitude =  0;
    uint8_t reg = 0x00, value = 0x00;

    // BMP180_readRawTemperature - BMP180_write8  
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR, BMP180_START_MEASURMENT_REG, 1, OFF, true);
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR, BMP180_GET_TEMPERATURE_CTRL, 1, ON, true);
  
    // BMP180_readRawTemperature - BMP180_write16 
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   BMP180_READ_ADC_MSB_REG, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);

    // BMP180_readRawPressure - BMP180_write8
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   BMP180_START_MEASURMENT_REG, 1, OFF, true);
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR, BMP180_GET_PRESSURE_OSS0_CTRL, 1, ON, true);

    // BMP180_readRawPressure - BMP180_read16 
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,   BMP180_READ_ADC_MSB_REG, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                     value, 1,  ON, true);

    // BMP180_readRawPressure - BMP180_write8
    i2cWrite_ExpectAndReturn ( I2C , BMP180_ADDR,      BMP180_READ_ADC_XLSB_REG, 1, OFF, true);
    i2cRead_ExpectAndReturn  ( I2C , BMP180_ADDR,                         value, 1, ON, true);

    altitude = BMP180_getPressure(BMP180_ADDR);

    TEST_ASSERT_EQUAL_INT32( 255,altitude);
}