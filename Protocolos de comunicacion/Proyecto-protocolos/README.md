# Proyecto protocolos

Proyecto final protocolos de comunicacion.

# Componentes.

- Edu -CIAA.
- LCD 4x20 -  4bits.

- Sensores 

- TMP006     - I2C. 
- BMP180     - I2C.
- ISL29023   - I2C.
- SHT21      - I2C.
- Memoria SD - SPI.

# A continuacion se relacionan las librerias y las fuciones del proyecto. 

## BMP180.h y BMP180.c

```c
bool     BMP180_init(uint8_t addr);
int32_t  BMP180_getPressure(uint8_t addr);                               	
float    BMP180_getTemperature(uint8_t addr);                            	
int32_t  BMP180_getSeaLevelPressure(uint8_t addr, int16_t trueAltitude);
void     BMP180_softReset(uint8_t addr);
uint8_t  BMP180_readFirmwareVersion(uint8_t addr);
uint8_t  BMP180_readDeviceID(uint8_t addr);
bool     BMP180_readCalibrationCoefficients(uint8_t addr);
uint16_t BMP180_readRawTemperature(uint8_t addr);
uint32_t BMP180_readRawPressure(uint8_t addr);
int32_t  BMP180_computeB5(int32_t UT);
double   BMP180_altitude(uint8_t addr);
uint8_t  BMP180_read8(uint8_t addr, uint8_t reg);
uint16_t BMP180_read16(uint8_t addr, uint8_t reg);
uint16_t BMP180_read16_data(uint8_t addr, uint8_t reg);
bool     BMP180_write8(uint8_t addr, uint8_t reg, uint8_t control);
```
## isl29023.h y isl29023.c

```c
void ISL29023_init(uint8_t addr);
float ISL29023_read(uint8_t addr);
```

## STH21.h y STH21.c

```c
uint16_t SHT21_readSensor_hm(uint8_t command, uint8_t addr);
float SHT21_CalcRH(uint16_t rh);
float SHT21_CalcT(uint16_t t);
uint8_t SHT21_CRC_Checksum(uint8_t data[], uint8_t no_of_bytes, uint8_t checksum);
float SHT21_getHumidity(uint8_t addr);
float SHT21_getTemperature(uint8_t addr);
void SHT21_init(uint8_t addr);
uint8_t SHT21_getSerialNumber(uint8_t return_sn, uint8_t addr);
```
## TMP006.h y TMP006.c

```c
void TMP006_init(uint8_t addr, uint16_t samples);
int16_t TMP006_readRawDieTemperature(uint8_t addr);
int16_t TMP006_readRawVoltage(uint8_t addr);
double TMP006_readObjTempC(uint8_t addr);
double TMP006_readObjTempF(uint8_t addr); 
double TMP006_readDieTempC(uint8_t addr);
double TMP006_readDieTempF(uint8_t addr)
```

## LCD.h y LCD.c

```c
void SIDWrite(uint8_t* buffer, uint32_t bufferSize);
void ParallelWrite(uint8_t dato);
void LCD_SendCmd (uint8_t cmd);
void LCD_SendData (uint8_t data);
void LCD_SendString(int row, int col, char* string);
void LCD_GraphicMode (int enable);
void LCD_Clear(void);
void LCD_DrawBitmap(const unsigned char* graphic);
void LCD_Update(void);
void LCD_Config (void);
void LCD_Init (void);
void SetPixel(uint8_t x, uint8_t y);
void DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius);
void DrawFilledCircle(int16_t x0, int16_t y0, int16_t r);
void DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);
void DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);
```
## SD_DATOS.h y SD_DATOS.c

```c
void SD_data_init	(void);
void Read_data_SD (char * buffer, uint8_t size);
void Write_data_SD(char * buffer, uint8_t size);
```
