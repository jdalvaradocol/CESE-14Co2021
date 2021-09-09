
/*
 * mian.c
 *
 *  Created on: 13/06/2021
 *      Author: jdalvarado
 */

/*==================[inclusiones]============================================*/

#include "sapi.h"

#include "../inc/ff.h"       // <= Biblioteca FAT FS
#include "../inc/fssdc.h"      // API de bajo nivel para unidad "SDC:" en FAT FS

#include "../inc/isl29023.h"
#include "../inc/SHT21.h"
#include "../inc/TMP006.h"
#include "../inc/BMP180.h"
#include "../inc/LCD.h"

/*==================[definiciones y macros]==================================*/

#define USED_UART UART_USB
#define UART_RATE 115200

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

extern UINT Read_nbytes;
extern UINT Write_nbytes;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr );

DEBUG_PRINT_ENABLE;

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{

	uint8_t n = 0;
	int index = 0;
	int32_t Pre = 0;

	double Temperatura = 0.0;
	double	Lux= 0.0;
	double	Hum= 0.0;
	double	Altura= 0.0;

	char	Buffer[20];
	char	Buffer_data[50];
	char	Buffer_read[50];


    // ---------- CONFIGURACIONES ------------------------------
    boardConfig();									// Inicializar y configurar la plataforma

    debugPrintConfigUart( USED_UART , UART_RATE );		// UART for debug messages

    SD_data_init();

    LCD_Config();

    LCD_Init();

    LCD_SendCmd(0x01);

    i2cInit(I2C0, 100000);

    TMP006_init(TMP_ADDR, 1);

    ISL29023_init(ISL29023_ADDR);

    SHT21_init(STH21_ADDR);

    BMP180_init(BMP180_ADDR);

    while(1)
    {

    	Temperatura = TMP006_readObjTempC(TMP_ADDR);
    	sprintf(Buffer, "Temperatura = %2.2f",Temperatura);
    	LCD_SendString(0, 0, Buffer);

    	Lux = ISL29023_read(ISL29023_ADDR);
    	sprintf(Buffer, "Lux = %4.2f",Lux);
    	LCD_SendString(1, 0, Buffer);

    	Hum = SHT21_getHumidity(STH21_ADDR);
    	sprintf(Buffer, "Humedad = %4.2f",Hum);
    	LCD_SendString(2, 0, Buffer);

    	// Pre = BMP180_getPressure(BMP180_ADDR);
    	// Pre = BMP180_getSeaLevelPressure(BMP180_ADDR, 1618 );
    	// sprintf(Buffer, "Presion = %d",Pre);
    	// LCD_SendString(3, 0, Buffer);

    	Altura =  BMP180_altitude(BMP180_ADDR);
    	sprintf(Buffer, "Altura = %4.2f",Altura);
    	LCD_SendString(3, 0, Buffer);

		//n = sprintf(Buffer_data, "%4.2f, %4.2f, %4.2f, %4.2f \r\n", Temperatura, Lux, Hum, Altura);

    	n = sprintf(Buffer_data, "%4d, %4d, %4d, %4d \r\n", index, index+1, index+2, index+3);

    	index += 4;

		Write_data_SD(Buffer_data, n);

		printf("Escritura |%d| \n\r",index);
		printf("%s", Buffer_data);
		printf("\n");

		printf("Write_nbytes |%d| \n\r",Write_nbytes);


		if(gpioRead(TEC1) == 0)
		{

			Read_nbytes = 0;

			for(int i =0;i<(index/4) ;i++ )
			{

				Read_data_SD(Buffer_read, n);
				printf("Lectura |%d| \n\r",i);
				printf("%s", Buffer_read);
				printf("\n");

				printf("Read_nbytes |%d| \n\r",Read_nbytes);

			}

			index = 0;

		}

		Delay_ms(5000);
    	gpioToggle(LED3);

    }

}



