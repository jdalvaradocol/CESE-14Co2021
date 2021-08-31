/*
 * SD_DATOS.c
 *
 *  Created on: 17/06/2021
 *      Author: jdalvarado
 */

#include "SD_DATOS.h"

// FUNCION que se ejecuta cada vez que ocurre un Tick
void diskTickHook( void *ptr );

void SD_data_init(void)
{

	// SPI configuration
	spiConfig( SPI0 );

	// Inicializar el conteo de Ticks con resolucion de 10ms,
	// con tickHook diskTickHook
	tickConfig( 100 );
	tickCallbackSet( diskTickHook, NULL );

	// ------ PROGRAMA QUE ESCRIBE EN LA SD -------

	// Initialize SD card driver
	FSSDC_InitSPI ();
	// Give a work area to the default drive
	if( f_mount( &fs, "SDC:", 0 ) != FR_OK ) {
	// If this fails, it means that the function could
	// not register a file system object.
	// Check whether the SD card is correctly connected
	}

	// Create/open a file, then write a string and close it

	if( f_open( &fp, FILENAME, FA_WRITE | FA_OPEN_APPEND ) == FR_OK )
	{
		f_write(&fp, "Temperatura,Lumens,Humedad,Altura\r\n", 35, &Write_nbytes);
		f_close(&fp);

        if( Write_nbytes == 35 )
        {
        // Turn ON LEDG if the write operation was successful
        	gpioWrite( LEDG, ON );
	    }
	}
	else
	{
		// Turn ON LEDR if the write operation was fail
	    gpioWrite( LEDR, ON );
	}

}
void Read_data_SD(char * buffer, uint8_t size)
{
	static uint32_t contador = 0;

	contador += size;

	if( f_open( &fp, FILENAME, FA_READ ) == FR_OK )
	{
		while (!f_eof( &fp ) )
		{
			Read_nbytes = contador;
	        f_read ( &fp , buffer, size, &Read_nbytes);
	    }
	   }
	else
	printf(" Error al abrir archivo\n");

	fflush(stdout);
}
void Write_data_SD(char * buffer, uint8_t size)
{

	if (f_open(&fp, FILENAME, FA_WRITE | FA_OPEN_APPEND) == FR_OK)
	{
		f_write(&fp, buffer, size, &Write_nbytes);
		f_close(&fp);

		if (Write_nbytes == size)
		{
		// Turn ON LED1 if the write operation was successful
			gpioWrite(LED1, ON);
		}
	}
	else
	{
		// Turn OFF LED1 if the write operation was fail
		gpioWrite(LED1, OFF);
	}

}

/*==================[definiciones de funciones externas]=====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
}

