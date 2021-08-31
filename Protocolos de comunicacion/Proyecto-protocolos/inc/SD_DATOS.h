/*
 * SD_DATOS.h
 *
 *  Created on: 17/06/2021
 *      Author: jdalvarado
 */

#ifndef ACTIVIDADES_PROYECTO_PROTOCOLOS_INC_SD_DATOS_H_
#define ACTIVIDADES_PROYECTO_PROTOCOLOS_INC_SD_DATOS_H_

#include "sapi.h"
#include "../inc/ff.h"       // <= Biblioteca FAT FS
#include "../inc/fssdc.h"      // API de bajo nivel para unidad "SDC:" en FAT FS

/*==================[definiciones y macros]==================================*/

#define FILENAME "SDC:/Datos.csv"

/*==================[definiciones de datos internos]=========================*/

static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file

UINT Read_nbytes;
UINT Write_nbytes;

void SD_data_init	(void);
void Read_data_SD (char * buffer, uint8_t size);
void Write_data_SD(char * buffer, uint8_t size);

#endif /* ACTIVIDADES_PROYECTO_PROTOCOLOS_INC_SD_DATOS_H_ */
