/*
 * LCD.h
 *
 *  Created on: 13/06/2021
 *      Author: jdalvarado
 */

#ifndef __ST7920_H
#define __ST7920_H

#include "stdint.h"
#include "sapi.h"
#include <math.h>

#define SPI 		0
#define RST		 	1
#define PSB		 	2
#define RS 			3
#define RW 			4
#define E 			5
#define CS	 		6
#define SID 		7
#define SCLK	 	8
#define LCD4	 	9	
#define LCD3	 	10	
#define LCD2	 	11	
#define LCD1	 	12	
#define MODE	 	2	// 0 = SERIAL 	- 1 = Parallel bus 4 bits	- 2 = LCD
#define SERIAL	 	0	// 0 = SID 		- 1 = SPI
#define GPIO_OUTPUT	 	0	
#define GPIO_INPUT	 	1	

// Send the command to the LCD
void SIDWrite(uint8_t* buffer, uint32_t bufferSize);

void ParallelWrite(uint8_t dato);

// Send the command to the LCD
void LCD_SendCmd (uint8_t cmd);

// send the data to the LCD
void LCD_SendData (uint8_t data);

/* send the string to the LCD
 * 'row' = starting ROW for the string (from 0 to 3)
 * 'col' = starting COL for the string (from 0 to 7)
 */
bool LCD_SendString(int row, int col, char* string);

/* ENABLE or DISABLE the graphic mode
 * enable =1 --> graphic mode enabled
 */
void LCD_GraphicMode (int enable);

// clear screen in any mode
void LCD_Clear(void);

// Draw bitmap on the display
void LCD_DrawBitmap(const unsigned char* graphic);

// Update the display with the selected graphics
void LCD_Update(void);

// Initialize the display
void LCD_Config (void);
int LCD_Init (void);

/* Common functions used
 * in other LCDs also
 */

// Set a pixel on the display
void SetPixel(uint8_t x, uint8_t y);

// draw line from (X0, Y0) to (X1, Y1)
void DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

// draw rectangle from (X,Y) w- width, h- height
void DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

// draw filled rectangle
void DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

// draw circle with centre (X0, Y0) and radius= radius
void DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius);

// Draw Filled Circle with centre (X0, Y0) and radius= r

void DrawFilledCircle(int16_t x0, int16_t y0, int16_t r);

// Draw Traingle with coordimates (x1, y1), (x2, y2), (x3, y3)
void DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);

// Draw Filled Traingle with coordimates (x1, y1), (x2, y2), (x3, y3)
void DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3);

//Other
void GLCD_Buf_Clear(void);
void GLCD_Font_Print(uint8_t x,uint8_t y,char * String);
void ClearPixel(uint8_t x, uint8_t y);
void TogglePixel(uint8_t x, uint8_t y);
void ClearLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ToggleLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ToggleRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void ClearPixel(uint8_t x, uint8_t y);
void ClearFilledCircle(int16_t x0, int16_t y0, int16_t r);
void GLCD_ICON_Print(uint8_t x,uint8_t y,const uint8_t * ICON);

//Debug
void GLCD_Test(void);

#endif

