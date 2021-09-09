/*
 * LCD.c
 *
 *  Created on: 13/06/2021
 *      Author: jdalvarado
 */

#include "LCD.h"

uint8_t startRow, startCol, endRow, endCol; // coordinates of the dirty rectangle
uint8_t numRows = 64;
uint8_t numCols = 128;
uint8_t Graphic_Check = 0;

//GLCD Buf
uint8_t GLCD_Buf[1024];

// A replacement for SPI_TRANSMIT

void LCD_Config (void)
{
	gpioInit( RST, GPIO_OUTPUT );
	gpioInit( PSB, GPIO_OUTPUT );

	if(MODE == 0) // SERIAL
	{

		if(SERIAL == 0) // SID
		{
			gpioInit(   CS, GPIO_OUTPUT );
			gpioInit(  SID, GPIO_OUTPUT );
			gpioInit( SCLK, GPIO_OUTPUT );
		}

		// LCD is in serial interface mode when pulling down PSB pin

		gpioWrite( PSB, OFF );

	}
	else if((MODE == 1) || (MODE == 2)) // Parallel bus 4 bits
	{
		gpioInit(   RS, GPIO_OUTPUT );
		gpioInit(   RW, GPIO_OUTPUT );
		gpioInit( 	 E, GPIO_OUTPUT );
		gpioInit( LCD4, GPIO_OUTPUT );
		gpioInit( LCD3, GPIO_OUTPUT );
		gpioInit( LCD2, GPIO_OUTPUT );
		gpioInit( LCD1, GPIO_OUTPUT );

		// LCD is in parallel mode by pulling up PSB pin.
		gpioWrite( PSB , ON );
	}

}

void ParallelWrite(uint8_t dato)
{
	gpioWrite( LCD4, ( dato & 0x80 ) );
	gpioWrite( LCD3, ( dato & 0x40 ) );
	gpioWrite( LCD2, ( dato & 0x20 ) );
	gpioWrite( LCD1, ( dato & 0x10 ) );
}

void SIDWrite(uint8_t* buffer, uint32_t bufferSize)
{
	uint32_t data 	= 0;
	uint32_t length = 0;
	uint32_t bit 	= 0;
	uint32_t bitbuf[32];

	length =  bufferSize / 8;	data = 0;

	for(int i=0;i < length;i++)
	{
		data = data + (buffer[i] << (bufferSize - 8 - (i*8)));
	}

	for(int i=bufferSize-1;i >= 0;i--)
	{
		bit = 2 ^ i;
		bitbuf[i]= (data & bit) >> i;
	}

	gpioWrite(CS, ON);		// PUll the CS high
	Delay_us(2);

	gpioWrite (SID, OFF);
	Delay_us(1);

	for(int i=bufferSize-1;i >= 0;i--)
	{

		gpioWrite (SID, bitbuf[i] );

		gpioWrite(SCLK, ON);		// PUll the CS high
		Delay_us(2);

		gpioWrite(SCLK, OFF);		// PUll the CS high
		Delay_us(2);
	}

	gpioWrite (SID, ON);
	Delay_us(2);

	gpioWrite(CS, OFF);		// PUll the CS high

}

void LCD_SendCmd (uint8_t cmd)
{
	uint8_t buffer[3];

	buffer[0] = 0xF8;
	buffer[1] = cmd & 0xF0;
	buffer[2] = (cmd & 0x0F) << 4;

	if(MODE == 0) // SERIAL
	{
		if(SERIAL == 0) // SID
		{
			SIDWrite(buffer, 24);  	// send the SYNC + RS(0)
		}

	}
	else if((MODE == 1) || (MODE == 2)) // Parallel bus 4 bits
	{
		gpioWrite(RS, OFF);		// RS  = 0
		gpioWrite(RW, OFF);		// RW  = 0

		ParallelWrite(buffer[1]);

		Delay_us(5);
		gpioWrite(E, ON);		// E  = 0
		Delay_us(5);
		gpioWrite(E, OFF);		// E  = 1

		ParallelWrite(buffer[2]);

		Delay_us(5);
		gpioWrite(E, ON);		// E  = 0
		Delay_us(5);
		gpioWrite(E, OFF);		// E  = 1
		Delay_us(5);
	}

	if (cmd == 0x01) 	Delay_us(1600); 		// 1.6mS
	else 				Delay_us(72);       	// 72uS

}

void LCD_SendData (uint8_t data)
{

	uint8_t buffer[3];

	buffer[0] = 0xFA;
	buffer[1] =  data & 0xF0;
	buffer[2] = (data & 0x0F) << 4;

	if(MODE == 0) // SERIAL
	{
		if(SERIAL == 0) // SID
		{
			SIDWrite(buffer, 24);  	// send the SYNC + RS(0)
		}
	}
	else if((MODE == 1) || (MODE == 2)) // Parallel bus 4 bits
	{
		gpioWrite(RS, ON);		// RS  = 1
		gpioWrite(RW, OFF);		// RW  = 0

		ParallelWrite(buffer[1]);

		Delay_us(5);
		gpioWrite(E, ON);		// E  = 0
		Delay_us(5);
		gpioWrite(E, OFF);		// E  = 1

		ParallelWrite(buffer[2]);

		Delay_us(5);
		gpioWrite(E, ON);		// E  = 0
		Delay_us(5);
		gpioWrite(E, OFF);		// E  = 1
		Delay_us(5);

	}

	Delay_us(72);										// 72uS
}

bool LCD_SendString(int row, int col, char* string)
{

	if((MODE == 0) || (MODE == 1))
	{

	col &= 0x0F;

    switch (row)
    {
        case 0:
            col += 0x80;
            break;
        case 1:
            col += 0x90;
            break;
        case 2:
            col += 0x88;
            break;
        case 3:
            col += 0x98;
            break;
        default:
            col += 0x80;
            break;
    }

	}
	else if(MODE == 2)
	{
	    switch (row)
	    {
	        case 0:
	            col += 128;
	            break;
	        case 1:
	            col += 192;
	            break;
	        case 2:
	            col += 148;
	            break;
	        case 3:
	            col += 212;
	            break;
	        default:
	            col += 128;
	            break;
	    }

	}

    LCD_SendCmd(col);

    while (*string)
    	{
    		LCD_SendData(*string++);
    	}

	return true;

}
// switch to graphic mode or normal mode::: enable = 1 -> graphic mode enable = 0 -> normal mode

void LCD_GraphicMode (int enable)   // 1-enable, 0-disable
{
	if (enable == 1)
	{
		LCD_SendCmd(0x30);  // 8 bit mode
		Delay_ms (1);
		LCD_SendCmd(0x34);  // switch to Extended instructions
		Delay_ms (1);
		LCD_SendCmd(0x36);  // enable graphics
		Delay_ms (1);
		Graphic_Check = 1;  // update the variable
	}

	else if (enable == 0)
	{
		LCD_SendCmd(0x30);  // 8 bit mode
		Delay_ms (1);
		Graphic_Check = 0;  // update the variable
	}
}

void LCD_DrawBitmap(const unsigned char* graphic)
{
	
	uint8_t x, y;
	
	uint16_t Index=0;
	uint8_t Temp,Db;
	
	for(y=0;y<64;y++)
	{
		for(x=0;x<8;x++)
		{
			if(y<32)//Up
			{
				LCD_SendCmd(0x80 | y);										//Y(0-31)
				LCD_SendCmd(0x80 | x);										//X(0-8)
			}
			else
			{
				LCD_SendCmd(0x80 | y-32);//Y(0-31)
				LCD_SendCmd(0x88 | x);//X(0-8)
			}
			
			Index=((y/8)*128)+(x*16);
			Db=y%8;			
			
			Temp=	(((graphic[Index+0]>>Db)&0x01)<<7)|
						(((graphic[Index+1]>>Db)&0x01)<<6)|
						(((graphic[Index+2]>>Db)&0x01)<<5)|
						(((graphic[Index+3]>>Db)&0x01)<<4)|
						(((graphic[Index+4]>>Db)&0x01)<<3)|
						(((graphic[Index+5]>>Db)&0x01)<<2)|
						(((graphic[Index+6]>>Db)&0x01)<<1)|
						(((graphic[Index+7]>>Db)&0x01)<<0);
			LCD_SendData(Temp);
			
			Temp=	(((graphic[Index+8]>>Db)&0x01)<<7)|
						(((graphic[Index+9]>>Db)&0x01)<<6)|
						(((graphic[Index+10]>>Db)&0x01)<<5)|
						(((graphic[Index+11]>>Db)&0x01)<<4)|
						(((graphic[Index+12]>>Db)&0x01)<<3)|
						(((graphic[Index+13]>>Db)&0x01)<<2)|
						(((graphic[Index+14]>>Db)&0x01)<<1)|
						(((graphic[Index+15]>>Db)&0x01)<<0);
			
			LCD_SendData(Temp);
		}
	}		
}

//Clear GLCD Buf
void GLCD_Buf_Clear(void)
{
	uint16_t i;
	for(i=0;i<1024;i++)GLCD_Buf[i]=0;
}

// Update the display with the selected graphics
void LCD_Update(void)
{
	LCD_DrawBitmap(GLCD_Buf);
}

void LCD_Clear()
{
	if (Graphic_Check == 1)  // if the graphic mode is set
	{
		uint8_t x, y;
		for(y = 0; y < 64; y++)
		{
			if(y < 32)
			{
				LCD_SendCmd(0x80 | y);
				LCD_SendCmd(0x80);
			}
			else
			{
				LCD_SendCmd(0x80 | (y-32));
				LCD_SendCmd(0x88);
			}
			for(x = 0; x < 8; x++)
			{
				LCD_SendData(0);
				LCD_SendData(0);
			}
		}
		GLCD_Buf_Clear();
	}
	else
	{
		LCD_SendCmd(0x01);   // clear the display using command
	}
}

int LCD_Init (void)
{

	gpioWrite(RS,OFF);			// RS = 0
	gpioWrite(CS,OFF);			// CS = 0
	gpioWrite(RW,OFF);			// RW = 0
	gpioWrite( E,OFF);			//  E = 0

	gpioWrite(RST,OFF);			// RESET=0
	Delay_ms(100);   			// wait for 1ms
	gpioWrite(RST,ON);;  		// RESET=1
	Delay_ms(50);   			// wait for >40 ms

	if(MODE == 0)
	{

		LCD_SendCmd(0x30);  			// 8bit mode
		Delay_us(110); 			 		//  >100us delay

		LCD_SendCmd(0x30);  			// 8bit mode
		Delay_us(40);  					// >37us delay

		LCD_SendCmd(0x01);  			// clear screen
		Delay_ms(12);  					// >10 ms delay

		LCD_SendCmd(0x06);  			// cursor increment right no shift
		Delay_us(40);  					// >37us delay

		LCD_SendCmd(0x0C);  			// D=1, C=0, B=0
		Delay_us(40);  					// >37us delay

		LCD_SendCmd(0x34);  			// return to home
		Delay_us(40);  					// >37us delay

		LCD_SendCmd(0x36);  			// return to home
		Delay_us(40);  					// >37us delay

	}
	else if(MODE == 1)
	{

		LCD_SendCmd(0x20);  			// Function SET
		Delay_ms(10);  					// >100us delay

		LCD_SendCmd(0x20);  			// Function SET
		Delay_ms(10);  					// >100us delay

		LCD_SendCmd(0x0C);  			// Function SET
		Delay_ms(1);  					// >100us delay

		LCD_SendCmd(0x06);  			// Function SET
		Delay_ms(10);  					// >10ms delay

		LCD_SendCmd(0x02);  			// Function SET
		Delay_ms(20);  					// >100us delay

	}
	else if(MODE == 2)
	{

	    for (int data = 1; data < 4; data++)
		    {

	    		ParallelWrite(0x30);
		    	gpioWrite(RS, OFF );
		    	gpioWrite( E, OFF );

		    	ParallelWrite(0x30);
		    	gpioWrite(RS, OFF );
		    	gpioWrite( E, ON  );

				Delay_us(5);

				ParallelWrite(0x30);
				gpioWrite(RS, OFF );
				gpioWrite( E, OFF );

				Delay_us(5500);

		    }

		    ParallelWrite(0x20);
		   	gpioWrite(RS, OFF );
		    gpioWrite( E, OFF );

		    ParallelWrite(0x20);
   		   	gpioWrite(RS, OFF );
   		    gpioWrite( E, ON  );

			Delay_us(5);

			ParallelWrite(0x20);
		   	gpioWrite(RS, OFF );
			gpioWrite( E, OFF );

		    Delay_us(5500);

			LCD_SendCmd(40);  			// 8bit mode

			LCD_SendCmd(16);  			// 8bit mode

			LCD_SendCmd(1);  			// clear screen

			LCD_SendCmd(12);  			// 8bit mode
	}

	return true;

}


// Set Pixel
void SetPixel(uint8_t x, uint8_t y)
{
  if (y < numRows && x < numCols)
  {
		GLCD_Buf[(x)+((y/8)*128)]|=0x01<<y%8;

    // Change the dirty rectangle to account for a pixel being dirty (we assume it was changed)
    if (startRow > y) { startRow = y; }
    if (endRow <= y)  { endRow = y + 1; }
    if (startCol > x) { startCol = x; }
    if (endCol <= x)  { endCol = x + 1; }
  }
}

// Clear Pixel
void ClearPixel(uint8_t x, uint8_t y)
{
  if (y < numRows && x < numCols)
  {
		GLCD_Buf[(x)+((y/8)*128)]&=(~(0x01<<y%8));

    // Change the dirty rectangle to account for a pixel being dirty (we assume it was changed)
    if (startRow > y) { startRow = y; }
    if (endRow <= y)  { endRow = y + 1; }
    if (startCol > x) { startCol = x; }
    if (endCol <= x)  { endCol = x + 1; }
  }
}

// Toggle Pixel
void TogglePixel(uint8_t x, uint8_t y)
{
  if (y < numRows && x < numCols)
  {
		if((GLCD_Buf[(x)+((y/8)*128)]>>y%8)&0x01)
			ClearPixel(x,y);
		else
			SetPixel(x,y);
  }
}
/* draw a line
 * start point (X0, Y0)
 * end point (X1, Y1)
 */
void DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
  int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  for (;;)
  {
    SetPixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = err + err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}

/* Draw rectangle
 * start point (x,y)
 * w -> width
 * h -> height
 */
void DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	/* Check input parameters */
	if (
		x >= numCols ||
		y >= numRows
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= numCols) {
		w = numCols - x;
	}
	if ((y + h) >= numRows) {
		h = numRows - y;
	}

	/* Draw 4 lines */
	DrawLine(x, y, x + w, y);         /* Top line */
	DrawLine(x, y + h, x + w, y + h); /* Bottom line */
	DrawLine(x, y, x, y + h);         /* Left line */
	DrawLine(x + w, y, x + w, y + h); /* Right line */
}

/* Draw filled rectangle
 * Start point (x,y)
 * w -> width
 * h -> height
 */
void DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	uint8_t i;

	/* Check input parameters */
	if (
		x >= numCols ||
		y >= numRows
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= numCols) {
		w = numCols - x;
	}
	if ((y + h) >= numRows) {
		h = numRows - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		DrawLine(x, y + i, x + w, y + i);
	}
}

/* draw circle
 * centre (x0,y0)
 * radius = radius
 */
void DrawCircle(uint8_t x0, uint8_t y0, uint8_t radius)
{
  int f = 1 - (int)radius;
  int ddF_x = 1;

  int ddF_y = -2 * (int)radius;
  int x = 0;

  SetPixel(x0, y0 + radius);
  SetPixel(x0, y0 - radius);
  SetPixel(x0 + radius, y0);
  SetPixel(x0 - radius, y0);

  int y = radius;
  while(x < y)
  {
    if(f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    SetPixel(x0 + x, y0 + y);
    SetPixel(x0 - x, y0 + y);
    SetPixel(x0 + x, y0 - y);
    SetPixel(x0 - x, y0 - y);
    SetPixel(x0 + y, y0 + x);
    SetPixel(x0 - y, y0 + x);
    SetPixel(x0 + y, y0 - x);
    SetPixel(x0 - y, y0 - x);
  }
}

// Draw Filled Circle
void DrawFilledCircle(int16_t x0, int16_t y0, int16_t r)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    SetPixel(x0, y0 + r);
    SetPixel(x0, y0 - r);
    SetPixel(x0 + r, y0);
    SetPixel(x0 - r, y0);
    DrawLine(x0 - r, y0, x0 + r, y0);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        DrawLine(x0 - x, y0 + y, x0 + x, y0 + y);
        DrawLine(x0 + x, y0 - y, x0 - x, y0 - y);

        DrawLine(x0 + y, y0 + x, x0 - y, y0 + x);
        DrawLine(x0 + y, y0 - x, x0 - y, y0 - x);
    }
}

// Clear Filled Circle
void ClearFilledCircle(int16_t x0, int16_t y0, int16_t r)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

    ClearPixel(x0, y0 + r);
    ClearPixel(x0, y0 - r);
    ClearPixel(x0 + r, y0);
    ClearPixel(x0 - r, y0);
    ClearLine(x0 - r, y0, x0 + r, y0);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ClearLine(x0 - x, y0 + y, x0 + x, y0 + y);
        ClearLine(x0 + x, y0 - y, x0 - x, y0 - y);

        ClearLine(x0 + y, y0 + x, x0 - y, y0 + x);
        ClearLine(x0 + y, y0 - x, x0 - y, y0 - x);
    }
}

// Draw Traingle with coordimates (x1, y1), (x2, y2), (x3, y3)
void DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3)
{
	/* Draw lines */
	DrawLine(x1, y1, x2, y2);
	DrawLine(x2, y2, x3, y3);
	DrawLine(x3, y3, x1, y1);
}

// Draw Filled Traingle with coordimates (x1, y1), (x2, y2), (x3, y3)
void DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

#define ABS(x)   ((x) > 0 ? (x) : -(x))

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		DrawLine(x, y, x3, y3);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

//Clear Line
void ClearLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
  int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  for (;;)
  {
    ClearPixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = err + err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}

//Toggle Line
void ToggleLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  int dx = (x1 >= x0) ? x1 - x0 : x0 - x1;
  int dy = (y1 >= y0) ? y1 - y0 : y0 - y1;
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  for (;;)
  {
    TogglePixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = err + err;
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
}

//Print Fonted String x=0-15 y=0-7
/*
void GLCD_Font_Print(uint8_t x,uint8_t y,char * String)
{
	int i;
	while(*String)
	{
		for(i=0;i<8;i++)
			GLCD_Buf[i+(x*8)+(y*128)]=Font[(*String)*8+i];
		String++;
		x++;
		if(x>15)
		{
			x=0;
			y++;
		}
	}
}
*/
//Print ICON(8*8) x=0-15 y=0-7
void GLCD_ICON_Print(uint8_t x,uint8_t y,const uint8_t * ICON)
{
	int i;
	for(i=0;i<8;i++)
		GLCD_Buf[i+(x*8)+(y*128)]=ICON[i];
}

/* Toggle rectangle
 * Start point (x,y)
 * w -> width
 * h -> height
 */
void ToggleRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	uint8_t i;

	/* Check input parameters */
	if (
		x >= numCols ||
		y >= numRows
	) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= numCols) {
		w = numCols - x;
	}
	if ((y + h) >= numRows) {
		h = numRows - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		ToggleLine(x, y + i, x + w, y + i);
	}
}

//Debug
void GLCD_Test(void)
{

}
