
#ifndef _LEDS_H_
#define _LEDS_H_

#include<stdint.h>
#include<stdbool.h>

#define OFF	 0	
#define ON	 1	

void LedsInit(uint16_t * direccion);
void gpioInit(int led , bool estado);
void gpioWrite(int led , bool estado);
void Delay_us(int delay);
void Delay_ms(int delay);
void delay(int delay);

#endif