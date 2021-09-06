
#ifndef _LEDS_H_
#define _LEDS_H_

#include<stdint.h>
#include<stdbool.h>

void LedsInit(uint16_t * direccion);
void LedsTurnOn(int led);
void LedsTurnOff(int led);
void LedsTurnOnall(void);
void LedsTurnOffall(void);
bool LedsState(int led);

#endif