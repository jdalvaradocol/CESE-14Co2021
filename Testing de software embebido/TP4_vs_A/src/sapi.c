

#include "sapi.h"

#define LEDS_ALL_OFF            0x0000
#define LEDS_ALL_ON             0xFFFF
#define LEDS_INDEX_OFFSET       1
#define LEDS_STATE_ON           1

static uint16_t * puerto;

static uint16_t LedsIndexToMask (int led)
{
    return (LEDS_STATE_ON << ( led - LEDS_INDEX_OFFSET));
}
void LedsInit(uint16_t * direccion)
{
    puerto = direccion;
    *direccion = LEDS_ALL_OFF;
}
void gpioInit(int led , bool estado)
{
    puerto = LEDS_ALL_OFF;
}
void gpioWrite(int led , bool estado)
{
    if(estado == ON)
    {
        *puerto |= LedsIndexToMask(led);
    }
    else if(estado == OFF)
    {
        *puerto &= (~LedsIndexToMask(led));
    }

}
void Delay_us(int delay)
{

}
void Delay_ms(int delay)
{

}
void delay(int delay)
{

}