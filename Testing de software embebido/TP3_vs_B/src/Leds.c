
#include "Leds.h"

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
void LedsTurnOn(int led)
{
    *puerto |= LedsIndexToMask(led);
}
void LedsTurnOff(int led)
{
    *puerto = (*puerto) & (~LedsIndexToMask(led));
}
void LedsTurnOnall(void)
{
    *puerto = LEDS_ALL_ON;
}
void LedsTurnOffall(void)
{
    *puerto = LEDS_ALL_OFF;
}
bool LedsState(int led)
{
    bool a = (*puerto) & (LedsIndexToMask(led));

    return (a);
}
