
#include "unity.h"
#include <stdbool.h>
#include "Leds.h"

uint16_t ledsvirtuales;

void setUp (void)
{
    LedsInit(&ledsvirtuales);
}
void tearDown (void)
{
    
}
/* prueba para que cuando se reliza la inicialización todos los LEDs deban quedar apagados. */
void test_LedsOffAfterCreate (void)
{
    uint16_t ledsvirtuales = 0xFFFF;
    LedsInit(&ledsvirtuales);
    TEST_ASSERT_EQUAL_HEX16(0x0000, ledsvirtuales);
}
/* Prueba para que se puedan prender un LED individual. */
void test_prender_un_led (void)
{
    const int led = 2;
    LedsTurnOn(led);
    TEST_ASSERT_EQUAL_HEX16(1 << (led-1), ledsvirtuales);
}
/* Prueba para que ee puedan apagar un LED individual. */
void test_apagar_un_led (void)
{
    const int led = 2;
    LedsTurnOn(led);
    LedsTurnOff(led);
    TEST_ASSERT_EQUAL_HEX16(0x0000, ledsvirtuales);
}
/* Prueba para que se puedan prender y apagar múltiples LED’s. */
void test_prender_y_apagar_varios_leds (void)
{
    // Enciendo 4 leds y realizo la verificacion.  

    LedsTurnOn(1);
    LedsTurnOn(2);
    LedsTurnOn(3);
    LedsTurnOn(4);
    TEST_ASSERT_EQUAL_HEX16(0x000F, ledsvirtuales);

    // Apago los 4 leds y realizo la verificacion.  
       
    LedsTurnOff(1);
    LedsTurnOff(2);
    LedsTurnOff(3);
    LedsTurnOff(4);
    TEST_ASSERT_EQUAL_HEX16(0x0000, ledsvirtuales);

}
/* Prueba para que se puedan prender todos los LEDs de una vez.*/
void test_encender_todos_los_leds (void)
{
    LedsTurnOffall();
    LedsTurnOnall();
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, ledsvirtuales);
}
/* Prueba para que se puedan apagar todos los LEDs de una vez. */
void test_apagar_todos_los_leds (void)
{
    LedsTurnOnall();
    LedsTurnOffall();
    TEST_ASSERT_EQUAL_HEX16(0x0000, ledsvirtuales);
}
/* Prueba para que se puedan consultar el estado de un LED. */
void test_estado_leds (void)
{
    bool estado;
    int led = 2;
    LedsTurnOn(led);
    estado = LedsState(led);
    TEST_ASSERT_EQUAL_HEX16(estado,1);
    LedsTurnOff(led);
    estado = LedsState(led);
    TEST_ASSERT_EQUAL_HEX16(estado,0);
}