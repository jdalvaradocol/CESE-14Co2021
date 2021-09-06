#include "src/Leds.h"
#include "/var/lib/gems/2.7.0/gems/ceedling-0.31.1/vendor/unity/src/unity.h"




uint16_t ledsvirtuales;



void setUp (void)

{

    LedsInit(&ledsvirtuales);

}

void tearDown (void)

{



}



void test_LedsOffAfterCreate (void)

{

    uint16_t ledsvirtuales = 0xFFFF;

    LedsInit(&ledsvirtuales);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((0x0000)), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(20), UNITY_DISPLAY_STYLE_HEX16);

}



void test_prender_un_led (void)

{

    const int led = 2;

    LedsTurnOn(led);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((1 << (led-1))), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(27), UNITY_DISPLAY_STYLE_HEX16);

}



void test_apagar_un_led (void)

{

    const int led = 2;

    LedsTurnOn(led);

    LedsTurnOff(led);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((0x0000)), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(35), UNITY_DISPLAY_STYLE_HEX16);

}



void test_prender_y_apagar_varios_leds (void)

{





    LedsTurnOn(1);

    LedsTurnOn(2);

    LedsTurnOn(3);

    LedsTurnOn(4);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((0x000F)), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(46), UNITY_DISPLAY_STYLE_HEX16);







    LedsTurnOff(1);

    LedsTurnOff(2);

    LedsTurnOff(3);

    LedsTurnOff(4);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((0x0000)), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(54), UNITY_DISPLAY_STYLE_HEX16);



}



void test_encender_todos_los_leds (void)

{

    LedsTurnOffall();

    LedsTurnOnall();

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((0xFFFF)), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(62), UNITY_DISPLAY_STYLE_HEX16);

}



void test_apagar_todos_los_leds (void)

{

    LedsTurnOnall();

    LedsTurnOffall();

    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT16)((0x0000)), (UNITY_INT)(UNITY_INT16)((ledsvirtuales)), (

   ((void *)0)

   ), (UNITY_UINT)(69), UNITY_DISPLAY_STYLE_HEX16);

}



void test_estado_leds (void)

{

    

   _Bool 

        estado;

    int led = 2;

    LedsTurnOn(led);

    estado = LedsState(led);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_UINT8 )((estado)), (UNITY_INT)(UNITY_UINT8 )((1)), (

   ((void *)0)

   ), (UNITY_UINT)(78), UNITY_DISPLAY_STYLE_UINT8);



    LedsTurnOff(led);

    estado = LedsState(led);

    UnityAssertEqualNumber((UNITY_INT)(UNITY_UINT8 )((estado)), (UNITY_INT)(UNITY_UINT8 )((0)), (

   ((void *)0)

   ), (UNITY_UINT)(82), UNITY_DISPLAY_STYLE_UINT8);



}
