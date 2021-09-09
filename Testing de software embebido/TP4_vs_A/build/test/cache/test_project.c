#include "src/BMP180.h"
#include "src/LCD.h"
#include "src/sapi.h"
#include "build/test/mocks/mock_i2c.h"
#include "/var/lib/gems/2.7.0/gems/ceedling-0.31.1/vendor/unity/src/unity.h"




uint16_t ledsvirtuales;



void setUp (void)

{

    LedsInit(&ledsvirtuales);

}

void tearDown (void)

{



}



void test_init_LCD(void)

{

    

   _Bool 

        valid = 0;

    valid = LCD_Init();

    do {if ((valid)) {} else {UnityFail( ((" Expected TRUE Was FALSE")), (UNITY_UINT)((UNITY_UINT)(23)));}} while(0);

}



void test_init_LCD_SendString(void)

{

    

   _Bool 

           valid = 0;

    char Buffer[20];

    int Temperatura = 50;



    sprintf(Buffer, "Temperatura = %2d",Temperatura);

    valid = LCD_SendString(0,0,Buffer);

    do {if ((valid)) {} else {UnityFail( ((" Expected TRUE Was FALSE")), (UNITY_UINT)((UNITY_UINT)(34)));}} while(0);

}



void test_init_BMP180(void)

{

    

   _Bool 

        valid = 0;

    uint8_t reg = 0x00, value = 0x00;



    i2cWrite_CMockExpectAndReturn(42, 0, 0x77, reg, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(43, 0, 0x77, value, 1, 1, 

   1

   );



    for ( reg = BMP180_CAL_AC1_REG; reg <= BMP180_CAL_MD_REG; reg+=2)

    {

        i2cWrite_CMockExpectAndReturn(47, 0, 0x77, reg, 1, 0, 

       1

       );

        i2cRead_CMockExpectAndReturn(48, 0, 0x77, value, 1, 1, 

       1

       );

        i2cWrite_CMockExpectAndReturn(49, 0, 0x77, reg+1, 1, 0, 

       1

       );

        i2cRead_CMockExpectAndReturn(50, 0, 0x77, value, 1, 1, 

       1

       );

    }



    valid = BMP180_init(0x77);

    do {if ((valid)) {} else {UnityFail( ((" Expected TRUE Was FALSE")), (UNITY_UINT)((UNITY_UINT)(54)));}} while(0);

}



void test_init_BMP180_altitude(void)

{

    int32_t altitude = 0;

    uint8_t reg = 0x00, value = 0x00;





    i2cWrite_CMockExpectAndReturn(63, 0, 0x77, 0xF4, 1, 0, 

   1

   );

    i2cWrite_CMockExpectAndReturn(64, 0, 0x77, 0x2E, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(67, 0, 0x77, 0xF6, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(68, 0, 0x77, value, 1, 1, 

   1

   );

    i2cRead_CMockExpectAndReturn(69, 0, 0x77, value, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(72, 0, 0x77, 0xF4, 1, 0, 

   1

   );

    i2cWrite_CMockExpectAndReturn(73, 0, 0x77, 0x34, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(76, 0, 0x77, 0xF6, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(77, 0, 0x77, value, 1, 1, 

   1

   );

    i2cRead_CMockExpectAndReturn(78, 0, 0x77, value, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(81, 0, 0x77, 0xF8, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(82, 0, 0x77, value, 1, 1, 

   1

   );



    altitude = BMP180_altitude(0x77);



    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT32)((44330)), (UNITY_INT)(UNITY_INT32)((altitude)), (

   ((void *)0)

   ), (UNITY_UINT)(86), UNITY_DISPLAY_STYLE_INT32);

}



void test_init_BMP180_getPressure(void)

{

    int32_t altitude = 0;

    uint8_t reg = 0x00, value = 0x00;





    i2cWrite_CMockExpectAndReturn(95, 0, 0x77, 0xF4, 1, 0, 

   1

   );

    i2cWrite_CMockExpectAndReturn(96, 0, 0x77, 0x2E, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(99, 0, 0x77, 0xF6, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(100, 0, 0x77, value, 1, 1, 

   1

   );

    i2cRead_CMockExpectAndReturn(101, 0, 0x77, value, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(104, 0, 0x77, 0xF4, 1, 0, 

   1

   );

    i2cWrite_CMockExpectAndReturn(105, 0, 0x77, 0x34, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(108, 0, 0x77, 0xF6, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(109, 0, 0x77, value, 1, 1, 

   1

   );

    i2cRead_CMockExpectAndReturn(110, 0, 0x77, value, 1, 1, 

   1

   );





    i2cWrite_CMockExpectAndReturn(113, 0, 0x77, 0xF8, 1, 0, 

   1

   );

    i2cRead_CMockExpectAndReturn(114, 0, 0x77, value, 1, 1, 

   1

   );



    altitude = BMP180_getPressure(0x77);



    UnityAssertEqualNumber((UNITY_INT)(UNITY_INT32)((255)), (UNITY_INT)(UNITY_INT32)((altitude)), (

   ((void *)0)

   ), (UNITY_UINT)(118), UNITY_DISPLAY_STYLE_INT32);

}
