

_Bool 

    i2cInit( uint8_t i2cNumber, uint32_t clockRateHz );





_Bool 

    i2cRead( uint8_t i2cNumber,

                uint8_t i2cSlaveAddress,

                uint8_t receiveDataBuffer,

                uint16_t receiveDataBufferSize,

                

               _Bool 

                           sendReadStop );





_Bool 

    i2cWrite( uint8_t i2cNumber,

                uint8_t i2cSlaveAddress,

                uint8_t transmitDataBuffer,

                uint16_t transmitDataBufferSize,

                

               _Bool 

                           sendWriteStop );
