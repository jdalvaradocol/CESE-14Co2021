/*
 * i2c.h
 *
 *  Created on: 16/06/2021
 *      Author: jdalvarado
 */

#ifndef i2c_H_
#define i2c_H_

#include <stdint.h>
#include <stdbool.h>

bool i2cInit( uint8_t i2cNumber, uint32_t clockRateHz );

bool i2cRead(   uint8_t     i2cNumber,
                uint8_t     i2cSlaveAddress,
                uint8_t     receiveDataBuffer,
                uint16_t    receiveDataBufferSize,
                bool        sendReadStop );

bool i2cWrite(  uint8_t     i2cNumber,
                uint8_t     i2cSlaveAddress,
                uint8_t     transmitDataBuffer,
                uint16_t    transmitDataBufferSize,
                bool        sendWriteStop );

#endif 