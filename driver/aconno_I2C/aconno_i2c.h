/*
 *  I2C library made by Jurica Resetar @ aconno
 *  2017
 *  More info @ aconno.de
 *  jurica_resetar@yahoo.com
 *  All right reserved
 *
 */

#ifndef ACONNO_I2C_H
#define ACONNO_I2C_H

#include "mbed.h"

class aconno_i2c{
    public:
        aconno_i2c(I2C *i2c, char address);
        uint8_t writeToReg(char regAddress, char *data, int len);
        uint8_t readFromReg(char regAddress, char *dataBuffer, int len);
        uint8_t sendCommand(char *command, uint8_t len, char *response,
                uint8_t responseLen, bool repeated = false);
        uint8_t sendCommand(char *command, uint8_t len);
        uint8_t readBus(char *dataBuffer, int len);
		uint16_t changeRegBits(char regAddress, uint8_t regSize,
			uint16_t newValue, uint16_t numOfBits, uint16_t offset);
    private:
        uint8_t i2cAddress;
        I2C *i2c;
};

#endif // ACONNO_I2C_H
