/*
 *  I2C library made by Jurica Resetar @ aconno
 *  2017
 *  More info @ aconno.de
 *  jurica_resetar@yahoo.com
 *  All right reserved
 *
 */

#include "aconno_i2c.h"

/**
 * [aconno_i2c::aconno_i2c description]
 * @param i2c     [description]
 * @param address [description]
 */
aconno_i2c::aconno_i2c(I2C *i2c, char address){
    this->i2c = i2c;
    i2cAddress = address;
}

/**
 * Method for changing specific bits within register
 * @param  regAddress: 8bit address
 * @param  regSize:	 register size in B
 * @param  newValue  new value for [numOfBits] bits within the register
 * @param  numOfBits  sizeof[newValue] in b
 * @param  offset     newValue offset within the register
 * @return            returns new value of the register
 */
uint16_t aconno_i2c::changeRegBits(char regAddress, uint8_t regSize,
	uint16_t newValue, uint16_t numOfBits, uint16_t offset)
{
	uint16_t mask;
    uint16_t regData = 0;
	// Read old configuration
	readFromReg(regAddress, (char*)&regData, regSize);
	mask = 0 | newValue << offset;
	// Clear bits
	regData &= ~(((1 << numOfBits)-1) << offset);
	regData |= mask; // Set/clear bits as required
    writeToReg(regAddress, (char*)&regData, regSize);
    return regData;
}

/**
 * [aconno_i2c::writeToReg description]
 * @param  regAddress [description]
 * @param  data       [description]
 * @param  len        [description]
 * @return            [description]
 */
uint8_t aconno_i2c::writeToReg(char regAddress, char *data, int len){
    uint8_t success; /* 0 on success (ack), non-0 on failure (nack) */
    char dataToSend[len+1];

    dataToSend[0] = regAddress;
	memcpy(dataToSend+1, data, len);
	// R/W bit is set low for a write command
    success = i2c->write(i2cAddress & 0xFE, dataToSend, len + 1);
    return success;
}

/**
 * [aconno_i2c::readFromReg description]
 * @param  regAddress [description]
 * @param  dataBuffer [description]
 * @param  len        [description]
 * @return            [description]
 */
uint8_t aconno_i2c::readFromReg(char regAddress, char *dataBuffer, int len){
    uint8_t success;    /* 0 on success (ack), non-0 on failure (nack) */
    char regAddr = regAddress;

    i2c->write(i2cAddress & 0xFE, &regAddr, 1);               // R/W bit is set low for a write command
    success = i2c->read(i2cAddress | 0x01, dataBuffer, len);     // R/W bit is set high for a read command
    return success;
}

/**
 * This method is used with sendCommand(char *command, uint8_t len) method when delay between command
 * and response from the slave is required
 * @param  dataBuffer [description]
 * @param  len        [description]
 * @return            [description]
 */
uint8_t aconno_i2c::readBus(char *dataBuffer, int len){
    uint8_t success;    /* 0 on success (ack), non-0 on failure (nack) */

    success = i2c->read(i2cAddress | 0x01, dataBuffer, len);     // R/W bit is set high for a read command
    return success;
}

/**
 *  This method is used to send commands to I2C Device.
 *  Ex. send 2B command to Si7006 to get Device ID.
 * @param  command     [description]
 * @param  len         [description]
 * @param  response    [description]
 * @param  responseLen [description]
 * @param  repeated    [description]
 * @return 0 on success (ack), non-0 on failure (nack)
 */
uint8_t aconno_i2c::sendCommand(char *command, uint8_t len, char *response,
    uint8_t responseLen, bool repeated)
    {
    uint8_t success;    /* 0 on success (ack), non-0 on failure (nack) */

    // R/W bit is set low for a write command
    i2c->write(i2cAddress & 0xFE, command, len, repeated);
    // R/W bit is set high for a read command
    success = i2c->read(i2cAddress | 0x01, response, responseLen);
    return success;
}

/**
 * [aconno_i2c::sendCommand description]
 * @param  command [description]
 * @param  len     [description]
 * @return         [description]
 */
uint8_t aconno_i2c::sendCommand(char *command, uint8_t len){
    uint8_t success;    /* 0 on success (ack), non-0 on failure (nack) */

    success = i2c->write(i2cAddress & 0xFE, command, len);
    return success;
}
