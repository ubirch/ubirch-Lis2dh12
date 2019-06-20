//
// Created by larox on 20.06.19.
//

#include "lis2dh12.h"



 int32_t FOO::platform_read(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
	// Most significant bit represents read from register, bit after it
	// represents address increment if multiple read, which is enabled.
	const uint8_t spiSetup = 0xC0;

	uint8_t retVal = 0;
	*cs = 0;

	spi->write(spiSetup | regAddr);

	while(buffSize--)
	{
		*buff = spi->write(0x00);
		buff++;
	}

	*cs = 1;
	return retVal;
}

int32_t FOO::platform_write(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
	// Most significant bit represents write from register, bit after it
	// represents address increment if multiple write, which is enabled.
	const uint8_t spiSetup = 0x40;

	uint8_t retVal = 0;

	*cs = 0;

	spi->write(spiSetup | regAddr);

	while(buffSize--)
	{
		spi->write(*buff);
		buff++;
	}

	*cs = 1;
	return retVal;
}

int32_t write(void *handle, uint8_t regAddr, uint8_t* buff, uint16_t buffSize){
	FOO *sensor = (FOO*)handle;
	sensor->platform_write(regAddr, buff, buffSize);
	return 0;
}

int32_t read(void *handle, uint8_t regAddr, uint8_t* buff, uint16_t buffSize){
	FOO *sensor = (FOO*)handle;
	sensor->platform_read(regAddr, buff, buffSize);
	return 0;
}

FOO::FOO(SPI *_spi, DigitalOut *_cs):
	spi(_spi),
	cs(_cs)
{
	dev_ctx = new lis2dh12_ctx_t;

	dev_ctx->write_reg = write;
	dev_ctx->read_reg = read;
	dev_ctx->handle = this;
}

