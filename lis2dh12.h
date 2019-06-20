//
// Created by larox on 20.06.19.
//

#ifndef UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
#define UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H

#include "mbed.h"
#include "lis2dh12_reg.h"


class FOO {
public:
	FOO(SPI *_spi, DigitalOut *_cs);
	void attach(Callback<lis2dh12_write_ptr> cb);

private:
	SPI *spi;
	DigitalOut *cs;
	lis2dh12_ctx_t *dev_ctx;

public:
	int32_t platform_read(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);
	int32_t platform_write(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);
};


#endif //UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
