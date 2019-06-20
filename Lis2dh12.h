//
// Created by larox on 20.06.19.
//

#ifndef UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
#define UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H

#include "mbed.h"
#include "lis2dh12_reg.h"

class Lis2dh12 {
public:
    Lis2dh12(SPI *_spi, DigitalOut *_cs);

    virtual ~Lis2dh12();

    int32_t init();

    int32_t readAxis(float *x_axis, float *y_axis, float *z_axis);

private:
    axis3bit16_t data_raw_acceleration;
    float acceleration_mg[3];
    uint8_t whoamI;
    uint8_t tx_buffer[1000];

    SPI *spi;
    DigitalOut *cs;
    lis2dh12_ctx_t dev_ctx;

public:
    int32_t platform_read(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);
    int32_t platform_write(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);

};

#endif //UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
