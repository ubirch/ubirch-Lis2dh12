//
// Created by larox on 20.06.19.
//

#ifndef UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
#define UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H

#include "mbed.h"
#include "lis2dh12_reg.h"

#define ACC_ARRAYSIZE 32    //size of FIFO

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
}acceleration_t;

class Lis2dh12 {
public:
    Lis2dh12(SPI *_spi, DigitalOut *_cs, uint16_t _thresholdInMg, uint16_t _durationInMs);

    virtual ~Lis2dh12();

    int32_t init();

    int32_t readAxis(acceleration_t& acceleration);

	void readAllRegisters(void);

    int32_t checkFifoStatus();

    uint8_t pollFifoOverrun();

    int32_t platform_read(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);

    int32_t platform_write(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);

private:
    int32_t setDuration(uint16_t userDurationInMs);

    int32_t setThreshold(uint16_t userThresholdInMg);

    int16_t convert_to_mg_fs2(int16_t rawData);

    int16_t convert_to_mg_fs4(int16_t rawData);

    uint8_t tx_buffer[1000];

    SPI *spi;
    DigitalOut *cs;
    lis2dh12_ctx_t dev_ctx;

    uint16_t thresholdInMg;
    uint16_t durationInMs;

};

#endif //UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
