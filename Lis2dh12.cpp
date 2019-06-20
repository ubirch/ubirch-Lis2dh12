//
// Created by larox on 20.06.19.
//

#include <edebug.h>
#include "Lis2dh12.h"




int32_t Lis2dh12::platform_read(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
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

int32_t Lis2dh12::platform_write(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
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
    Lis2dh12 *sensor = (Lis2dh12*)handle;
    sensor->platform_write(regAddr, buff, buffSize);
    return 0;
}

int32_t read(void *handle, uint8_t regAddr, uint8_t* buff, uint16_t buffSize){
    Lis2dh12 *sensor = (Lis2dh12*)handle;
    sensor->platform_read(regAddr, buff, buffSize);
    return 0;
}

Lis2dh12::Lis2dh12(SPI *_spi, DigitalOut *_cs):
        spi(_spi),
        cs(_cs)
{
    //dev_ctx = new lis2dh12_ctx_t;

    dev_ctx.write_reg = write;
    dev_ctx.read_reg = read;
    dev_ctx.handle = this;
}



Lis2dh12::~Lis2dh12() {
}

int32_t Lis2dh12::init() {
    int32_t error;

    /*
     *  Check device ID
     */
    error = lis2dh12_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2DH12_ID)
    {
        EDEBUG_PRINTF("Device not found\r\n");
        return 0xffff;
    }
    if(!error) return error;

    /*
     *  Enable Block Data Update
     */
    error = lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    if(!error) return error;

    /*
     * Set Output Data Rate to 1Hz
     */
    error = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);
    if(!error) return error;

    /*
     * Set full scale to 2g
     */
    error = lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
    if(!error) return error;

    /*
     * Set device in continuous mode with 12 bit resol.
     */
    error = lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);

    return error;
}

int32_t Lis2dh12::readAxis(float *x_axis, float *y_axis, float *z_axis){
    uint8_t ready;
    int32_t error;

    /*
     * Read output only if new value available
     */
    error = lis2dh12_xl_data_ready_get(&dev_ctx, &ready);
    if (ready)
    {
        /* Read accelerometer data */
        memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
        error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        *x_axis =
                lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[0]);
        *y_axis =
                lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[1]);
        *z_axis =
                lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[2]);
    }
    return error;
}