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
    }else{
        EDEBUG_PRINTF("Device ID check OK. Initialize device...\r\n");
    }
    if(error) return error;

    /*
     *  Enable Block Data Update
     */
    error = lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    if(error) return error;

    /*
     * Set full scale to 2g
     */
    error = lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
    if(error) return error;

    /*
     * Enable FIFO
     */
    error = lis2dh12_fifo_set(&dev_ctx, 1);
    if(error) return error;

    /*
     * select FIFO mode
     */
    error = lis2dh12_fifo_mode_set(&dev_ctx, LIS2DH12_DYNAMIC_STREAM_MODE);
    if(error) return error;

    /*
     * set FIFO watermark
     */
    uint8_t fifoWtm = 1;
    error = lis2dh12_fifo_watermark_set(&dev_ctx, fifoWtm);
    if(error) return error;


    /*
     * trigger interrupt on int1 pin
     */
    error = lis2dh12_fifo_trigger_event_set(&dev_ctx, LIS2DH12_INT1_GEN);
    if(error) return error;

    /*
     * generate interrupt for fifo overrun / watermark
     */
    lis2dh12_ctrl_reg3_t ctrlReg3;
    ctrlReg3.i1_overrun = 1;
    ctrlReg3.i1_wtm =1;         //TODO take this line out, just for testing
    error = lis2dh12_pin_int1_config_set(&dev_ctx, &ctrlReg3);
    if(error) return error;


    /*
     * latch interrupt request (read INT1_SRC (31h) to reset)
     */
    error = lis2dh12_int1_pin_notification_mode_set(&dev_ctx, LIS2DH12_INT1_LATCHED);
    if(error) return error;


    /*
     * Set device in continuous mode with 10 bit resolution.
     */
    error = lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_NM_10bit);
    if(error) return error;

    /*
     * Set Output Data Rate to 1Hz
     */
    error = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_1Hz);
    if(error) return error;

    /*
     * Read (-> clear) REFERENCE register
     */
    uint8_t buff;
    error = lis2dh12_filter_reference_get(&dev_ctx, &buff);
//    if(error) return error;
//
//
//
//    while(true){
//        error = checkFifoStatus();
//        if(error) return error;
//        wait_ms(1000);
//    }

    return error;
}

int32_t Lis2dh12::checkFifoStatus() {
    int32_t error;
    uint8_t fifoDataLevel = 0;
    uint8_t fifoOverrun = 0;
    acceleration_t acceleration;

    error = lis2dh12_fifo_data_level_get(&dev_ctx, &fifoDataLevel);
    if(error) return error;

    error = lis2dh12_fifo_ovr_flag_get(&dev_ctx, &fifoOverrun);
    if(error) return error;

    if (fifoOverrun) {
        EDEBUG_PRINTF("FIFO OVERRUN\r\n");
        lis2dh12_int1_src_t int1Src;
        error = lis2dh12_int1_gen_source_get(&dev_ctx, &int1Src);       // clearing latched interrupt here
        if(int1Src.ia) {
            EDEBUG_PRINTF("INT1_SRC: IA = 1 -> interrupt generated \r\n");
        } else {
            EDEBUG_PRINTF("INT1_SRC: IA = 0 -> no interrupt generated \r\n");
        }

        for (int i = 0; i < 32; i++) {
            error = readAxis(acceleration);
            if(!error) {
                EDEBUG_PRINTF("%d.) X: %d | Y: %d | Z: %d | ", i, acceleration.x_axis, acceleration.y_axis, acceleration.z_axis);
            } else return error;

            error = lis2dh12_fifo_data_level_get(&dev_ctx, &fifoDataLevel);
            if(!error) {
                EDEBUG_PRINTF("(%d unread values in FIFO) \r\n", fifoDataLevel);
            } else return error;
        }

    } else {
        EDEBUG_PRINTF("No overrun. %d unread values in FIFO\r\n", fifoDataLevel);
    }

    return error;
}

int32_t Lis2dh12::readAxis(acceleration_t& acceleration) {
    int32_t error;

    /* Read accelerometer data */
    memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
    error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
    acceleration.x_axis = (int32_t) (lis2dh12_from_fs2_nm_to_mg(data_raw_acceleration.i16bit[0]));            //functions relate to full scale and resolution
    acceleration.y_axis = (int32_t) (lis2dh12_from_fs2_nm_to_mg(data_raw_acceleration.i16bit[1]));
    acceleration.z_axis = (int32_t) (lis2dh12_from_fs2_nm_to_mg(data_raw_acceleration.i16bit[2]));

    return error;
}