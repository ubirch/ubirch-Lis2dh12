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

Lis2dh12::Lis2dh12(SPI *_spi, DigitalOut *_cs, uint16_t _thresholdInMg, uint16_t _durationInMs) :
        spi(_spi),
        cs(_cs),
        thresholdInMg(_thresholdInMg),
        durationInMs(_durationInMs)
{
    dev_ctx.write_reg = write;
    dev_ctx.read_reg = read;
    dev_ctx.handle = this;
}



Lis2dh12::~Lis2dh12() {
}

int32_t Lis2dh12::init() {
    /*
     *  Check device ID
     */
    uint8_t whoamI;
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
     * Set Output Data Rate
     */
    error = lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_10Hz);
    if (error) return error;

    /*
     * Set device in continuous mode with 10 bit resolution.
     */
    error = lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_NM_10bit);
    if (error) return error;

    /*
     * Set full scale to 2g
     */
    error = lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
    if(error) return error;

    /*
     * Set threshold in mg
     */
    error = setThreshold(thresholdInMg);
    if (error) return error;

    /*
     * Set duration in ms
     */
    error = setDuration(durationInMs);
    if (error) return error;

    /*
     * enable high event interrupts on Int1 pin (if acc > threshold for t > duration)
     */
    lis2dh12_int1_cfg_t int1Cfg = {0};
    int1Cfg.xhie = 1;
    int1Cfg.yhie = 1;
    int1Cfg.zhie = 1;
    error = lis2dh12_int1_gen_conf_set(&dev_ctx, &int1Cfg);
    if (error) return error;

    /*
     * generate interrupt for fifo overrun / interrupt activity on INT1
     */
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_ia1 = 1;
    error = lis2dh12_pin_int1_config_set(&dev_ctx, &ctrlReg3);
    if(error) return error;

    waitingForThresholdInterrupt = true;

    /*
     * latch interrupt request (read INT1_SRC (31h) to reset)
     */
    error = lis2dh12_int1_pin_notification_mode_set(&dev_ctx, LIS2DH12_INT1_LATCHED);
    if (error) return error;

    /*
     * Enable FIFO
     */
    error = lis2dh12_fifo_set(&dev_ctx, 1);
    if (error) return error;

    /*
     * select FIFO mode
     */
    error = lis2dh12_fifo_mode_set(&dev_ctx, LIS2DH12_DYNAMIC_STREAM_MODE);
    if (error) return error;

    /*
     * trigger FIFO interrupt on int1 pin
     */
    error = lis2dh12_fifo_trigger_event_set(&dev_ctx, LIS2DH12_INT1_GEN);

//    /*
//     * Read (-> clear) REFERENCE register
//     */
//    uint8_t buff;
//    error = lis2dh12_filter_reference_get(&dev_ctx, &buff);

    return error;
}

int32_t Lis2dh12::setThreshold(uint16_t userThresholdInMg) {
    uint8_t thresholdBits;
    lis2dh12_fs_t fs;

    error = lis2dh12_full_scale_get(&dev_ctx, &fs);
    if (error) return error;

    /* LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g */
    switch (fs) {
        case LIS2DH12_2g:
            thresholdBits = (uint8_t) (userThresholdInMg >> 4);
            break;
        case LIS2DH12_4g:
            thresholdBits = (uint8_t) (userThresholdInMg >> 5);
            break;
        case LIS2DH12_8g:
            thresholdBits = (uint8_t) (userThresholdInMg / 62);
            break;
        case LIS2DH12_16g:
            thresholdBits = (uint8_t) (userThresholdInMg / 186);
            break;
        default:
            thresholdBits = 0;
            break;
    }

    error = lis2dh12_int1_gen_threshold_set(&dev_ctx, thresholdBits);
    return error;
}

int32_t Lis2dh12::setDuration(uint16_t userDurationInMs) {
    uint8_t durationBits;
    lis2dh12_odr_t odr;

    error = lis2dh12_data_rate_get(&dev_ctx, &odr);
    if (error) return error;

    switch (odr) {
        case LIS2DH12_ODR_1Hz:
            durationBits = (uint8_t) (userDurationInMs / 1000);
            break;
        case LIS2DH12_ODR_10Hz:
            durationBits = (uint8_t) (userDurationInMs / 100);
            break;
        case LIS2DH12_ODR_25Hz:
            durationBits = (uint8_t) (userDurationInMs / 40);
            break;
        case LIS2DH12_ODR_50Hz:
            durationBits = (uint8_t) (userDurationInMs / 20);
            break;
        case LIS2DH12_ODR_100Hz:
            durationBits = (uint8_t) (userDurationInMs / 10);
            break;
        case LIS2DH12_ODR_200Hz:
            durationBits = (uint8_t) (userDurationInMs / 5);
            break;
        case LIS2DH12_ODR_400Hz:
            durationBits = (uint8_t) ((userDurationInMs * 2) / 5);
            break;
        default:
            durationBits = 0;
            break;
    }

    error = lis2dh12_int1_gen_duration_set(&dev_ctx, durationBits);
    return error;
}

bool Lis2dh12::selfTest() {
    wait_ms(90);    // wait for 90ms for stable output



}

int32_t Lis2dh12::checkFifoStatus() {
    uint8_t fifoDataLevel = 0;
    uint8_t fifoOverrun = 0;

    error = lis2dh12_fifo_data_level_get(&dev_ctx, &fifoDataLevel);
    if(error) return error;

    error = lis2dh12_fifo_ovr_flag_get(&dev_ctx, &fifoOverrun);
    if(error) return error;

    if (fifoOverrun) {
        EDEBUG_PRINTF("overrun. %02d unread values in FIFO | ", fifoDataLevel);
    } else {
        EDEBUG_PRINTF("No ovrn. %02d unread values in FIFO | ", fifoDataLevel);
    }

    return error;
}

/* reset latched interrupt */
int16_t Lis2dh12::resetInterrupt() {
    lis2dh12_int1_src_t int1Src = {0};

    error = lis2dh12_int1_gen_source_get(&dev_ctx, &int1Src);

    return error;
}

/* reset latched interrupt and return cause */
int16_t Lis2dh12::resetInterrupt(uint8_t *_xyzHighEvent, uint8_t *_overrun) {
    lis2dh12_int1_src_t int1Src = {0};

    error = lis2dh12_int1_gen_source_get(&dev_ctx, &int1Src);
    if (error) return error;

    *_xyzHighEvent = int1Src.ia;

    error = lis2dh12_fifo_ovr_flag_get(&dev_ctx, _overrun);

    return error;
}

/* activates interrupt when FIFO full and deactivates activity interrupt */
int16_t Lis2dh12::waitForOverrunInt() {
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_overrun = 1;
    ctrlReg3.i1_ia1 = 0;
    error = lis2dh12_pin_int1_config_set(&dev_ctx, &ctrlReg3);
    if (!error) waitingForThresholdInterrupt = false;
    EDEBUG_PRINTF("waiting for overrun interrupt...\r\n");
    return error;
}

/* deactivates interrupt when FIFO full and activates activity interrupt */
int16_t Lis2dh12::waitForThresholdInt() {
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_overrun = 0;
    ctrlReg3.i1_ia1 = 1;
    error = lis2dh12_pin_int1_config_set(&dev_ctx, &ctrlReg3);
    if (!error) waitingForThresholdInterrupt = true;
    EDEBUG_PRINTF("waiting for threshold interrupt...\r\n");
    return error;
}

int32_t Lis2dh12::getAccelerationFifo(acceleration_t *accelerationArray) {
    axis3bit16_t data_raw_acceleration;

    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));

    for (int i = 0; i < ACC_ARRAYSIZE; i++) {
        /* Read accelerometer data */
        error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        if (error) return error;

        /* following functions relate to selected full scale */
        accelerationArray[i].x_axis = convert_to_mg_fs2(data_raw_acceleration.i16bit[0]);
        accelerationArray[i].y_axis = convert_to_mg_fs2(data_raw_acceleration.i16bit[1]);
        accelerationArray[i].z_axis = convert_to_mg_fs2(data_raw_acceleration.i16bit[2]);
    }

    return error;
}

int16_t Lis2dh12::convert_to_mg_fs2(int16_t rawData) {
    return (rawData >> 4);  //TODO check if safe to shift signed int
}

int16_t Lis2dh12::convert_to_mg_fs4(int16_t rawData) {
    return (rawData >> 3);
}

void Lis2dh12::readAllRegisters(void){

	uint8_t data[10];
	for (int i = 0; i < 0x3f; ++i) {
		lis2dh12_read_reg(&dev_ctx, i, data, 1);
		EDEBUG_PRINTF("REG: %02x = %02x\r\n", i, data[0]);
	}
}
