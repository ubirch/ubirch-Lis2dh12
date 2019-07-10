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

Lis2dh12::Lis2dh12(SPI *_spi, DigitalOut *_cs, uint16_t _thresholdInMg, uint16_t _durationInMs,
                   lis2dh12_odr_t _samplRate,
                   lis2dh12_fs_t _fullScale) :
        spi(_spi),
        cs(_cs),
        thresholdInMg(_thresholdInMg),
        durationInMs(_durationInMs),
        samplRate(_samplRate),
        fullScale(_fullScale)
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
    if (error) return error;

    if (whoamI != LIS2DH12_ID)
    {
        EDEBUG_PRINTF("Device not found\r\n");
        return 0xffff;
    }else{
        EDEBUG_PRINTF("Device ID check OK. Initialize device...\r\n");
    }

    /* High-pass filter */
    lis2dh12_ctrl_reg2_t ctrlReg2 = {0};
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG2, (uint8_t *) &ctrlReg2, 1);
    if (error) return error;

    /* Interrupt 1 enable */
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};    // do not enable any interrupts yet
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG3, (uint8_t *) &ctrlReg3, 1);
    if (error) return error;

    /* Block Data Update, Big/Little Endian data selection,
     * Full-scale selection, Operating mode selection, Self-test enable,
     * SPI serial interface mode selection */
    lis2dh12_ctrl_reg4_t ctrlReg4 = {0};
    ctrlReg4.bdu = 1;                       // Enable Block Data Update
    ctrlReg4.fs = fullScale;                // Set full scale
    ctrlReg4.hr = 0;                        // Set device in continuous mode with 10 bit resolution (normal mode)
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG4, (uint8_t *) &ctrlReg4, 1);
    if (error) return error;

    /* FIFO enable and latch interrupt request */
    lis2dh12_ctrl_reg5_t ctrlReg5 = {0};
    ctrlReg5.fifo_en = 1;                   // Enable FIFO
    ctrlReg5.lir_int1 = 1;                  // latch interrupt request (read INT1_SRC (31h) to reset)
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG5, (uint8_t *) &ctrlReg5, 1);
    if (error) return error;

    /* Interrupt 2 enable */
    lis2dh12_ctrl_reg6_t ctrlReg6 = {0};
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG6, (uint8_t *) &ctrlReg6, 1);
    if (error) return error;

    /* FIFO control register */
    lis2dh12_fifo_ctrl_reg_t fifoCtrlReg = {0};
    fifoCtrlReg.fm = LIS2DH12_DYNAMIC_STREAM_MODE;  // select FIFO mode
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_FIFO_CTRL_REG, (uint8_t *) &fifoCtrlReg, 1);
    if (error) return error;

    /* ODR, LPen, Axes enable */
    lis2dh12_ctrl_reg1_t ctrlReg1 = {0};
    ctrlReg1.odr = samplRate;               // Set Sampling Rate
    ctrlReg1.xen = 1;                       // Enable All Axes
    ctrlReg1.yen = 1;
    ctrlReg1.zen = 1;
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG1, (uint8_t *) &ctrlReg1,
                               1); // writing to CTRL_REG1 turns sensor on
    if (error) return error;

//    error = selfTest();
//    if (error) return error;

    error = enableThsInterrupt();
    if (error) return error;

//    /*
//     * Read (-> clear) REFERENCE register
//     */
//    uint8_t buff;
//    error = lis2dh12_filter_reference_get(&dev_ctx, &buff);

    return error;
}

int32_t Lis2dh12::enableThsInterrupt() {

    /* Set threshold in mg */
    error = setThreshold(thresholdInMg);
    if (error) return error;

    /* Set duration in ms */
    error = setDuration(durationInMs);
    if (error) return error;

    /* Interrupt 1 Configuration */
    lis2dh12_int1_cfg_t int1Cfg = {0};
    int1Cfg.xhie = 1;                       // enable high event interrupts on Int1 pin for all axes
    int1Cfg.yhie = 1;                       // (if acc > threshold for t > duration)
    int1Cfg.zhie = 1;
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_INT1_CFG, (uint8_t *) &int1Cfg, 1);
    if (error) return error;

    /* Interrupt 1 enable */
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_ia1 = 1;                    //generate interrupt for interrupt activity on INT1 and set flag
    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_CTRL_REG3, (uint8_t *) &ctrlReg3, 1);
    if (error) return error;
    waitingForThresholdInterrupt = true;

    return error;
}

int32_t Lis2dh12::setThreshold(uint16_t userThresholdInMg) {
    lis2dh12_int1_ths_t int1Ths = {0};
    lis2dh12_fs_t fs;

    error = lis2dh12_full_scale_get(&dev_ctx, &fs);
    if (error) return error;

    /* LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g */
    switch (fs) {
        case LIS2DH12_2g:
            int1Ths.ths = (uint8_t) (userThresholdInMg >> 4);
            EDEBUG_PRINTF("Full Scale: 2g\r\n");
            break;
        case LIS2DH12_4g:
            int1Ths.ths = (uint8_t) (userThresholdInMg >> 5);
            EDEBUG_PRINTF("Full Scale: 4g\r\n");
            break;
        case LIS2DH12_8g:
            int1Ths.ths = (uint8_t) (userThresholdInMg / 62);
            EDEBUG_PRINTF("Full Scale: 8g\r\n");
            break;
        case LIS2DH12_16g:
            int1Ths.ths = (uint8_t) (userThresholdInMg / 186);
            EDEBUG_PRINTF("Full Scale: 16g\r\n");
            break;
        default:
            EDEBUG_PRINTF("Threshold could not be set.\r\n\r\n");
            return error;
    }

    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_INT1_THS, (uint8_t *) &int1Ths, 1);
    return error;
}

int32_t Lis2dh12::setDuration(uint16_t userDurationInMs) {
    lis2dh12_int1_duration_t int1Dur = {0};
    lis2dh12_odr_t odr;

    error = lis2dh12_data_rate_get(&dev_ctx, &odr);
    if (error) return error;

    switch (odr) {
        case LIS2DH12_ODR_1Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 1000);
            EDEBUG_PRINTF("Sampling Rate: 1Hz\r\n");
            break;
        case LIS2DH12_ODR_10Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 100);
            EDEBUG_PRINTF("Sampling Rate: 10Hz\r\n");
            break;
        case LIS2DH12_ODR_25Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 40);
            EDEBUG_PRINTF("Sampling Rate: 25Hz\r\n");
            break;
        case LIS2DH12_ODR_50Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 20);
            EDEBUG_PRINTF("Sampling Rate: 50Hz\r\n");
            break;
        case LIS2DH12_ODR_100Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 10);
            EDEBUG_PRINTF("Sampling Rate: 100Hz\r\n");
            break;
        case LIS2DH12_ODR_200Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 5);
            EDEBUG_PRINTF("Sampling Rate: 200Hz\r\n");
            break;
        case LIS2DH12_ODR_400Hz:
            int1Dur.d = (uint8_t) ((userDurationInMs * 2) / 5);
            EDEBUG_PRINTF("Sampling Rate: 400Hz\r\n");
            break;
        default:
            EDEBUG_PRINTF("Duration could not be set.\r\n\r\n");
            return error;
    }

    error = lis2dh12_write_reg(&dev_ctx, LIS2DH12_INT1_DURATION, (uint8_t *) &int1Dur, 1);
    return error;
}

int32_t Lis2dh12::selfTest() {
    uint8_t dataReady = 0;
    uint8_t dataLevel = 0;
    axis3bit16_t data_raw_acceleration;
    acceleration_t selfTestArray[TEST_ARRAYSIZE];
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    acceleration_t firstAverage = {0};
    acceleration_t selfTestAverage = {0};
    acceleration_t selfTestAbsMin = {0};
    acceleration_t selfTestAbsMax = {0};

    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));

    /* wait for available data */
    do {
        wait_ms(100);
        error = lis2dh12_xl_data_ready_get(&dev_ctx, &dataReady);
        if (error) return error;
    } while (dataReady == 0);

    /* read first available data and discard */
    error = lis2dh12_fifo_data_level_get(&dev_ctx, &dataLevel);
    if (error) return error;
    EDEBUG_PRINTF("%d unread values in FIFO -> discard\r\n", dataLevel);

    for (int i = 0; i < dataLevel; i++) {
        error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        if (error) return error;
    }

    /* wait until new measurements in fifo */
    do {
        wait_ms(100);
        error = lis2dh12_fifo_data_level_get(&dev_ctx, &dataLevel);
        if (error) return error;
        EDEBUG_PRINTF("%d unread values in FIFO\r\n", dataLevel);
    } while (dataLevel < TEST_ARRAYSIZE);

    /* read new values and save average of each axis */
    for (int i = 0; i < TEST_ARRAYSIZE; i++) {
        error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        if (error) return error;

        selfTestArray[i].x_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[0]);
        selfTestArray[i].y_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[1]);
        selfTestArray[i].z_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[2]);

        x_sum += selfTestArray[i].x_axis;
        y_sum += selfTestArray[i].y_axis;
        z_sum += selfTestArray[i].z_axis;
    }

    /* average values */
    firstAverage.x_axis = x_sum / TEST_ARRAYSIZE;
    firstAverage.y_axis = y_sum / TEST_ARRAYSIZE;
    firstAverage.z_axis = z_sum / TEST_ARRAYSIZE;

    x_sum = 0;
    y_sum = 0;
    z_sum = 0;

    /* enable selftest */
    error = lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_POSITIVE);
    if (error) return error;

    /* wait for available data */
    do {
        wait_ms(90);
        error = lis2dh12_xl_data_ready_get(&dev_ctx, &dataReady);
        if (error) return error;
    } while (dataReady == 0);

    /* read first available data and discard */
    error = lis2dh12_fifo_data_level_get(&dev_ctx, &dataLevel);
    if (error) return error;
    EDEBUG_PRINTF("%d unread values in FIFO -> discard\r\n", dataLevel);

    for (int i = 0; i < dataLevel; i++) {
        error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        if (error) return error;
    }

    /* wait until new measurements in fifo */
    do {
        wait_ms(100);
        error = lis2dh12_fifo_data_level_get(&dev_ctx, &dataLevel);
        if (error) return error;
        EDEBUG_PRINTF("%d unread values in FIFO\r\n", dataLevel);
    } while (dataLevel < TEST_ARRAYSIZE);

    /* read new values */
    for (int i = 0; i < TEST_ARRAYSIZE; i++) {
        error = lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
        if (error) return error;

        selfTestArray[i].x_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[0]);
        selfTestArray[i].y_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[1]);
        selfTestArray[i].z_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[2]);

        x_sum += selfTestArray[i].x_axis;
        y_sum += selfTestArray[i].y_axis;
        z_sum += selfTestArray[i].z_axis;

        if (i == 0) {
            selfTestAbsMin.x_axis = abs(selfTestArray[i].x_axis);
            selfTestAbsMin.y_axis = abs(selfTestArray[i].y_axis);
            selfTestAbsMin.z_axis = abs(selfTestArray[i].z_axis);

            selfTestAbsMax.x_axis = abs(selfTestArray[i].x_axis);
            selfTestAbsMax.y_axis = abs(selfTestArray[i].y_axis);
            selfTestAbsMax.z_axis = abs(selfTestArray[i].z_axis);
        } else {

            /* find MIN */
            if (selfTestAbsMin.x_axis > abs(selfTestArray[i].x_axis)) {
                selfTestAbsMin.x_axis = abs(selfTestArray[i].x_axis);
            }

            if (selfTestAbsMin.y_axis > abs(selfTestArray[i].y_axis)) {
                selfTestAbsMin.y_axis = abs(selfTestArray[i].y_axis);
            }

            if (selfTestAbsMin.z_axis > abs(selfTestArray[i].z_axis)) {
                selfTestAbsMin.z_axis = abs(selfTestArray[i].z_axis);
            }

            /* find MAX */
            if (selfTestAbsMax.x_axis < abs(selfTestArray[i].x_axis)) {
                selfTestAbsMax.x_axis = abs(selfTestArray[i].x_axis);
            }

            if (selfTestAbsMax.y_axis < abs(selfTestArray[i].y_axis)) {
                selfTestAbsMax.y_axis = abs(selfTestArray[i].y_axis);
            }

            if (selfTestAbsMax.z_axis < abs(selfTestArray[i].z_axis)) {
                selfTestAbsMax.z_axis = abs(selfTestArray[i].z_axis);
            }
        }
    }

    /* average values */
    selfTestAverage.x_axis = x_sum / TEST_ARRAYSIZE;
    selfTestAverage.y_axis = y_sum / TEST_ARRAYSIZE;
    selfTestAverage.z_axis = z_sum / TEST_ARRAYSIZE;

    /* disable selftest */
    error = lis2dh12_self_test_set(&dev_ctx, LIS2DH12_ST_DISABLE);
    if (error) return error;

    acceleration_t absDif = {0};

    absDif.x_axis = abs(selfTestAverage.x_axis - firstAverage.x_axis);
    absDif.y_axis = abs(selfTestAverage.y_axis - firstAverage.y_axis);
    absDif.z_axis = abs(selfTestAverage.z_axis - firstAverage.z_axis);

    EDEBUG_PRINTF("selfTestAverage.x_axis (%d) - firstAverage.x_axis (%d) \r\n", selfTestAverage.x_axis,
                  firstAverage.x_axis);
    EDEBUG_PRINTF("Check: STx-min %d < %d < STx-max %d \r\n", selfTestAbsMin.x_axis, absDif.x_axis,
                  selfTestAbsMax.x_axis);

    /* finally, check if passed */
    if ((selfTestAbsMin.x_axis <= absDif.x_axis) && (absDif.x_axis <= selfTestAbsMax.x_axis)
        && (selfTestAbsMin.y_axis <= absDif.y_axis) && (absDif.y_axis <= selfTestAbsMax.y_axis)
        && (selfTestAbsMin.z_axis <= absDif.z_axis) && (absDif.z_axis <= selfTestAbsMax.z_axis)) {
        EDEBUG_PRINTF("SELF TEST PASSED\r\n\r\n");
        return 0;
    } else {
        EDEBUG_PRINTF("SELF TEST FAILED\r\n\r\n");
        return 0xffff;
    }
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
//    EDEBUG_PRINTF("waiting for overrun interrupt...\r\n");
    return error;
}

/* deactivates interrupt when FIFO full and activates activity interrupt */
int16_t Lis2dh12::waitForThresholdInt() {
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_overrun = 0;
    ctrlReg3.i1_ia1 = 1;
    error = lis2dh12_pin_int1_config_set(&dev_ctx, &ctrlReg3);
    if (!error) waitingForThresholdInterrupt = true;
//    EDEBUG_PRINTF("waiting for threshold interrupt...\r\n");
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
        accelerationArray[i].x_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[0]);
        accelerationArray[i].y_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[1]);
        accelerationArray[i].z_axis = convert_to_mg_fs8(data_raw_acceleration.i16bit[2]);
    }

    return error;
}

int16_t Lis2dh12::convert_to_mg_fs2(int16_t rawData) {
    return (rawData >> 4);  //TODO check if safe to shift signed int
}

int16_t Lis2dh12::convert_to_mg_fs4(int16_t rawData) {
    return (rawData >> 3);
}

int16_t Lis2dh12::convert_to_mg_fs8(int16_t rawData) {
    return (rawData >> 2);
}

void Lis2dh12::readAllRegisters(void){

	uint8_t data[10];
    for (int i = 0x1E; i <= 0x33; ++i) {
		lis2dh12_read_reg(&dev_ctx, i, data, 1);
        EDEBUG_PRINTF("REG:%02x = %02x\r\n", i, data[0]);
	}
}
