/*!
 * @file
 * @brief Lis2dh12 Accelerometer
 *
 * @author Roxana Meixner
 * @date   2019-06-20
 *
 * @copyright &copy; 2017 ubirch GmbH (https://ubirch.com)
 *
 * ```
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ```
 */

#include <edebug.h>
#include "Lis2dh12.h"
#include "aconno_i2c.h"




int32_t Lis2dh12::readFromReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
    return i2c.readFromReg((char)regAddr, (char*)buff, buffSize);
}

int32_t Lis2dh12::writeToReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
    return i2c.writeToReg((char)regAddr, (char*)buff, buffSize);
}

Lis2dh12::Lis2dh12(I2C *_i2c,
                   uint16_t _thresholdInMg, uint16_t _durationInMs,
                   lis2dh12_odr_t _samplRate, lis2dh12_fs_t _fullScale) :
        i2c(_i2c, LIS2DH12_I2C_ADD_H),
        thresholdInMg(_thresholdInMg),
        durationInMs(_durationInMs),
        samplRate(_samplRate),
        fullScale(_fullScale)
{
    waitingForThresholdInterrupt = false;
}



Lis2dh12::~Lis2dh12() {
}

int32_t Lis2dh12::init() {
    /*
     *  Check sensor ID
     */
    uint8_t whoamI;
    error = readFromReg(LIS2DH12_WHO_AM_I, &whoamI, 1);
    if (error)
    {
        EDEBUG_PRINTF("reading from WHOAMI REG failed!\r\n");
        return error;
    }

    if (whoamI != LIS2DH12_ID)
    {
        EDEBUG_PRINTF("Sensor ID check failed! (Expected ID: 0x%02x - Got: 0x%02x)\r\n", LIS2DH12_ID, whoamI);
        return 0xffff;
    }else{
        EDEBUG_PRINTF("Sensor ID check OK\r\n");
    }

    readAllRegisters();

    lis2dh12_ctrl_reg0_t ctrlReg0 = {0};
    ctrlReg0.sdo_pu_disc = 1;    // pull-up disconnected to SDO/SA0 pin
    error = writeToReg(LIS2DH12_CTRL_REG0, (uint8_t *) &ctrlReg0, 1);
    if (error)
    {
        EDEBUG_PRINTF("writing to CTRL REG 0 failed!\r\n");
        return error;
    }

    /* High-pass filter */
    lis2dh12_ctrl_reg2_t ctrlReg2 = {0};    // bypass high-pass filter
    error =  writeToReg(LIS2DH12_CTRL_REG2, (uint8_t *) &ctrlReg2, 1);
    if (error) return error;

    /* Interrupt 1 enable */
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};    // do not enable any interrupts on interrupt 1 pin yet
    error =  writeToReg(LIS2DH12_CTRL_REG3, (uint8_t *) &ctrlReg3, 1);
    if (error) return error;

    /* Interrupt 2 enable */
    lis2dh12_ctrl_reg6_t ctrlReg6 = {0};    // do not enable any interrupt on interrupt 2 pin
    error =  writeToReg(LIS2DH12_CTRL_REG6, (uint8_t *) &ctrlReg6, 1);
    if (error) return error;

    /* Interrupt 1 Configuration */
    lis2dh12_int1_cfg_t int1Cfg = {0};    // do not enable any interrupts yet
    error =  writeToReg(LIS2DH12_INT1_CFG, (uint8_t *) &int1Cfg, 1);
    if (error) return error;

    /* Block Data Update, Big/Little Endian data selection,
     * Full-scale selection, Operating mode selection, Self-test enable,
     * SPI serial interface mode selection */
    lis2dh12_ctrl_reg4_t ctrlReg4 = {0};
    ctrlReg4.bdu = 1;                       // Enable Block Data Update
    ctrlReg4.fs = fullScale;                // Set full scale
    ctrlReg4.hr = 0;                        // Set device to normal / low power mode
    error =  writeToReg(LIS2DH12_CTRL_REG4, (uint8_t *) &ctrlReg4, 1);
    if (error) return error;

    /* FIFO enable and latch interrupt request */
    lis2dh12_ctrl_reg5_t ctrlReg5 = {0};
    ctrlReg5.fifo_en = 1;                   // Enable FIFO
    ctrlReg5.lir_int1 = 1;                  // latch interrupt request (read INT1_SRC (31h) to reset)
    error =  writeToReg(LIS2DH12_CTRL_REG5, (uint8_t *) &ctrlReg5, 1);
    if (error) return error;

    /* FIFO control register */
    lis2dh12_fifo_ctrl_reg_t fifoCtrlReg = {0};
    fifoCtrlReg.fm = LIS2DH12_DYNAMIC_STREAM_MODE;  // select FIFO mode
    error =  writeToReg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *) &fifoCtrlReg, 1);
    if (error) return error;

    error = activateSensor();
    if (error) return error;

    error = selfTest();
    if (error) return error;

    error = enableThsInterrupt();
    if (error) return error;

    return error;
}

int32_t Lis2dh12::activateSensor() {
    /* ODR, LPen, Axes enable */
    lis2dh12_ctrl_reg1_t ctrlReg1 = {0};
    ctrlReg1.odr = samplRate;               // Set sampling rate
    ctrlReg1.lpen = NORMAL_10bit;           // Set normal mode (10 bit resolution) or low power mode (8 bit resolution)
    ctrlReg1.xen = 1;                       // Enable all axes
    ctrlReg1.yen = 1;
    ctrlReg1.zen = 1;
    error =  writeToReg(LIS2DH12_CTRL_REG1, (uint8_t *) &ctrlReg1, 1); // turn on sensor
    return error;
}

int32_t Lis2dh12::powerDown() {
    lis2dh12_ctrl_reg1_t ctrlReg1 = {0};    // Disable all axes and set sampling rate to 0, SPI stays active
    error =  writeToReg(LIS2DH12_CTRL_REG1, (uint8_t *) &ctrlReg1, 1);
    return error;
}

int32_t Lis2dh12::enableThsInterrupt() {
    /* clear interrupts first */
    uint8_t data;
    readFromReg(LIS2DH12_INT1_SRC, &data, 1);

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
    error =  writeToReg(LIS2DH12_INT1_CFG, (uint8_t *) &int1Cfg, 1);
    if (error) return error;

    /* Interrupt 1 enable */
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_ia1 = 1;                    // generate interrupt for interrupt activity on INT1 and set flag
    error =  writeToReg(LIS2DH12_CTRL_REG3, (uint8_t *) &ctrlReg3, 1);
    if (error) return error;
    waitingForThresholdInterrupt = true;

    return error;
}

int32_t Lis2dh12::setThreshold(uint16_t userThresholdInMg) {
    lis2dh12_int1_ths_t int1Ths = {0};
    lis2dh12_fs_t fs;
    lis2dh12_ctrl_reg4_t ctrl_reg4;

    error = readFromReg(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (error) return error;

    fs = static_cast<lis2dh12_fs_t>(ctrl_reg4.fs);

    /* LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g */
    switch (fs) {
        case LIS2DH12_2g:
            int1Ths.ths = (uint8_t) (userThresholdInMg >> 4);
            EDEBUG_PRINTF("Full Scale: 2 g\r\n");
            EDEBUG_PRINTF("Threshold: %d mg\r\n", int1Ths.ths << 4);
            break;
        case LIS2DH12_4g:
            int1Ths.ths = (uint8_t) (userThresholdInMg >> 5);
            EDEBUG_PRINTF("Full Scale: 4 g\r\n");
            EDEBUG_PRINTF("Threshold: %d mg\r\n", int1Ths.ths << 5);
            break;
        case LIS2DH12_8g:
            int1Ths.ths = (uint8_t) (userThresholdInMg / 62);
            EDEBUG_PRINTF("Full Scale: 8 g\r\n");
            EDEBUG_PRINTF("Threshold: %d mg\r\n", int1Ths.ths * 62);
            break;
        case LIS2DH12_16g:
            int1Ths.ths = (uint8_t) (userThresholdInMg / 186);
            EDEBUG_PRINTF("Full Scale: 16 g\r\n");
            EDEBUG_PRINTF("Threshold: %d mg\r\n", int1Ths.ths * 186);
            break;
        default:
            EDEBUG_PRINTF("ERROR setting threshold. Undefined full-scale.\r\n");
            return error;
    }

    error =  writeToReg(LIS2DH12_INT1_THS, (uint8_t *) &int1Ths, 1);
    return error;
}

int32_t Lis2dh12::setDuration(uint16_t userDurationInMs) {
    lis2dh12_int1_duration_t int1Dur = {0};
    lis2dh12_odr_t odr;
    lis2dh12_ctrl_reg1_t ctrl_reg1;

    error = readFromReg(LIS2DH12_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (error) return error;

    odr = static_cast<lis2dh12_odr_t>(ctrl_reg1.odr);

    switch (odr) {
        case LIS2DH12_ODR_1Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 1000);
            EDEBUG_PRINTF("Duration: %d ms\r\n", int1Dur.d * 1000);
            EDEBUG_PRINTF("Sampling Rate: 1 Hz\r\n");
            break;
        case LIS2DH12_ODR_10Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 100);
            EDEBUG_PRINTF("Duration: %d ms\r\n", int1Dur.d * 100);
            EDEBUG_PRINTF("Sampling Rate: 10 Hz\r\n");
            break;
        case LIS2DH12_ODR_25Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 40);
            EDEBUG_PRINTF("Duration: %d ms\r\n", int1Dur.d * 40);
            EDEBUG_PRINTF("Sampling Rate: 25 Hz\r\n");
            break;
        case LIS2DH12_ODR_50Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 20);
            EDEBUG_PRINTF("Duration: %d ms\r\n", int1Dur.d * 20);
            EDEBUG_PRINTF("Sampling Rate: 50 Hz\r\n");
            break;
        case LIS2DH12_ODR_100Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 10);
            EDEBUG_PRINTF("Duration: %d ms\r\n", int1Dur.d * 10);
            EDEBUG_PRINTF("Sampling Rate: 100 Hz\r\n");
            break;
        case LIS2DH12_ODR_200Hz:
            int1Dur.d = (uint8_t) (userDurationInMs / 5);
            EDEBUG_PRINTF("Duration: %d ms\r\n", int1Dur.d * 5);
            EDEBUG_PRINTF("Sampling Rate: 200 Hz\r\n");
            break;
        case LIS2DH12_ODR_400Hz:
            int1Dur.d = (uint8_t) ((userDurationInMs << 1) / 5);
            EDEBUG_PRINTF("Duration: %d ms\r\n", (int1Dur.d * 5) >> 1);
            EDEBUG_PRINTF("Sampling Rate: 400 Hz\r\n");
            break;
        default:
            EDEBUG_PRINTF("ERROR setting duration. Undefined sampling rate.\r\n");
            return error;
    }

    error =  writeToReg(LIS2DH12_INT1_DURATION, (uint8_t *) &int1Dur, 1);
    return error;
}

int32_t Lis2dh12::selfTest() {
    uint8_t dataLevel = 0;
    acceleration_t selfTestArray[TEST_ARRAYSIZE];
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    acceleration_t firstAverage = {0};
    acceleration_t selfTestAverage = {0};
    acceleration_t absDif = {0};
    lis2dh12_fifo_src_reg_t fifo_src_reg;

    /* min and max values provided by sensor manufacturer (lis2dh12 datasheet) */
    int16_t minST = 17 << fullScale;
    int16_t maxST = 360 << fullScale;

    /* wait for stable output */
    wait_ms(100);

    /* read first available data and discard */
    error = readFromReg(LIS2DH12_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    if (error) return error;

    dataLevel = (uint8_t)fifo_src_reg.fss;

    for (int i = 0; i < dataLevel; i++) {
        error = getAcceleration(selfTestArray[0]);
        if (error) return error;
    }

    /* wait until new measurements in fifo */
    do {
        wait_ms(100);
        error = readFromReg(LIS2DH12_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
        if (error) return error;
    } while (dataLevel < TEST_ARRAYSIZE);

    /* read new values and save average of each axis */
    for (int i = 0; i < TEST_ARRAYSIZE; i++) {
        error = getAcceleration(selfTestArray[i]);
        if (error) return error;

        x_sum += selfTestArray[i].x_axis;
        y_sum += selfTestArray[i].y_axis;
        z_sum += selfTestArray[i].z_axis;
    }

    /* average values */
    firstAverage.x_axis = x_sum / TEST_ARRAYSIZE;
    firstAverage.y_axis = y_sum / TEST_ARRAYSIZE;
    firstAverage.z_axis = z_sum / TEST_ARRAYSIZE;

    /* reset variables for next average */
    x_sum = 0;
    y_sum = 0;
    z_sum = 0;

    /* enable selftest */
    lis2dh12_ctrl_reg4_t ctrl_reg4;
    error = readFromReg(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (error) return error;

    ctrl_reg4.st = (uint8_t)LIS2DH12_ST_POSITIVE;
    error = writeToReg(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (error) return error;

    /* wait for stable output */
    wait_ms(100);

    /* read first available data and discard */
    error = readFromReg(LIS2DH12_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    if (error) return error;

    dataLevel = (uint8_t)fifo_src_reg.fss;

    for (int i = 0; i < dataLevel; i++) {
        error = getAcceleration(selfTestArray[0]);
        if (error) return error;
    }

    /* wait until new measurements in fifo */
    do {
        wait_ms(100);
        error = readFromReg(LIS2DH12_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
        if (error) return error;

        dataLevel = (uint8_t)fifo_src_reg.fss;
    } while (dataLevel < TEST_ARRAYSIZE);

    /* read new values and save average of each axis */
    for (int i = 0; i < TEST_ARRAYSIZE; i++) {
        error = getAcceleration(selfTestArray[i]);
        if (error) return error;

        x_sum += selfTestArray[i].x_axis;
        y_sum += selfTestArray[i].y_axis;
        z_sum += selfTestArray[i].z_axis;
    }

    /* average values */
    selfTestAverage.x_axis = x_sum / TEST_ARRAYSIZE;
    selfTestAverage.y_axis = y_sum / TEST_ARRAYSIZE;
    selfTestAverage.z_axis = z_sum / TEST_ARRAYSIZE;

    /* disable selftest */
    error = readFromReg(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (error) return error;

    ctrl_reg4.st = (uint8_t)LIS2DH12_ST_DISABLE;
    error = writeToReg(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (error) return error;

    /* calculate absolute difference of both averages */
    absDif.x_axis = abs(selfTestAverage.x_axis - firstAverage.x_axis);
    absDif.y_axis = abs(selfTestAverage.y_axis - firstAverage.y_axis);
    absDif.z_axis = abs(selfTestAverage.z_axis - firstAverage.z_axis);

    /* check if difference is in acceptable range */
    if ((minST <= absDif.x_axis) && (absDif.x_axis <= maxST)
        && (minST <= absDif.y_axis) && (absDif.y_axis <= maxST)
        && (minST <= absDif.z_axis) && (absDif.z_axis <= maxST)) {
        EDEBUG_PRINTF("SELF TEST PASSED\r\n");
        return 0;
    } else {
        EDEBUG_PRINTF(" - - - SELF TEST FAILED - - - \r\n");
        return 0xffff;
    }
}

int32_t Lis2dh12::getAccelerationFifo(acceleration_t *accelerationArray) {

    for (int i = 0; i < ACC_ARRAYSIZE; i++) {
        error = getAcceleration(accelerationArray[i]);
        if (error) return error;
    }

    return error;
}

int32_t Lis2dh12::getAcceleration(acceleration_t &acceleration) {
    axis3bit16_t data_raw_acceleration;
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));

    /* read accelerometer data from sensor */
    error = readFromReg(LIS2DH12_OUT_X_L, data_raw_acceleration.u8bit, 6);
    if (error) return error;

    acceleration.x_axis = convert_to_mg(data_raw_acceleration.i16bit[0]);
    acceleration.y_axis = convert_to_mg(data_raw_acceleration.i16bit[1]);
    acceleration.z_axis = convert_to_mg(data_raw_acceleration.i16bit[2]);

    return error;
}

int16_t Lis2dh12::convert_to_mg(int16_t rawData) {

    switch (fullScale) {
        case LIS2DH12_2g:
            return (rawData >> 4);
        case LIS2DH12_4g:
            return (rawData >> 3);
        case LIS2DH12_8g:
            return (rawData >> 2);
        case LIS2DH12_16g:
            return ((rawData * 3) >> 2);
        default:
            EDEBUG_PRINTF("ERROR converting raw acceleration data. Undefined full-scale.\r\n");
            return 0xffff;
    }
}

/* reset latched threshold interrupt */
int16_t Lis2dh12::resetInterrupt() {
    lis2dh12_int1_src_t int1Src;

    return readFromReg(LIS2DH12_INT1_SRC, (uint8_t*) &int1Src, 1);
}

/* reset latched threshold interrupt and check cause */
int16_t Lis2dh12::resetInterrupt(bool *_xyzHighEvent) {
    lis2dh12_int1_src_t int1Src = {0};

    error = readFromReg(LIS2DH12_INT1_SRC, (uint8_t*) &int1Src, 1);

    *_xyzHighEvent = int1Src.ia;

    return error;
}

int32_t Lis2dh12::checkFifoStatus(bool *_overrun) {
    lis2dh12_fifo_src_reg_t fifoSrcReg = {0};

    error = readFromReg(LIS2DH12_FIFO_SRC_REG, (uint8_t*) &fifoSrcReg, 1);

    *_overrun = fifoSrcReg.ovrn_fifo;

    return error;
}

/* activate fifo overrun interrupt and deactivate threshold interrupt */
int16_t Lis2dh12::waitForOverrunInt() {
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_overrun = 1;
    ctrlReg3.i1_ia1 = 0;
    error = writeToReg(LIS2DH12_CTRL_REG3, (uint8_t*) &ctrlReg3, 1);
    if (!error) waitingForThresholdInterrupt = false;
//    EDEBUG_PRINTF("waiting for overrun interrupt...\r\n");
    return error;
}

/* deactivate fifo overrun interrupt and activates activity interrupt */
int16_t Lis2dh12::waitForThresholdInt() {
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_overrun = 0;
    ctrlReg3.i1_ia1 = 1;
    error = writeToReg(LIS2DH12_CTRL_REG3, (uint8_t*) &ctrlReg3, 1);
    if (!error) waitingForThresholdInterrupt = true;
//    EDEBUG_PRINTF("waiting for threshold interrupt...\r\n");
    return error;
}

bool Lis2dh12::isWaitingForThresholdInterrupt() {
    return waitingForThresholdInterrupt;
}

int32_t Lis2dh12::checkFifoDataLevel() {
    lis2dh12_fifo_src_reg_t fifoSrcReg = {0};

    error = readFromReg(LIS2DH12_FIFO_SRC_REG, (uint8_t*) &fifoSrcReg, 1);
    if (error) return error;

    EDEBUG_PRINTF("%d unread values in FIFO\r\n", fifoSrcReg.fss);
    return error;
}

void Lis2dh12::readAllRegisters(void) {

    uint8_t data[10];
    for (int i = 0x1E; i <= 0x33; ++i) {
        readFromReg(i, data, 1);
        EDEBUG_PRINTF("REG:%02x = %02x\r\n", i, data[0]);
    }
}
