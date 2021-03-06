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

#include "Lis2dh12.h"
#include <edebug.h>

// returns 0 on success , non-0 on failure
int8_t Lis2dh12::readReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize) {
    if (buffSize > 1) {
        regAddr |= 0x80;
    }
    i2c->write(i2cAddr & 0xFE, reinterpret_cast<const char *>(&regAddr), 1, true);
    return i2c->read(i2cAddr | 0x01, reinterpret_cast<char *>(buff), buffSize);
}

// returns 0 on success , non-0 on failure
int8_t Lis2dh12::writeReg(uint8_t regAddr, uint8_t *data, uint16_t len) {
    char dataToSend[len + 1];

    dataToSend[0] = regAddr;
    memcpy(dataToSend + 1, data, len);
    return i2c->write(i2cAddr & 0xFE, dataToSend, len + 1);
}

Lis2dh12::Lis2dh12(I2C *_i2c) : i2c(_i2c), i2cAddr(LIS2DH12_I2C_ADD_H), sampRate(), fullScale(), resolution() {}

Lis2dh12::~Lis2dh12() {}

int8_t Lis2dh12::init() {
    if (!whoAmI()) {
        return -1;
    }

    int8_t error;

    error = initFIFO();
    if (error) {
        return error;
    }

    error = enableFIFO();
    return error;
}

/*  Check sensor ID */
bool Lis2dh12::whoAmI() {
    uint8_t whoamI;
    int8_t error = readReg(LIS2DH12_WHO_AM_I, &whoamI, 1);
    if (error) {
        return false;
    }

    if (whoamI != LIS2DH12_ID) {
        EDEBUG_PRINTF("Sensor ID check failed! (Expected ID: 0x%02x - Got: 0x%02x)\r\n", LIS2DH12_ID, whoamI);
        return false;
    } else {
        EDEBUG_PRINTF("Sensor ID check OK\r\n");
        return true;
    }
}

int8_t Lis2dh12::enableSensor() {
    int8_t error;

    lis2dh12_ctrl_reg1_t ctrlReg1;
    error = readReg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrlReg1, 1);
    if (error) {
        return error;
    }

    ctrlReg1.odr = sampRate;                                      // Set sampling rate
    return writeReg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrlReg1, 1); // turn on sensor
}

int8_t Lis2dh12::disableSensor() {
    int8_t error;

    lis2dh12_ctrl_reg1_t ctrlReg1;
    error = readReg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrlReg1, 1);
    if (error) {
        return error;
    }

    ctrlReg1.odr = 0;                                             // Set sampling rate
    return writeReg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrlReg1, 1); // turn off sensor
}

int8_t Lis2dh12::setOperatingMode(lis2dh12_odr_t _sampRate, lis2dh12_fs_t _fullScale, lis2dh12_op_md_t _res) {
    int8_t error;

    sampRate = _sampRate;
    fullScale = _fullScale;
    resolution = _res;

    EDEBUG_PRINTF("Setting operating mode:\r\n"
                  "  - samp. rate: %3d Hz\r\n"
                  "  - full scale: %3d G\r\n"
                  "  - resolution: %3d bit\r\n\r\n",
                  sampRateToInt(sampRate), fullScaleToInt(fullScale), resolutionToInt(resolution));

    /* Block Data Update, Big/Little Endian data selection,
     * Full-scale selection, Operating mode selection, Self-test enable,
     * SPI serial interface mode selection */
    lis2dh12_ctrl_reg4_t ctrlReg4 = {0};
    ctrlReg4.bdu = 1;                                  // Enable Block Data Update
    ctrlReg4.fs = fullScale;                           // Set full scale
    ctrlReg4.hr = (_res == LIS2DH12_HR_12bit) ? 1 : 0; // set resolution
    error = writeReg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrlReg4, 1);
    if (error) {
        return error;
    }

    /* reset filtering block */
    uint8_t buf;
    error = readReg(LIS2DH12_REFERENCE, &buf, 1);
    if (error) {
        return error;
    }

    /* ODR, Low Power enable, Axes enable */
    lis2dh12_ctrl_reg1_t ctrlReg1 = {0};
    ctrlReg1.odr = sampRate;                                  // Set sampling rate
    ctrlReg1.lpen = (resolution == LIS2DH12_LP_8bit) ? 1 : 0; // set power mode
    ctrlReg1.xen = 1;                                         // Enable all axes
    ctrlReg1.yen = 1;
    ctrlReg1.zen = 1;
    error = writeReg(LIS2DH12_CTRL_REG1, (uint8_t *)&ctrlReg1, 1);
    if (error) {
        return error;
    }

    /* wait duration of turn-on time (7/odr) */
    if (sampRate != LIS2DH12_POWER_DOWN) {
        wait(float(7) / float(sampRateToInt(sampRate)));
    }

    return error;
}

int8_t Lis2dh12::initFIFO() {
    /* FIFO enable and latch interrupt request */
    lis2dh12_ctrl_reg5_t ctrlReg5 = {0};
    ctrlReg5.fifo_en = 1;  // Enable FIFO
    ctrlReg5.lir_int1 = 1; // latch interrupt 1 request
                           // (read INT1_SRC (31h) to reset)
    return writeReg(LIS2DH12_CTRL_REG5, (uint8_t *)&ctrlReg5, 1);
}

int8_t Lis2dh12::enableFIFO() {
    lis2dh12_fifo_ctrl_reg_t fifoCtrlReg = {0};
    fifoCtrlReg.fm = LIS2DH12_DYNAMIC_STREAM_MODE; // select FIFO mode
    return writeReg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifoCtrlReg, 1);
}

int8_t Lis2dh12::disableFIFO() {
    lis2dh12_fifo_ctrl_reg_t fifoCtrlReg = {0};
    return writeReg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifoCtrlReg, 1);
}

int8_t Lis2dh12::resetFIFO() {
    int8_t error;

    lis2dh12_fifo_ctrl_reg_t fifoCtrlReg = {0};

    // disable FIFO
    error = writeReg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifoCtrlReg, 1);
    if (error) {
        return error;
    }

    if (sampRate != LIS2DH12_POWER_DOWN) {
        wait(float(1) / float(sampRateToInt(sampRate)));
    }

    // re-enable FIFO
    fifoCtrlReg.fm = LIS2DH12_DYNAMIC_STREAM_MODE; // select FIFO mode
    return writeReg(LIS2DH12_FIFO_CTRL_REG, (uint8_t *)&fifoCtrlReg, 1);
}

int8_t Lis2dh12::initHPF(uint8_t hpcf) {
    int8_t error;

    lis2dh12_ctrl_reg2_t ctrlReg2;
    error = readReg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrlReg2, 1);
    if (error) {
        return error;
    }

    ctrlReg2.hpm = 0;            // filter mode: normal
    ctrlReg2.hpcf = hpcf & 0b11; // cutoff frequency
                                 //    ctrlReg2.fds = 1;
    error = writeReg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrlReg2, 1);
    if (error) {
        return error;
    }

    EDEBUG_PRINTF("High-pass filter cutoff frequency IDX: %x\r\n\r\n", (hpcf & 0b11));

    /* reset high-pass filter */
    uint8_t buf;
    return readReg(LIS2DH12_REFERENCE, &buf, 1);
}

int8_t Lis2dh12::getAccelerationFifo(acceleration_t *accelerationArray) {
    int8_t error;

    for (int i = 0; i < ACC_ARRAYSIZE; i++) {
        error = getAcceleration(accelerationArray[i]);
        if (error) {
            return error;
        }
    }
    return error;
}

int8_t Lis2dh12::getAcceleration(acceleration_t &acceleration) {
    axis3bit16_t data_raw_acceleration;
    memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));

    /* read accelerometer data from sensor */
    int8_t error = readReg(LIS2DH12_OUT_X_L, data_raw_acceleration.u8bit, 6);
    if (error) {
        return error;
    }

    acceleration.x_axis = convert_to_mg(data_raw_acceleration.i16bit[0]);
    acceleration.y_axis = convert_to_mg(data_raw_acceleration.i16bit[1]);
    acceleration.z_axis = convert_to_mg(data_raw_acceleration.i16bit[2]);

    return error;
}

int8_t Lis2dh12::isDRDY(uint8_t *ready) {
    lis2dh12_status_reg_t status;

    int8_t error = readReg(LIS2DH12_STATUS_REG, (uint8_t *)&status, 1);

    *ready = status.zyxda;

    return error;
}

int8_t Lis2dh12::isFIFOfull(uint8_t *overrun) {
    lis2dh12_fifo_src_reg_t fifoSrcReg = {0};

    int8_t error = readReg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifoSrcReg, 1);

    *overrun = fifoSrcReg.ovrn_fifo;

    return error;
}

/* reset latched IA1 interrupt */
int8_t Lis2dh12::resetThsInt() {
    uint8_t data;
    return readReg(LIS2DH12_INT1_SRC, &data, 1);
}

int8_t Lis2dh12::initThsInt(uint16_t thresholdInMg, uint16_t durationInMs) {
    int8_t error;

    /* Set threshold in mg */
    EDEBUG_PRINTF("Threshold Event Acceleration: ");
    lis2dh12_int1_ths_t int1Ths = {};
    int1Ths.ths = setThsMg(thresholdInMg);
    error = writeReg(LIS2DH12_INT1_THS, (uint8_t *)&int1Ths, 1);
    if (error) {
        return error;
    }

    /* Set duration in ms */
    EDEBUG_PRINTF("Threshold Event Duration: ");
    lis2dh12_int1_duration_t int1Dur = {0};
    int1Dur.d = setDurMs(durationInMs);
    error = writeReg(LIS2DH12_INT1_DURATION, (uint8_t *)&int1Dur, 1);
    if (error) {
        return error;
    }

    /* Configure high-pass filter for IA1 */
    lis2dh12_ctrl_reg2_t ctrlReg2;
    error = readReg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrlReg2, 1);
    if (error) {
        return error;
    }

    ctrlReg2.hp |= 0b001; // HPCLICK + HP_IA2 + HP_IA1 -> HP
    error = writeReg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrlReg2, 1);
    if (error) {
        return error;
    }

    /* reset high-pass filter */
    uint8_t buf;
    error = readReg(LIS2DH12_REFERENCE, &buf, 1);
    if (error) {
        return error;
    }

    /* IA1 configuration */
    lis2dh12_int1_cfg_t int1Cfg = {};
    error = readReg(LIS2DH12_INT1_CFG, (uint8_t *)&int1Cfg, 1);
    if (error) {
        return error;
    }

    int1Cfg.xhie = 1; // enable high event interrupts for all axes
    int1Cfg.yhie = 1; // (if acc > threshold for t > duration)
    int1Cfg.zhie = 1;
    return writeReg(LIS2DH12_INT1_CFG, (uint8_t *)&int1Cfg, 1);
}

/* enable interrupt for IA1 on INT2 pin */
int8_t Lis2dh12::enableThsInt() {
    int8_t error;

    lis2dh12_ctrl_reg6_t ctrlReg6 = {0};
    ctrlReg6.i2_ia1 = 1;
    error = writeReg(LIS2DH12_CTRL_REG6, (uint8_t *)&ctrlReg6, 1);
    if (error) {
        return error;
    }
    EDEBUG_PRINTF("Threshold Event Interrupt enabled\r\n\r\n");

    /* clear interrupts */
    return resetThsInt();
}

int8_t Lis2dh12::disableThsInt() {
    int8_t error;

    lis2dh12_ctrl_reg6_t ctrlReg6;
    error = readReg(LIS2DH12_CTRL_REG6, (uint8_t *)&ctrlReg6, 1);
    if (error) {
        return error;
    }

    ctrlReg6.i2_ia1 = 0;
    error = writeReg(LIS2DH12_CTRL_REG6, (uint8_t *)&ctrlReg6, 1);
    if (error) {
        return error;
    }
    EDEBUG_PRINTF("Threshold Event Interrupt disabled\r\n\r\n");

    return error;
}

/* enable interrupt for fifo overflow on INT1 pin */
int8_t Lis2dh12::enableFIFOOverrunInt() {
    lis2dh12_ctrl_reg3_t ctrlReg3 = {0};
    ctrlReg3.i1_overrun = 1;
    return writeReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg3, 1);
}

int8_t Lis2dh12::disableFIFOOverrunInt() {
    int8_t error;

    lis2dh12_ctrl_reg3_t ctrlReg3;
    error = readReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg3, 1);
    if (error) {
        return error;
    }

    ctrlReg3.i1_overrun = 0;
    error = writeReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg3, 1);
    if (error) {
        return error;
    }
    EDEBUG_PRINTF("FIFO Overrun Interrupt disabled\r\n\r\n");

    return error;
}

int8_t Lis2dh12::enableDoubleClickInterrupt() {
    int8_t error;

    /* set double click interrupt threshold and latch interrupt */
    EDEBUG_PRINTF("Double Click Threshold: ");
    lis2dh12_click_ths_t clickThs = {};
    clickThs.lir_click = 1;
    clickThs.ths = setThsMg(DOUBLE_CLICK_THS);
    error = writeReg(LIS2DH12_CLICK_THS, (uint8_t *)&clickThs, 1);
    if (error) {
        return error;
    }

    /* set time limit */
    EDEBUG_PRINTF("Time Limit: ");
    lis2dh12_time_limit_t timeLimit = {};
    timeLimit.tli = setDurMs(128);
    error = writeReg(LIS2DH12_TIME_LIMIT, (uint8_t *)&timeLimit, 1);
    if (error) {
        return error;
    }

    /* set time latency */
    EDEBUG_PRINTF("Time Latency: ");
    lis2dh12_time_latency_t timeLatency = {};
    timeLatency.tla = setDurMs(53);
    error = writeReg(LIS2DH12_TIME_LATENCY, (uint8_t *)&timeLatency, 1);
    if (error) {
        return error;
    }

    /* set time window */
    EDEBUG_PRINTF("Time Window: ");
    lis2dh12_time_window_t timeWindow = {};
    timeWindow.tw = setDurMs(638);
    error = writeReg(LIS2DH12_TIME_WINDOW, (uint8_t *)&timeWindow, 1);
    if (error) {
        return error;
    }

    /* double click interrupt configuration */
    lis2dh12_click_cfg_t clickCfg = {};
    clickCfg.zd = 1; // only enable for Z-axis
    error = writeReg(LIS2DH12_CLICK_CFG, (uint8_t *)&clickCfg, 1);
    if (error) {
        return error;
    }

    /* configure high-pass filter for double click interrupt*/
    lis2dh12_ctrl_reg2_t ctrlReg2;
    error = readReg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrlReg2, 1);
    if (error) {
        return error;
    }

    ctrlReg2.hp |= 0b100; // HPCLICK + HP_IA2 + HP_IA1 -> HP
    error = writeReg(LIS2DH12_CTRL_REG2, (uint8_t *)&ctrlReg2, 1);
    if (error) {
        return error;
    }

    /* reset high-pass filter */
    uint8_t buf;
    error = readReg(LIS2DH12_REFERENCE, &buf, 1);
    if (error) {
        return error;
    }

    /* enable interrupt for double click on INT2 pin */
    lis2dh12_ctrl_reg6_t ctrlReg6 = {};
    error = readReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg6, 1);
    if (error) {
        return error;
    }

    ctrlReg6.i2_click = 1; // generate double click interrupt on INT1 pin
    error = writeReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg6, 1);
    if (error) {
        return error;
    }

    EDEBUG_PRINTF("Double Click Interrupt enabled\r\n\r\n");

    /* clear interrupt register */
    return resetDoubleClickInterrupt();
}

int8_t Lis2dh12::disableDoubleClickInterrupt() {
    int8_t error;

    lis2dh12_ctrl_reg3_t ctrlReg3 = {};
    error = readReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg3, 1);
    if (error) {
        return error;
    }

    ctrlReg3.i1_click = 0;
    return writeReg(LIS2DH12_CTRL_REG3, (uint8_t *)&ctrlReg3, 1);
}

int8_t Lis2dh12::resetDoubleClickInterrupt() {
    uint8_t data;
    return readReg(LIS2DH12_CLICK_SRC, &data, 1);
}

int8_t Lis2dh12::selfTest() {
    int8_t error = 0;
    acceleration_t selfTestArray[TEST_ARRAYSIZE];
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    int32_t z_sum = 0;
    acceleration_t firstAverage = {0};
    acceleration_t selfTestAverage = {0};
    acceleration_t absDif = {0};

    /* min and max values provided by sensor manufacturer (lis2dh12 datasheet) */
    int16_t minST = 17 << fullScale;
    int16_t maxST = 360 << fullScale;

    /* wait for stable output */
    wait_ms(100);

    /* read first available data and discard */
    lis2dh12_fifo_src_reg_t fifo_src_reg;
    error = readReg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
    if (error) {
        return error;
    }

    for (int i = 0; i < fifo_src_reg.fss; i++) {
        error = getAcceleration(selfTestArray[0]);
        if (error) {
            return error;
        }
    }

    /* wait until new measurements in fifo */
    do {
        wait_ms(100);
        error = readReg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
        if (error) {
            return error;
        }
    } while (fifo_src_reg.fss < TEST_ARRAYSIZE);

    /* read new values and save average of each axis */
    for (int i = 0; i < TEST_ARRAYSIZE; i++) {
        error = getAcceleration(selfTestArray[i]);
        if (error) {
            return error;
        }

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
    error = readReg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
    if (error) {
        return error;
    }

    ctrl_reg4.st = (uint8_t)LIS2DH12_ST_POSITIVE;
    error = writeReg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
    if (error) {
        return error;
    }

    /* wait for stable output */
    wait_ms(100);

    /* read first available data and discard */
    error = readReg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
    if (error) {
        return error;
    }

    for (int i = 0; i < fifo_src_reg.fss; i++) {
        error = getAcceleration(selfTestArray[0]);
        if (error) {
            return error;
        }
    }

    /* wait until new measurements in fifo */
    do {
        wait_ms(100);
        error = readReg(LIS2DH12_FIFO_SRC_REG, (uint8_t *)&fifo_src_reg, 1);
        if (error) {
            return error;
        }
    } while (fifo_src_reg.fss < TEST_ARRAYSIZE);

    /* read new values and save average of each axis */
    for (int i = 0; i < TEST_ARRAYSIZE; i++) {
        error = getAcceleration(selfTestArray[i]);
        if (error) {
            return error;
        }

        x_sum += selfTestArray[i].x_axis;
        y_sum += selfTestArray[i].y_axis;
        z_sum += selfTestArray[i].z_axis;
    }

    /* average values */
    selfTestAverage.x_axis = x_sum / TEST_ARRAYSIZE;
    selfTestAverage.y_axis = y_sum / TEST_ARRAYSIZE;
    selfTestAverage.z_axis = z_sum / TEST_ARRAYSIZE;

    /* disable selftest */
    error = readReg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
    if (error) {
        return error;
    }

    ctrl_reg4.st = (uint8_t)LIS2DH12_ST_DISABLE;
    error = writeReg(LIS2DH12_CTRL_REG4, (uint8_t *)&ctrl_reg4, 1);
    if (error) {
        return error;
    }

    /* calculate absolute difference of both averages */
    absDif.x_axis = abs(selfTestAverage.x_axis - firstAverage.x_axis);
    absDif.y_axis = abs(selfTestAverage.y_axis - firstAverage.y_axis);
    absDif.z_axis = abs(selfTestAverage.z_axis - firstAverage.z_axis);

    /* check if difference is in acceptable range */
    if ((minST <= absDif.x_axis) && (absDif.x_axis <= maxST) && (minST <= absDif.y_axis) && (absDif.y_axis <= maxST) &&
        (minST <= absDif.z_axis) && (absDif.z_axis <= maxST)) {
        EDEBUG_PRINTF("Sensor self test passed\r\n\r\n");
        return 0;
    } else {
        EDEBUG_PRINTF(" - - - SENSOR SELF TEST FAILED! - - - \r\n");
        return -1;
    }
}

void Lis2dh12::readAllRegisters(void) {
    uint8_t buf;
    EDEBUG_PRINTF("---------------\r\n");
    for (int i = 0x1E; i <= 0x33; ++i) {
        readReg(i, &buf, 1);
        EDEBUG_PRINTF("REG 0x%02x = 0x%02x\r\n", i, buf);
    }
    EDEBUG_PRINTF("---------------\r\n");
}

void Lis2dh12::readRegister(uint8_t registerAddr) {
    uint8_t buf;
    readReg(registerAddr, &buf, 1);
    EDEBUG_PRINTF("REG 0x%02x = 0x%02x\r\n\r\n", registerAddr, buf);
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
        EDEBUG_PRINTF("ERROR converting raw acceleration data: undefined full-scale\r\n");
        return 0xffff;
    }
}

uint8_t Lis2dh12::setThsMg(uint16_t userThresholdInMg) {
    uint8_t ths;

    /* LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g */
    switch (fullScale) {
    case LIS2DH12_2g:
        ths = (uint8_t)(userThresholdInMg >> 4);
        EDEBUG_PRINTF("%d mg\r\n", ths << 4);
        return ths;
    case LIS2DH12_4g:
        ths = (uint8_t)(userThresholdInMg >> 5);
        EDEBUG_PRINTF("%d mg\r\n", ths << 5);
        return ths;
    case LIS2DH12_8g:
        ths = (uint8_t)(userThresholdInMg / 62);
        EDEBUG_PRINTF("%d mg\r\n", ths * 62);
        return ths;
    case LIS2DH12_16g:
        ths = (uint8_t)(userThresholdInMg / 186);
        EDEBUG_PRINTF("%d mg\r\n", ths * 186);
        return ths;
    default:
        EDEBUG_PRINTF("ERROR setting threshold: undefined full-scale\r\n");
        return 0xff;
    }
}

int16_t Lis2dh12::setDurMs(uint16_t userDurationInMs) {
    uint8_t d;

    switch (sampRate) {
    case LIS2DH12_ODR_1Hz:
        d = (uint8_t)(userDurationInMs / 1000);
        EDEBUG_PRINTF("%d ms\r\n", d * 1000);
        return d;
    case LIS2DH12_ODR_10Hz:
        d = (uint8_t)(userDurationInMs / 100);
        EDEBUG_PRINTF("%d ms\r\n", d * 100);
        return d;
    case LIS2DH12_ODR_25Hz:
        d = (uint8_t)(userDurationInMs / 40);
        EDEBUG_PRINTF("%d ms\r\n", d * 40);
        return d;
    case LIS2DH12_ODR_50Hz:
        d = (uint8_t)(userDurationInMs / 20);
        EDEBUG_PRINTF("%d ms\r\n", d * 20);
        return d;
    case LIS2DH12_ODR_100Hz:
        d = (uint8_t)(userDurationInMs / 10);
        EDEBUG_PRINTF("%d ms\r\n", d * 10);
        return d;
    case LIS2DH12_ODR_200Hz:
        d = (uint8_t)(userDurationInMs / 5);
        EDEBUG_PRINTF("%d ms\r\n", d * 5);
        return d;
    case LIS2DH12_ODR_400Hz:
        d = (uint8_t)((userDurationInMs << 1) / 5);
        EDEBUG_PRINTF("%d ms\r\n", (d * 5) >> 1);
        return d;
    default:
        EDEBUG_PRINTF("ERROR setting duration: undefined sampling rate\r\n");
        return -1;
    }
}

uint16_t Lis2dh12::sampRateToInt(lis2dh12_odr_t sr) {
    switch (sr) {
    case LIS2DH12_POWER_DOWN:
        return 0;
    case LIS2DH12_ODR_1Hz:
        return 1;
    case LIS2DH12_ODR_10Hz:
        return 10;
    case LIS2DH12_ODR_25Hz:
        return 25;
    case LIS2DH12_ODR_50Hz:
        return 50;
    case LIS2DH12_ODR_100Hz:
        return 100;
    case LIS2DH12_ODR_200Hz:
        return 200;
    case LIS2DH12_ODR_400Hz:
        return 400;
    default:
        return 0;
    }
}

uint8_t Lis2dh12::fullScaleToInt(lis2dh12_fs_t fs) {
    switch (fs) {
    case LIS2DH12_2g:
        return 2;
    case LIS2DH12_4g:
        return 4;
    case LIS2DH12_8g:
        return 8;
    case LIS2DH12_16g:
        return 16;
    default:
        return 0;
    }
}

uint8_t Lis2dh12::resolutionToInt(lis2dh12_op_md_t r) {
    switch (r) {
    case LIS2DH12_LP_8bit:
        return 8;
    case LIS2DH12_NM_10bit:
        return 10;
    case LIS2DH12_HR_12bit:
        return 12;
    default:
        return 0;
    }
}
