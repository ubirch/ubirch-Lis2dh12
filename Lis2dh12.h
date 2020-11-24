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

#ifndef UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
#define UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H

#include "lis2dh12_reg.h"
#include "mbed.h"

#define ACC_ARRAYSIZE 32 // size of accelerometer FIFO
#define TEST_ARRAYSIZE 5 // array for sensor self test

#define DOUBLE_CLICK_THS 900 // threshold for double click detection in mg

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} acceleration_t;

class Lis2dh12 {
  public:
    explicit Lis2dh12(I2C *_i2c);

    virtual ~Lis2dh12();

    int8_t init();

    bool whoAmI();

    int8_t enableSensor();

    int8_t disableSensor();

    int8_t setOperatingMode(lis2dh12_odr_t _sampRate, lis2dh12_fs_t _fullScale, lis2dh12_op_md_t _res);

    int8_t initFIFO();

    int8_t enableFIFO();

    int8_t disableFIFO();

    int8_t resetFIFO();

    int8_t initHPF(uint8_t hpcf = 0);

    int8_t getAccelerationFifo(acceleration_t *accelerationArray);

    int8_t getAcceleration(acceleration_t &acceleration);

    int8_t isDRDY(uint8_t *ready);

    int8_t isFIFOfull(uint8_t *overrun);

    int8_t initThsInt(uint16_t thresholdInMg, uint16_t durationInMs);

    int8_t enableThsInt();

    int8_t disableThsInt();

    int8_t resetThsInt();

    int8_t enableFIFOOverrunInt();

    int8_t disableFIFOOverrunInt();

    int8_t resetDoubleClickInterrupt();

    int8_t enableDoubleClickInterrupt();

    int8_t disableDoubleClickInterrupt();

    int8_t selfTest();

    void readAllRegisters();

    void readRegister(uint8_t registerAddr);

  private:
    int8_t readReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);

    int8_t writeReg(uint8_t regAddr, uint8_t *data, uint16_t len);

    int16_t convert_to_mg(int16_t rawData);

    uint8_t setThsMg(uint16_t userThresholdInMg);

    int16_t setDurMs(uint16_t userDurationInMs);

    uint16_t sampRateToInt(lis2dh12_odr_t sr);

    uint8_t fullScaleToInt(lis2dh12_fs_t fs);

    uint8_t resolutionToInt(lis2dh12_op_md_t r);

    I2C *i2c;
    uint8_t i2cAddr;
    lis2dh12_odr_t sampRate;
    lis2dh12_fs_t fullScale;
    lis2dh12_op_md_t resolution;
};

#endif // UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
