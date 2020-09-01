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

#include "mbed.h"
#include "lis2dh12_reg.h"

#define ACC_ARRAYSIZE 32    // size of accelerometer FIFO
#define TEST_ARRAYSIZE 5    // array for sensor self test

typedef struct {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
}acceleration_t;

typedef enum {
    NORMAL_10bit = 0,
    LOW_POWER_8bit = 1,
} resolution_mode_t;

class Lis2dh12 {
public:
    Lis2dh12(I2C *_i2c,
             uint16_t _thresholdInMg, uint16_t _durationInMs,
             lis2dh12_odr_t _samplRate, lis2dh12_fs_t _fullScale);

    virtual ~Lis2dh12();

    int32_t init();

    int32_t getAccelerationFifo(acceleration_t *accelerationArray);

    int16_t resetInterrupt();

    int32_t activateSensor();

    int32_t powerDown();

private:
    int32_t selfTest();

    int32_t enableThsInterrupt();

    int32_t setDuration(uint16_t userDurationInMs);

    int32_t setThreshold(uint16_t userThresholdInMg);

    int32_t getAcceleration(acceleration_t &acceleration);

    int16_t convert_to_mg(int16_t rawData);

    int16_t resetInterrupt(bool *_xyzHighEvent);

    int32_t checkFifoStatus(bool *_overrun);

    int32_t checkFifoDataLevel();

    int16_t waitForOverrunInt();

    int16_t waitForThresholdInt();

    bool isWaitingForThresholdInterrupt();

    void readAllRegisters(void);

    uint8_t tx_buffer[1000];

    I2C *i2c;
    uint8_t i2cAddr;

    int32_t readFromReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);
    int32_t writeToReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);

    int16_t error;
    uint16_t thresholdInMg;
    uint16_t durationInMs;
    lis2dh12_odr_t samplRate;
    lis2dh12_fs_t fullScale;
    bool waitingForThresholdInterrupt;
};

#endif //UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
