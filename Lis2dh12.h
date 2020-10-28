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
  Lis2dh12(I2C *_i2c);

  virtual ~Lis2dh12();

  int16_t init();

  int16_t enableSensor();

  int16_t disableSensor();

  int16_t setOperatingMode(lis2dh12_odr_t _sampRate, lis2dh12_fs_t _fullScale,
                           lis2dh12_op_md_t _res);

  int16_t initFIFO();

  int16_t enableFIFO();

  int16_t disableFIFO();

  int16_t resetFIFO();

  int16_t initHPF();

  int16_t getAccelerationFifo(acceleration_t *accelerationArray, bool debug);

  int16_t getAcceleration(acceleration_t &acceleration);

  int16_t isDRDY(uint8_t *ready);

  int16_t isFIFOfull(uint8_t *overrun);

  int16_t resetInt1();

  int16_t resetInt2();

  int16_t initThsInt(uint16_t thresholdInMg, uint16_t durationInMs);

  int16_t enableThsInt();

  int16_t disableThsInterrupt();

  int16_t enableFIFOOverrunInt();

  int16_t disableFIFOOverflowInterrupt();

  int16_t resetDoubleClickInterrupt();

  int16_t enableDoubleClickInterrupt();

  int16_t disableDoubleClickInterrupt();

  int16_t selfTest();

  void readAllRegisters();

private:
  int16_t readReg(uint8_t regAddr, uint8_t *buff, uint16_t buffSize);

  int16_t writeReg(uint8_t regAddr, uint8_t *data, uint16_t len);

  int16_t convert_to_mg(int16_t rawData);

  uint8_t setThsMg(uint16_t userThresholdInMg);

  int16_t setDurMs(uint16_t userDurationInMs);

  uint16_t sampRateToInt(lis2dh12_odr_t sr);

  uint8_t fullScaleToInt(lis2dh12_fs_t fs);

  uint8_t resolutionToInt(lis2dh12_op_md_t r);

  I2C *i2c;
  uint8_t i2cAddr = LIS2DH12_I2C_ADD_H;
  lis2dh12_odr_t sampRate = LIS2DH12_POWER_DOWN;
  lis2dh12_fs_t fullScale = LIS2DH12_2g;
  lis2dh12_op_md_t resolution = LIS2DH12_NM_10bit;
};

#endif // UBIRCH_ENERTHING_FIRMWARE_LIS2DH12_H
