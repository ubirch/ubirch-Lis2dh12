//
// Created by larox on 20.06.19.
//

#include <edebug.h>
#include "Lis2dh12.h"

Lis2dh12::Lis2dh12() {

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

int32_t Lis2dh12::read(float *x_axis, float *y_axis, float *z_axis){
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