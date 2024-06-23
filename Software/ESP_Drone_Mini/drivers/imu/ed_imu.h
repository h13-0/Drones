#ifndef __ED_IMU_H__
#define __ED_IMU_H__

#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

int ed_imu_init(i2c_port_t i2c_port, uint16_t frequency, int retry);

int ed_imu_int_enable(gpio_num_t int_io, gpio_isr_t int_handler);

int ed_imu_get_eular(float *pitch, float *roll, float *yaw);

int ed_imu_get_gyro(float *gx, float *gy, float *gz);

int ed_imu_get_accel(float *ax, float *ay, float *az);

#ifdef __cplusplus
}
#endif

#endif
