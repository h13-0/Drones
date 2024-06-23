#ifndef __ED_DRIVERS_H__
#define __ED_DRIVERS_H__

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "ed_motor.h"

typedef struct {
    /** wifi configs **/
    // sta configs.
    char* sta_ssid;
    char* sta_pswd;

    /** i2c configs **/
    i2c_port_t i2c_num;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;

    /** imu configs **/
    uint16_t imu_freq;
    uint8_t imu_retry_times;

    /** motor configs **/
    ed_motor_t m1;
    ed_motor_t m2;
    ed_motor_t m3;
    ed_motor_t m4;

    /** debugger configs **/
    uint16_t tcp_port;
} ed_drivers_config_t;


int ed_drivers_init(ed_drivers_config_t *config);

#endif
