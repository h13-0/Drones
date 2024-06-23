#ifndef __ESP_DRONE_CONFIG_H__
#define __ESP_DRONE_CONFIG_H__

#include "esp_drone_private_config.h"

/******************************************************************************/
#include "ed_drivers.h"
#include "oh_quadrotor_pid.h"
typedef struct {
    ed_drivers_config_t drivers;
    oh_quad_pid_t       pid_param;
} ed_drv_t;

#define ESP_DRONE   { \
                        .drivers = { \
                            .sta_ssid = ESP_DRONE_WIFI_CLIENT_SSID, \
                            .sta_pswd = ESP_DRONE_WIFI_CLIENT_PSWD, \
                            .i2c_num  = I2C_NUM_0, \
                            .sda_io_num = GPIO_NUM_1, \
                            .scl_io_num = GPIO_NUM_2, \
                            .imu_freq = 100, \
                            .imu_retry_times = 10, \
                            .m1 = { \
                                .gpio_num = GPIO_NUM_5, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_0, \
                            }, \
                            .m2 = { \
                                .gpio_num = GPIO_NUM_6, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_1, \
                            }, \
                            .m3 = { \
                                .gpio_num = GPIO_NUM_4, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_2, \
                            }, \
                            .m4 = { \
                                .gpio_num = GPIO_NUM_7, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_3, \
                            }, \
                            .tcp_port = 8080, \
                        }, \
                        .pid_param = { \
                            .pitch_pid = { \
                                .setpoint = 0, \
                                .proportion = 0, \
                                .integration = 0, \
                                .differention = 0, \
                                .max_abs_output = 50, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .max_abs_int_output = 25, \
                            }, \
                            .roll_pid = { \
                                .setpoint = 0, \
                                .proportion = 0, \
                                .integration = 0, \
                                .differention = 0, \
                                .max_abs_output = 50, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .max_abs_int_output = 25, \
                            }, \
                            .yaw_pid = { \
                                .setpoint = 0, \
                                .proportion = 0, \
                                .integration = 0, \
                                .differention = 0, \
                                .max_abs_output = 50, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .max_abs_int_output = 25, \
                            }, \
                        }, \
}

#endif
