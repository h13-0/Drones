#ifndef __ESP_DRONE_CONFIG_H__
#define __ESP_DRONE_CONFIG_H__

#include "esp_drone_private_config.h"

/******************************* driver configs *******************************/
#define ED_MOTOR_MIN_RPS                        (1)




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
                                .k = 203.77, \
                                .c = -136.57, \
                                .min_rps = 1, \
                            }, \
                            .m2 = { \
                                .gpio_num = GPIO_NUM_6, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_1, \
                                .k = 216.21, \
                                .c = -127.61, \
                                .min_rps = 1, \
                            }, \
                            .m3 = { \
                                .gpio_num = GPIO_NUM_4, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_2, \
                                .k = 190.7, \
                                .c = -114.13, \
                                .min_rps = 1, \
                            }, \
                            .m4 = { \
                                .gpio_num = GPIO_NUM_7, \
                                .timer_sel = LEDC_TIMER_0, \
                                .channel = LEDC_CHANNEL_3, \
                                .k = 196.37, \
                                .c = -120.08, \
                                .min_rps = 1, \
                            }, \
                            .tcp_port = 8080, \
                        }, \
                        .pid_param = { \
                            .veloc_pitch = { \
                                .target = 0, \
                                .proportion = 1.5, \
                                .integration = 0.1, \
                                .differention = 2.56, \
                                .max_abs_output = 1000, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .configs.autoResetIntegration = PID_FUNC_DISABLE, \
                                .max_abs_int_output = 400, \
                            }, \
                            .veloc_roll = { \
                                .target = 0, \
                                .proportion = 1.5, \
                                .integration = 0.1, \
                                .differention = 2.56, \
                                .max_abs_output = 1000, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .configs.autoResetIntegration = PID_FUNC_DISABLE, \
                                .max_abs_int_output = 400, \
                            }, \
                            .veloc_yaw = { \
                                .target = 0, \
                                .proportion = 0, \
                                .integration = 0, \
                                .differention = 0, \
                                .max_abs_output = 1000, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .configs.autoResetIntegration = PID_FUNC_DISABLE, \
                                .max_abs_int_output = 250, \
                            }, \
                            .angle_pitch = { \
                                .target = 0, \
                                .proportion = 0.5, \
                                .integration = 0.003, \
                                .differention = 0, \
                                .max_abs_output = 30, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .configs.autoResetIntegration = PID_FUNC_DISABLE, \
                                .max_abs_int_output = 3.15, \
                            }, \
                            .angle_roll = { \
                                .target = 0, \
                                .proportion = 0.5, \
                                .integration = 0.003, \
                                .differention = 0, \
                                .max_abs_output = 30, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .configs.autoResetIntegration = PID_FUNC_DISABLE, \
                                .max_abs_int_output = 3.15, \
                            }, \
                            .angle_yaw = { \
                                .target = 0, \
                                .proportion = 0, \
                                .integration = 0, \
                                .differention = 0, \
                                .max_abs_output = 1000, \
                                .configs.limitIntegration = PID_FUNC_ENABLE, \
                                .max_abs_int_output = 250, \
                            }, \
                        }, \
}

#endif
