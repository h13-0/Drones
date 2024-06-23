#include "ed_drivers.h"

// periperals
#include "peripherals/ed_idf_i2c.h"

// esp idf libraries
#include "esp_log.h"
#include "esp_check.h"

// drivers
#include "ed_debugger.h"
#include "ed_imu.h"
#include "ed_motor.h"
#include "ed_nvs_flash.h"
#include "ed_wifi.h"

static const char* tag = "ed_drivers";

int ed_drivers_init(ed_drivers_config_t *config)
{
    int ret = 0;
    // init nvs_flash
    ed_nvs_flash_init();

    // init wifi.
    ed_wifi_sta_connect(config->sta_ssid, config->sta_pswd);

    // init i2c0
    if((ret = ed_idf_i2c_init(config->i2c_num, config->sda_io_num, config->scl_io_num))) {
        ESP_LOGE(tag, "i2c bus init failed, error code: %d", ret);
        goto i2c_failed;
    }
    ESP_LOGI(tag, "i2c0 init success.");
    
    // init imu
    if((ret = ed_imu_init(config->i2c_num, config->imu_freq, config->imu_retry_times))) {
        ESP_LOGE(tag, "imu init failed, error code: %d", ret);
        goto imu_failed;
    }
    ESP_LOGI(tag, "imu init success.");

    // init motors
    ESP_GOTO_ON_ERROR(
        (ret = ed_motor_init(&(config->m1))),
        motor_failed,
        tag,
        "execute ed_motor_init failed, ret: %d", ret
    );
    ESP_GOTO_ON_ERROR(
        (ret = ed_motor_init(&(config->m2))),
        motor_failed,
        tag,
        "execute ed_motor_init failed, ret: %d", ret
    );
    ESP_GOTO_ON_ERROR(
        (ret = ed_motor_init(&(config->m3))),
        motor_failed,
        tag,
        "execute ed_motor_init failed, ret: %d", ret
    );
    ESP_GOTO_ON_ERROR(
        (ret = ed_motor_init(&(config->m4))),
        motor_failed,
        tag,
        "execute ed_motor_init failed, ret: %d", ret
    );
    

    // init debugger.
    ed_debugger_create(config->tcp_port);
    return 0;

motor_failed:
    ed_motor_deinit(&(config->m1));
    ed_motor_deinit(&(config->m2));
    ed_motor_deinit(&(config->m3));
    ed_motor_deinit(&(config->m4));

imu_failed:
    

i2c_failed:


    return ret;
}
