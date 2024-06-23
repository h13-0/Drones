#include "ed_motor.h"

#include "esp_log.h"
#include "esp_check.h"

static const char* tag = "ed_motor";

/**
 * @brief: initialize the motor and its peripherals.
 * @param: handle of the motor
 * @return: 0 if success.
 */
int ed_motor_init(ed_motor_t* motor)
{
    esp_err_t ret = ESP_OK;
    ledc_timer_config_t ledc_timer = { 
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = motor->timer_sel,
    };
    ESP_RETURN_ON_FALSE(
        (ret = ledc_timer_config(&ledc_timer)) == ESP_OK,
        -1,
        tag,
        "execute ledc_timer_config failed, ret: %s,", esp_err_to_name(ret)
    );

    ledc_channel_config_t ledc_channel = { 
        .channel =	motor->channel,
        .duty = 0, 
        .gpio_num = motor->gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = motor->timer_sel,
    };
    ESP_RETURN_ON_FALSE(
        (ret = ledc_channel_config(&ledc_channel)) == ESP_OK,
        -2,
        tag,
        "execute ledc_channel_config failed, ret: %s,", esp_err_to_name(ret)
    );
    return 0;
}


/**
 * @brief: set the duty cycle of the motor.
 * @param:
 *      - ed_motor_t* motor : handle of the motor
 *      - float duty        : range: [0, 100]
 * @return: 0 if success.
 */
int ed_motor_set_duty(ed_motor_t* motor, float duty)
{
    if(duty > 100) duty = 100;
    if(duty < 0) duty = 0;

    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(
        (ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, (uint32_t)(duty * 8191))) == ESP_OK,
        -1,
        tag,
        "execute ledc_set_duty failed, ret: %s", esp_err_to_name(ret)
    );
    ESP_RETURN_ON_FALSE(
        (ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel)) == ESP_OK,
        -2,
        tag,
        "execute ledc_update_duty failed, ret: %s", esp_err_to_name(ret)
    );

    return 0;
}

/**
 * @brief: release and reset the peripherals of motor.
 * @param: ed_motor_t* motor : handle of the motor
 * @note: this function will not actively release memory.
 */
void ed_motor_deinit(ed_motor_t* motor)
{
    ed_motor_set_duty(motor, 0);
}
