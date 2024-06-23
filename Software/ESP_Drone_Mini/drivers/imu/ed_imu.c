#include "ed_imu.h"

#include "esp_log.h"

#if(IMU_SELECT == IMU_MPU6050)
#include "mpu6050/inv_mpu.h"
#endif

static const char* tag = "ed_imu";

int ed_imu_init(i2c_port_t i2c_port, uint16_t frequency, int retry)
{
    mpu_err_t ret = MPU_OK;

#if(IMU_SELECT == IMU_MPU6050)
    while(retry > 0)
    {
        ret = mpu_simp_init(i2c_port, frequency);
        if(ret == MPU_OK)
        {
            ESP_LOGI(tag, "mpu6050 init success.");
            break;
        } else {
            vTaskDelay(pdMS_TO_TICKS(300));
            retry--;
        }
        ESP_LOGE(tag, "mpu6050 init failed with ret: %d.", ret);
    }
#endif

    return ret;
}


int ed_imu_int_enable(gpio_num_t int_io, gpio_isr_t int_handler)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ull << int_io,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(int_io, int_handler, NULL);

    set_int_enable(1);
    
    return 0;
}

int ed_imu_get_eular(float *pitch, float *roll, float *yaw)
{
    int ret = 0;
#if(IMU_SELECT == IMU_MPU6050)
    if((ret = mpu_simp_get_eular(pitch, roll, yaw)))
        ESP_LOGE(tag, "mpu6050 get eular failed with ret: %d.", ret);
#endif
    return ret;
}

int ed_imu_get_gyro(float *gx, float *gy, float *gz)
{
    int ret = 0;
#if(IMU_SELECT == IMU_MPU6050)
    if((ret = mpu_simp_get_gyro(gx, gy, gz)))
        ESP_LOGE(tag, "mpu6050 get gyro failed with ret: %d.", ret);
#endif
    return ret;
}

int ed_imu_get_accel(float *ax, float *ay, float *az)
{
    int ret = 0;
#if(IMU_SELECT == IMU_MPU6050)
    if((ret = mpu_simp_get_accel(ax, ay, az)))
        ESP_LOGE(tag, "mpu6050 get accel failed with ret: %d.", ret);
#endif
    return ret;
}


