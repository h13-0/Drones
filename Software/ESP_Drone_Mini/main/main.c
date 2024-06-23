#include "esp_drone_config.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ed_drivers.h"
#include "ed_debugger.h"
#include "ed_imu.h"
#include "quadrotor_pid.h"

static const char* tag = "app";

// task handles.
static TaskHandle_t motion_control_task_handle = NULL;
static const UBaseType_t imu_ready_index = 0;

// sensor data.
static float pitch = 0, roll = 0, yaw = 0;
static float ax = 0, ay = 0, az = 0;
static float gx = 0, gy = 0, gz = 0;

// open hover
static oh_drv_status_t oh_status = { 0 };
static oh_drv_quadrotor_output_t oh_output = { 0 };

static ed_drv_t drv = ESP_DRONE;

void IRAM_ATTR imu_int_handler(void *args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveIndexedFromISR(motion_control_task_handle, imu_ready_index, &xHigherPriorityTaskWoken);
}


void motion_control_task(void *pvParameters)
{
    // Count the number of consecutive failures of imu.
    int imu_eular_failed_times = 0;

    for( ;; )
    {
        // waiting for imu ready, its frequence is 100hz.
        ulTaskNotifyTakeIndexed(imu_ready_index, pdTRUE, pdMS_TO_TICKS(10)); //pdMS_TO_TICKS(10) or portMAX_DELAY
        
        // get eular angle.
        if(ed_imu_get_eular(&pitch, &roll, &yaw) == 0)
            imu_eular_failed_times = 0;
        else
            imu_eular_failed_times ++;
        
        // get accel and gyro
        ed_imu_get_accel(&ax, &ay, &az);
        ed_imu_get_gyro(&gx, &gy, &gz);

        // copy sensor data(TODO)
        oh_status.pitch = pitch;
        oh_status.roll = roll;
        oh_status.yaw = yaw;

        // control realize.
        quad_pid_control_realize(&oh_status, &oh_output);

        // perform output.
        ed_motor_set_duty(&(drv.drivers.m1), oh_output.m1);
        ed_motor_set_duty(&(drv.drivers.m2), oh_output.m2);
        ed_motor_set_duty(&(drv.drivers.m3), oh_output.m3);
        ed_motor_set_duty(&(drv.drivers.m4), oh_output.m4);
    }
    vTaskDelete( NULL );
}

void app_main(void)
{
    // init all drivers.
    ed_drivers_init(&(drv.drivers)); 

    // start motion control
    xTaskCreate(
        motion_control_task,
        "motion control",
        2048,
        NULL,
        5,       // uxPriority = 5
        &motion_control_task_handle
    );

    // bind parameters to debugger.

    while(1)
    {
        // report sensor data.
        ESP_LOGI(tag, "Pitch: %5.3lf, roll: %5.3lf, yaw: %5.3lf, ax: %5.3lf, ay: %5.3lf, az: %5.3lf", pitch, roll, yaw, ax, ay, az);
        //ESP_LOGI(tag, "gx: %5.3lf, gy: %5.3lf, gz: %5.3lf", gx, gy, gz);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

//err:
    while(1) vTaskDelay(pdMS_TO_TICKS(100));

}