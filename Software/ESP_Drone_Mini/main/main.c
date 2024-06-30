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
#include "oh_quadrotor_pid.h"

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

// drv
static ed_drv_t drv = ESP_DRONE;

// temp for debug
static float base_rps = 0;

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

        // copy sensor data
        oh_status.pitch = pitch;
        oh_status.roll = roll;
        oh_status.yaw = yaw;
        oh_status.gx = gx;
        oh_status.gy = gy;
        oh_status.gz = gz;

        // control realize.
        oh_quad_pid_control_realize(&oh_status, &(drv.pid_param), &oh_output);

        // perform output.
        if(base_rps > 1)
        {
            ed_motor_set_rps(&(drv.drivers.m1), oh_output.m1 + base_rps);
            ed_motor_set_rps(&(drv.drivers.m2), oh_output.m2 + base_rps);
            ed_motor_set_rps(&(drv.drivers.m3), oh_output.m3 + base_rps);
            ed_motor_set_rps(&(drv.drivers.m4), oh_output.m4 + base_rps);
        } else {
            ed_motor_set_rps(&(drv.drivers.m1), 0);
            ed_motor_set_rps(&(drv.drivers.m2), 0);
            ed_motor_set_rps(&(drv.drivers.m3), 0);
            ed_motor_set_rps(&(drv.drivers.m4), 0);
        }
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
    ed_debugger_bind_float(0, &base_rps);
    // veloc_roll
    ed_debugger_bind_float(1, &(drv.pid_param.veloc_roll.proportion));
    ed_debugger_bind_float(2, &(drv.pid_param.veloc_roll.integration));
    ed_debugger_bind_float(3, &(drv.pid_param.veloc_roll.differention));
    ed_debugger_bind_float(4, &(drv.pid_param.veloc_roll.max_abs_int_output));
    ed_debugger_bind_float(5, &(drv.pid_param.veloc_roll.target));
    // angle_roll
    ed_debugger_bind_float(6, &(drv.pid_param.angle_roll.proportion));
    ed_debugger_bind_float(7, &(drv.pid_param.angle_roll.integration));
    ed_debugger_bind_float(8, &(drv.pid_param.angle_roll.differention));
    ed_debugger_bind_float(9, &(drv.pid_param.angle_roll.max_abs_int_output));
    ed_debugger_bind_float(10, &(drv.pid_param.angle_roll.target));


    while(1)
    {
        float veloc_int_out = drv.pid_param.veloc_roll._sumError * drv.pid_param.veloc_roll.integration;
        float speed_int_out = drv.pid_param.angle_roll._sumError * drv.pid_param.angle_roll.integration;

        // report sensor data.
        ed_debugger_send_float(pitch, roll, yaw, gx, gy, gz, drv.pid_param.veloc_roll._sumError, veloc_int_out, drv.pid_param.veloc_roll.target, speed_int_out);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

//err:
    while(1) vTaskDelay(pdMS_TO_TICKS(100));

}