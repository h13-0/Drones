#ifndef __ED_MOTOR_H__
#define __ED_MOTOR_H__

#include "driver/ledc.h"

typedef struct {
    // peripherals
    int gpio_num;
    ledc_timer_t timer_sel;
    ledc_channel_t channel;

    // calibration
    // NOTE: The motor must undergo speed calibration and be fitted logarithmically to the following formula
    //          rps = k*exp(duty) + c
    float k;
    float c;
    float min_rps;
} ed_motor_t;


/**
 * @brief: initialize the motor and its peripherals.
 * @param: handle of the motor
 * @return: 0 if success.
 */
int ed_motor_init(ed_motor_t* motor);


/**
 * @brief: set the duty cycle of the motor.
 * @param:
 *      - ed_motor_t* motor : handle of the motor
 *      - float duty        : range: [0, 100]
 * @return: 0 if success.
 */
int ed_motor_set_duty(ed_motor_t* motor, float duty);


/**
 * @brief: set the rps of the motor.
 * @param:
 *      - ed_motor_t* motor : handle of the motor
 *      - float rps         : range: [0, 100]
 * @return: 0 if success.
 */
int ed_motor_set_rps(ed_motor_t* motor, float rps);


/**
 * @brief: release and reset the peripherals of motor.
 * @param: ed_motor_t* motor : handle of the motor
 * @note: this function will not actively release memory.
 */
void ed_motor_deinit(ed_motor_t* motor);

#endif
