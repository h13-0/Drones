#include "quadrotor_pid.h"

#include "oh_pid.h"

/**
 *       +------+         +------+
 *       |      |         |      |
 *       |  M3  |         |  M1  |
 *       |      |         |      |
 *       +------+---------+------+
 *              |         |       
 *              |         |       
 *              |         |       
 *  pitch       |         |       
 *    ^         |         |       
 *   y|  +------+---------+------+
 *    |  |      |         |      |
 *    |  |  M2  |         |  M4  |
 *    |  |      |         |      |
 *    |  +------+         +------+
 *    |                           
 *    +----------> roll           
 * yaw          x                 
 */

static __oh_drv_debug_target_status_t target = { 0 };

static oh_pos_pid_t pitch_pid = {
    .setpoint = 0,
    .proportion = 0,
    .integration = 0,
    .differention = 0,
    .max_abs_output = 50,
    .configs.limitIntegration = PID_FUNC_ENABLE,
    .max_abs_int_output = 25,
};

void oh_quad_pid_control_realize(oh_drv_status_t *status, oh_quad_pid_t *pid, oh_drv_quadrotor_output_t *output)
{
    static float pitch_diff = 0;
    static float yaw_diff = 0;

    pitch_diff = oh_pos_pid_calc_with_diff(&pitch_pid, status->pitch, status->gy);
    yaw_diff = oh_pos_pid_calc_with_diff(&pitch_pid, status->pitch, status->gy);

    // calculate output
    if(pitch_diff > 0)
    {
        output->m1 = pitch_diff;
        output->m4 = pitch_diff;
        output->m2 = 0;
        output->m3 = 0;
    } else {
        output->m1 = 0;
        output->m4 = 0;
        output->m2 = -pitch_diff;
        output->m3 = -pitch_diff;
    }
}

