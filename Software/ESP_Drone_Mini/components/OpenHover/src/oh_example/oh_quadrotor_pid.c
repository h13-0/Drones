#include "oh_quadrotor_pid.h"

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

void oh_quad_pid_control_realize(oh_drv_status_t *status, oh_quad_pid_t *pid, oh_drv_quadrotor_output_t *output)
{
    static float base_rps = 0;

    static float pitch_diff = 0;
    static float roll_diff = 0;
    static float yaw_diff = 0;

    // calc angle pids
    pid->veloc_pitch.target = oh_pos_pid_calc_with_diff(&pid->angle_pitch, status->pitch, status->gy);
    pid->veloc_roll.target = oh_pos_pid_calc_with_diff(&pid->angle_roll, status->roll, status->gx);

    // calc angular velocity pids
    pitch_diff = oh_pos_pid_calc(&pid->veloc_pitch, status->gy);
    roll_diff = oh_pos_pid_calc(&pid->veloc_roll, status->gx);
    yaw_diff = oh_pos_pid_calc(&pid->veloc_yaw, status->gz);

    // calculate output
    // output base_rps
    output->m1 = base_rps;
    output->m4 = base_rps;
    output->m2 = base_rps;
    output->m3 = base_rps;

    // output pitch_diff
    output->m1 += - pitch_diff / 2;
    output->m4 += - pitch_diff / 2;
    output->m2 += pitch_diff / 2;
    output->m3 += pitch_diff / 2;

    // output roll_diff
    output->m2 += - roll_diff / 2;
    output->m4 += - roll_diff / 2;
    output->m1 += roll_diff / 2;
    output->m3 += roll_diff / 2;

    // Offset output to ensure differential value is achieved.
    // Find the minimum output value
    float minimun_rps = output->m1;
    if(output->m2 < minimun_rps) minimun_rps = output->m2;
    if(output->m3 < minimun_rps) minimun_rps = output->m3;
    if(output->m4 < minimun_rps) minimun_rps = output->m4;
    // outputs += fabs(minimun_rps)
    if(minimun_rps < 0)
    {
        output->m1 -= minimun_rps;
        output->m2 -= minimun_rps;
        output->m3 -= minimun_rps;
        output->m4 -= minimun_rps;
    }
}

