#ifndef __OH_QUADROTOR_PID_H__
#define __OH_QUADROTOR_PID_H__

#include "oh_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // attitude angle pid.
    oh_pos_pid_t pitch_pid;
    oh_pos_pid_t roll_pid;
    oh_pos_pid_t yaw_pid;

} oh_quad_pid_t;

/**
 *        clock          anti-clock
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
 *    | anti-clock         clock   
 *    +----------> roll            
 * yaw          x                  
 */
void oh_quad_pid_control_realize(oh_drv_status_t *status, oh_quad_pid_t *pid, oh_drv_quadrotor_output_t *output);


#ifdef __cplusplus
}
#endif

#endif
