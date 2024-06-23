#ifndef _OH_PID_H_
#define _OH_PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @group: Basic PID(Position PID only).
 * @note:  Try not to use basic PID because it doesn't work well in most cases.
 */

/**
 * @brief: Basic PID typedef struct.
 * @param:
 * 		float setpoint:       Target value of PID control system.
 * 		float proportion:     PID proportional coefficient.
 * 		float integration:    PID integral coefficient.
 * 		float differention:   PID differential coefficient.
 * 		float max_abs_output: Maximum absolute value of output.
 */
typedef struct
{
	//Basic parameters of PID
	float setpoint;

	float proportion;
	float integration;
	float differention;

	float max_abs_output;

	//private realizations.
	float _sumError;
	float _error;
} oh_basic_pid_t;

/**
 * @brief:  Basic PIC Calculate.
 * @param:
 * 		oh_basic_pid_t *pid:    PositionPID struct.
 * 		float curr_point: Current system status.
 * @return:
 * 		Calculation result.
 */
float oh_basic_pid_calc(oh_basic_pid_t *pid, float curr_point);

/**
 * @group: Position PID.
 */
typedef enum
{
	PID_FUNC_DISABLE = 0,
	PID_FUNC_ENABLE  = 1,
} oh_pid_func_status_t;

/**
 * @brief: Position PID typedef struct.
 * @param:
 * 		@Basic parameters are same as Basic PID.
 * 		@configs:
 * 			oh_pid_func_status_t limitIntegration:     Limit sum error.
 * 			oh_pid_func_status_t autoResetIntegration: Automatically reset integration when the current value crosses the setpoint.
 * 			float max_abs_int_output:  Integration term output limit.
 */
typedef struct
{
	//Basic parameters of PID
	float setpoint;

	float proportion;
	float integration;
	float differention;

	float max_abs_output;

	//private realizations.
	float _sumError;
	float _error;

	//Expand functions.
	struct
	{
		//integration term limiting.
		oh_pid_func_status_t limitIntegration : 1;

		//Automatically reset integration when crossing setpoint.
		oh_pid_func_status_t autoResetIntegration : 1;

		//private realizations.
		uint8_t _lastResetIntergrationStatus : 1;
		uint8_t _reserved : 5;
	} configs;

	//Expand functions realization.
	float max_abs_int_output;
} oh_pos_pid_t;

/**
 * @brief: Position PID calculate with expand functions.
 * @param:
 * 		oh_pos_pid_t *pid: Position PID struct.
 * 		float curr_point:  Current system status.
 * @return:
 * 		Calculation result.
 */
float oh_pos_pid_calc(oh_pos_pid_t *pid, float curr_point);

/**
 * @brief: Position PID calculate with expand functions.
 * @param:
 * 		oh_pos_pid_t *pid:       Position PID struct.
 * 		float curr_point:        Current system status.
 * 		float curr_diff: Customed differention status.
 * @return:
 * 		Calculation results.
 * @note:
 * 		This function is suitable for the control system of sensor with differential characteristic.
 * 		Such as when you have an angular velocimeter and want to control the angle.
 */
float oh_pos_pid_calc_with_diff(oh_pos_pid_t *pid, float curr_point, float curr_diff);

/**
 * @brief: Position PID calculate with expand functions.
 * @param:
 * 		oh_pos_pid_t *pid: Position PID struct.
 * 		float curr_err:  Customed error.
 * @return:
 * 		Calculation results.
 * @note:
 * 		This function is suitable for continuous control systems with sudden change points.
 * 		Such as in the yaw Angle control plus or minus 180 degree problem.
 */
float oh_pos_pid_calc_with_err(oh_pos_pid_t *pid, float curr_err);

/**
 * @brief: Position PID calculate with expand functions.
 * @param:
 * 		oh_pos_pid_t *pid:       Position PID struct.
 * 		float curr_err:        Customed error.
 * 		float curr_diff: Customed differention status.
 * @return:
 * 		Calculation results.
 * @note:
 * 		This function is suitable for continuous control systems with sudden change points and have a sensor with differential characteristic.
 */
float oh_pos_pid_calc_with_err_diff(oh_pos_pid_t *pid, float curr_err, float curr_diff);

/**
 * @group: Incremental PID.
 */

/**
 * @brief: Incremental PID typedef struct.
 * @param:
 * 		@Basic parameters are same as Basic PID.
 */
typedef struct {
	//Basic parameters of PID
	float setpoint;

	float proportion;
	float integration;
	float differention;

	float max_abs_output;

	//private realizations.
	float _previousError;
	float _lastError;
} oh_inc_pid_t;

/**
 * @brief: Incremental PID calculate.
 * @param:
 * 		oh_inc_pid_t *pid: Incremental PID struct.
 * 		float curr_point:      Current system status.
 * @return:
 * 		Calculation results.
 * @note:
 * 		You need to integrate and limit the output.
 */
float oh_inc_pid_calc(oh_inc_pid_t *pid, float curr_point);

#ifdef __cplusplus
}
#endif

#endif
