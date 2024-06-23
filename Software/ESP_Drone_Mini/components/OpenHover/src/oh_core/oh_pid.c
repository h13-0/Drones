#include "oh_pid.h"

#include <math.h>

/**
 * @brief:  Basic PIC Calculate.
 * @param:
 * 		oh_basic_pid_t *pid:    PositionPID struct.
 * 		float curr_point: Current system status.
 * @return:
 * 		Calculation result.
 */
float oh_basic_pid_calc(oh_basic_pid_t *pid, float curr_point)
{
	float error = pid->setpoint - curr_point;
	float result;

	pid -> _sumError += error;

	result =
		//proportion * error +
		pid -> proportion * (error) +
		//integration * sumerror +
		pid -> integration * pid -> _sumError +
		//differention * error'
		pid -> differention * (error - pid -> _error);

	pid -> _error = error;

	//Check the output.
	if (!isnormal(result))
	{
		return 0;
	} else {
		if(result > (pid -> max_abs_output))
		{
			return pid -> max_abs_output;
		} else if(result < - pid -> max_abs_output)
		{
			return - pid -> max_abs_output;
		}
	}

	return result;
}

/**
 * @brief: Position PID calculate with expand functions.
 * @param:
 * 		oh_pos_pid_t *pid: Position PID struct.
 * 		float curr_point:  Current system status.
 * @return:
 * 		Calculation result.
 */
float oh_pos_pid_calc(oh_pos_pid_t *pid, float curr_point)
{
	float error = pid->setpoint - curr_point;
	float result;

	pid -> _sumError += error;

	//Expand functions.
	//Auto reset integration.
	if(pid -> configs.autoResetIntegration == PID_FUNC_ENABLE)
	{
		if((error > 0) && (pid -> configs._lastResetIntergrationStatus == 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 1;
		}else if((error < 0) && (pid -> configs._lastResetIntergrationStatus != 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 0;
		}
	}

	//Calculate the integral output.
	result = pid -> integration * pid -> _sumError;

	//Expand functions.
	//Limit integration.
	if(pid -> configs.limitIntegration == PID_FUNC_ENABLE)
	{
		if(isnormal(result))
		{
			if(result > pid -> max_abs_int_output)
			{
				result = pid -> max_abs_int_output;
				pid -> _sumError = pid -> max_abs_int_output / pid -> integration;
			} else if(result < - pid -> max_abs_int_output)
			{
				result = - pid -> max_abs_int_output;
				pid -> _sumError = - (pid -> max_abs_int_output / pid -> integration);
			}
		} else {
			result = 0;
			pid -> _sumError = 0;
		}
	}

	//Calculate Output
	result =
		//proportion * error +
		pid -> proportion * (error) +
		//integral output +
		result +
		//differention * error'
		pid -> differention * (error - pid -> _error);

	//Update error.
	pid -> _error = error;

	//Check the output.
	if (!isnormal(result))
	{
		return 0;
	} else {
		if(result > (pid -> max_abs_output))
		{
			return pid -> max_abs_output;
		} else if(result < - pid -> max_abs_output)
		{
			return - pid -> max_abs_output;
		}
	}

	return result;
}

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
float oh_pos_pid_calc_with_diff(oh_pos_pid_t *pid, float curr_point, float curr_diff)
{
	float error = pid->setpoint - curr_point;
	float result;

	pid -> _sumError += error;

	//Expand functions.
	//Auto reset integration.
	if(pid -> configs.autoResetIntegration == PID_FUNC_ENABLE)
	{
		if((error > 0) && (pid -> configs._lastResetIntergrationStatus == 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 1;
		}else if((error < 0) && (pid -> configs._lastResetIntergrationStatus != 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 0;
		}
	}

	//Calculate the integral output.
	result = pid -> integration * pid -> _sumError;

	//Expand functions.
	//Limit integration.
	if(pid -> configs.limitIntegration == PID_FUNC_ENABLE)
	{
		if(isnormal(result))
		{
			if(result > pid -> max_abs_int_output)
			{
				result = pid -> max_abs_int_output;
				pid -> _sumError = pid -> max_abs_int_output / pid -> integration;
			} else if(result < - pid -> max_abs_int_output)
			{
				result = - pid -> max_abs_int_output;
				pid -> _sumError = - (pid -> max_abs_int_output / pid -> integration);
			}
		} else {
			result = 0;
			pid -> _sumError = 0;
		}
	}

	//Calculate Output
	result =
		//proportion * error +
		pid -> proportion * (error) +
		//integral output +
		result +
		//differention * error'
		pid -> differention * curr_diff;

	//Update error.
	pid -> _error = error;

	//Check the output.
	if (!isnormal(result))
	{
		return 0;
	} else {
		if(result > (pid -> max_abs_output))
		{
			return pid -> max_abs_output;
		} else if(result < - pid -> max_abs_output)
		{
			return - pid -> max_abs_output;
		}
	}

	return result;
}

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
float oh_pos_pid_calc_with_err(oh_pos_pid_t *pid, float curr_err)
{
	float result;

	pid -> _sumError += curr_err;

	//Expand functions.
	//Auto reset integration.
	if(pid -> configs.autoResetIntegration == PID_FUNC_ENABLE)
	{
		if((curr_err > 0) && (pid -> configs._lastResetIntergrationStatus == 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 1;
		}else if((curr_err < 0) && (pid -> configs._lastResetIntergrationStatus != 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 0;
		}
	}

	//Calculate the integral output.
	result = pid -> integration * pid -> _sumError;

	//Expand functions.
	//Limit integration.
	if(pid -> configs.limitIntegration == PID_FUNC_ENABLE)
	{
		if(isnormal(result))
		{
			if(result > pid -> max_abs_int_output)
			{
				result = pid -> max_abs_int_output;
				pid -> _sumError = pid -> max_abs_int_output / pid -> integration;
			} else if(result < - pid -> max_abs_int_output)
			{
				result = - pid -> max_abs_int_output;
				pid -> _sumError = - (pid -> max_abs_int_output / pid -> integration);
			}
		} else {
			result = 0;
			pid -> _sumError = 0;
		}
	}

	//Calculate Output
	result =
		//proportion * error +
		pid -> proportion * (curr_err) +
		//integral output +
		result +
		//differention * error'
		pid -> differention * (curr_err - pid -> _error);

	//Update error.
	pid -> _error = curr_err;

	//Check the output.
	if (!isnormal(result))
	{
		return 0;
	} else {
		if(result > (pid -> max_abs_output))
		{
			return pid -> max_abs_output;
		} else if(result < - pid -> max_abs_output)
		{
			return - pid -> max_abs_output;
		}
	}

	return result;
}

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
float oh_pos_pid_calc_with_err_diff(oh_pos_pid_t *pid, float curr_err, float curr_diff)
{
	float result;

	pid -> _sumError += curr_err;

	//Expand functions.
	//Auto reset integration.
	if(pid -> configs.autoResetIntegration == PID_FUNC_ENABLE)
	{
		if((curr_err > 0) && (pid -> configs._lastResetIntergrationStatus == 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 1;
		}else if((curr_err < 0) && (pid -> configs._lastResetIntergrationStatus != 0))
		{
			pid -> _sumError = 0;
			pid -> configs._lastResetIntergrationStatus = 0;
		}
	}

	//Calculate the integral output.
	result = pid -> integration * pid -> _sumError;

	//Expand functions.
	//Limit integration.
	if(pid -> configs.limitIntegration == PID_FUNC_ENABLE)
	{
		if(isnormal(result))
		{
			if(result > pid -> max_abs_int_output)
			{
				result = pid -> max_abs_int_output;
				pid -> _sumError = pid -> max_abs_int_output / pid -> integration;
			} else if(result < - pid -> max_abs_int_output)
			{
				result = - pid -> max_abs_int_output;
				pid -> _sumError = - (pid -> max_abs_int_output / pid -> integration);
			}
		} else {
			result = 0;
			pid -> _sumError = 0;
		}
	}

	//Calculate Output
	result =
		//proportion * error +
		pid -> proportion * (curr_err) +
		//integral output +
		result +
		//differention * error'
		pid -> differention * curr_diff;

	//Update error.
	pid -> _error = curr_err;

	//Check the output.
	if (!isnormal(result))
	{
		return 0;
	} else {
		if(result > (pid -> max_abs_output))
		{
			return pid -> max_abs_output;
		} else if(result < - pid -> max_abs_output)
		{
			return - pid -> max_abs_output;
		}
	}

	return result;
}

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
float oh_inc_pid_calc(oh_inc_pid_t *pid, float curr_point)
{
	float error = pid->setpoint - curr_point;
	float result;

	result =
		//proportion * errpr' -> proportion * (error - _lastError) +
		(pid -> proportion) * (error - pid -> _lastError) +
		//integration * error -> I * error +
		(pid -> integration) * (error) +
		//differention * error'' -> differention * [(error + _previousError) - 2 * _lastError]
		(pid -> differention) * ((error + pid -> _previousError) - 2 * (pid -> _lastError));

	//Update errors.
	pid -> _previousError = pid -> _lastError;
	pid -> _lastError = error;

	if(!isnormal(result))
	{
		return 0;
	} else {
		if(result > pid -> max_abs_output)
		{
			return pid -> max_abs_output;
		} else if(result < - pid -> max_abs_output)
		{
			return - pid -> max_abs_output;
		}
	}

	return result;
}
