/**
 * @file discrete_pid.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a discrete PID controller implementation.
 * @version 0.4
 * @date 2024-03-20
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _DISCRETE_PID_H_
#define _DISCRETE_PID_H_

#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup discrete_pid_controller Discrete PID Controller
 * @brief This module implements a standard discrete PID controller.
 * @details This module provides a standard discrete Proportional-Integral-Derivative
 * (PID) controller. The implementation is based on a difference equation derived
 * from the continuous-time PID formula using a combination of Tustin and
 * Backward Euler transformations for robust performance. It includes functions
 * for initialization, state clearing, setting output limits, and executing a
 * single control step.

 * @{
 */

/*---------------------------------------------------------------------------*/
/* Discrete Proportional-Integral-Derivative (PID) Controller                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Controller theory and discretization formulas.
 *
 * The ideal PID controller transfer function in the continuous domain is:
 * @f[ G(s) = k_p + \frac{k_i}{s} + k_d s @f]
 * Where the integral time constant is @f$ T_i = \frac{k_p}{k_i} @f$ and the derivative time constant is @f$ T_d = \frac{k_d}{k_p} @f$.
 *
 * The derivative term is discretized using the Tustin (Bilinear) transform:
 * @f[ s = 2f_s\frac{1-z^{-1}}{1+z^{-1}} @f]
 * The integral term is discretized using the Backward Euler method:
 * @f[ s = f_s (1-z^{-1}) @f]
 */

/**
 * @brief Structure for PID controller tuning parameters.
 * @note This structure is defined for clarity but is not directly used by any function
 * in the current implementation. The @ref ctl_init_discrete_pid function accepts
 * individual parameters instead.
 */
typedef struct _tag_discrete_pid_tuning_t
{
    parameter_gt kp; //!< Proportional Gain
    parameter_gt Ti; //!< Integral Time Constant
    parameter_gt Td; //!< Derivative Time Constant
    parameter_gt fs; //!< Sampling Frequency in Hz
} discrete_pid_tuning_t;

/**
 * @brief State variables and parameters for the discrete PID controller.
 */
typedef struct _tag_discrete_pid_t
{
    // SISO standard interface
    ctrl_gt input;  //!< Controller input at the current time step (u[k])
    ctrl_gt output; //!< Controller output at the current time step (y[k])

    // Difference equation coefficients
    ctrl_gt b0; //!< Coefficient for the input term u[k]
    ctrl_gt b1; //!< Coefficient for the input term u[k-1]
    ctrl_gt b2; //!< Coefficient for the input term u[k-2]

#ifdef _USE_DEBUG_DISCRETE_PID
    ctrl_gt kp; //!< Proportional gain (used for direct calculation in debug mode only)
#endif          // _USE_DEBUG_DISCRETE_PID

    // Historical state variables
    ctrl_gt input_1;  //!< Input value from the previous time step (u[k-1])
    ctrl_gt input_2;  //!< Input value from two time steps ago (u[k-2])
    ctrl_gt output_1; //!< Output value from the previous time step (y[k-1])

    // Output limits
    ctrl_gt output_max; //!< Output saturation upper limit
    ctrl_gt output_min; //!< Output saturation lower limit
} discrete_pid_t;

/**
 * @brief Initializes the discrete PID controller.
 *
 * @param[out] pid Pointer to a discrete_pid_t structure.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant.
 * @param[in] Td Derivative time constant.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_discrete_pid(discrete_pid_t* pid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt fs);

/**
 * @brief Clears the historical states of the PID controller.
 * @details Resets the historical input and output values to zero, which is useful
 * for restarting the control process without re-initialization.
 * @param[out] pid Pointer to a discrete_pid_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_discrete_pid(discrete_pid_t* pid)
{
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output_1 = 0;
}

/**
 * @brief Sets the output limits for the PID controller.
 *
 * @param[out] pid Pointer to a discrete_pid_t structure.
 * @param[in] limit_max The maximum output value (upper saturation limit).
 * @param[in] limit_min The minimum output value (lower saturation limit).
 */
GMP_STATIC_INLINE void ctl_set_discrete_pid_limit(discrete_pid_t* pid, ctrl_gt limit_max, ctrl_gt limit_min)
{
    pid->output_max = limit_max;
    pid->output_min = limit_min;
}

/**
 * @brief Executes one calculation step of the PID controller.
 * @details Calculates the controller output based on the current input and updates the historical states.
 * The output is saturated according to the configured limits.
 *
 * @param[in,out] ctrl Pointer to a discrete_pid_t structure.
 * @param[in] input The current controller input value (e.g., the error signal).
 * @return ctrl_gt The calculated and saturated output value of the controller.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_discrete_pid(discrete_pid_t* ctrl, ctrl_gt input)
{
    ctrl->input = input;

    // Calculate output based on the difference equation:
    // y[k] = y[k-1] + b0*u[k] + b1*u[k-1] + b2*u[k-2]
#ifdef _USE_DEBUG_DISCRETE_PID
    // WARNING: Debug logic differs significantly from standard implementation.
    // It adds an extra proportional term and a decayed output feedback.
    // This may lead to unexpected behavior. Please verify correctness.
    ctrl->output = ctl_mul(ctrl->kp, ctrl->input);
    ctrl->output += ctl_mul(ctrl->b0, ctrl->input);
    ctrl->output += ctl_mul(ctrl->b1, ctrl->input_1);
    ctrl->output += ctl_mul(ctrl->b2, ctrl->input_2);
    ctrl->output += ctl_mul(float2ctrl(0.95), ctrl->output_1);
#else
    // Standard mode
    ctrl->output = ctl_mul(ctrl->b0, ctrl->input);
    ctrl->output += ctl_mul(ctrl->b1, ctrl->input_1);
    ctrl->output += ctl_mul(ctrl->b2, ctrl->input_2);
    ctrl->output += ctrl->output_1;
#endif // _USE_DEBUG_DISCRETE_PID

    // Handle output saturation
    ctrl->output = ctl_sat(ctrl->output, ctrl->output_max, ctrl->output_min);

    // Update historical state variables for the next step
    ctrl->input_2 = ctrl->input_1;
    ctrl->input_1 = ctrl->input;
    ctrl->output_1 = ctrl->output;

    return ctrl->output;
}

/**
 * @brief Gets the most recent output value from the PID controller.
 *
 * @param[in] ctrl Pointer to a discrete_pid_t structure.
 * @return ctrl_gt The last calculated output value of the controller.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_discrete_pid_output(discrete_pid_t* ctrl)
{
    return ctrl->output;
}

/**
 * @}
 */ // end of discrete_pid_controller group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _DISCRETE_PID_H_
