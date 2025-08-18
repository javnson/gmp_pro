/**
 * @file slope_limiter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a discrete slope limiter (rate limiter).
 * @version 1,05
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#ifndef _SLOPE_LIMITER_H_
#define _SLOPE_LIMITER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup slope_limiter Slope Limiter
 * @brief A module to constrain the rate of change of a signal.
 * @details This module implements a slope limiter, also known as a rate limiter.
 * It constrains the rate of change of a signal, preventing abrupt jumps in the
 * output. This is commonly used for smooth setpoint changes or to protect
 * physical systems from excessive stress. The output value changes incrementally
 * towards the input value at a rate no greater than the configured maximum slope.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Slope Limiter (Rate Limiter)                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the slope limiter.
 */
typedef struct _tag_slope_limiter_t
{
    ctrl_gt out;       //!< The current, rate-limited output value.
    ctrl_gt slope_max; //!< The maximum allowed increase per step.
    ctrl_gt slope_min; //!< The maximum allowed decrease per step (a negative value).
} ctl_slope_limiter_t;

/**
 * @brief Initializes the slope limiter module.
 * @param[out] obj Pointer to the slope limiter instance.
 * @param[in] slope_max The maximum positive rate of change (e.g., units/sec).
 * @param[in] slope_min The maximum negative rate of change (e.g., -units/sec).
 * @param[in] fs The sampling frequency (Hz) at which this module will be called.
 */
void ctl_init_slope_limiter(ctl_slope_limiter_t* obj, parameter_gt slope_max, parameter_gt slope_min, parameter_gt fs);

/**
 * @brief Executes one step of the slope limiter.
 * @details Calculates the difference between the input and the current output,
 * limits this change by the configured slopes, and applies the limited change
 * to the output.
 * @param[in,out] obj Pointer to the slope limiter instance.
 * @param[in] input The target value for the output.
 * @return ctrl_gt The new, rate-limited output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_slope_limiter(ctl_slope_limiter_t* obj, ctrl_gt input)
{
    // Calculate the desired change
    ctrl_gt delta = input - obj->out;

    // Limit the change by the maximum and minimum slope
    delta = ctl_sat(delta, obj->slope_max, obj->slope_min);

    // Apply the limited change to the output
    obj->out = obj->out + delta;

    return obj->out;
}

/**
 * @brief Sets new slope limits for the module.
 * @param[out] obj Pointer to the slope limiter instance.
 * @param[in] slope_max The new maximum positive rate of change per step.
 * @param[in] slope_min The new maximum negative rate of change per step.
 */
GMP_STATIC_INLINE void ctl_set_slope_limiter_slopes(ctl_slope_limiter_t* obj, ctrl_gt slope_max, ctrl_gt slope_min)
{
    obj->slope_max = slope_max;
    obj->slope_min = slope_min;
}

/**
 * @brief Sets the current output value of the limiter directly.
 * @details This function bypasses the slope limit and forces the output to a
 * specific value. Useful for initialization or re-synchronization.
 * @param[out] obj Pointer to the slope limiter instance.
 * @param[in] current The value to which the output should be set.
 */
GMP_STATIC_INLINE void ctl_set_slope_limiter_current(ctl_slope_limiter_t* obj, ctrl_gt current)
{
    obj->out = current;
}

/**
 * @brief Clears the output of the slope limiter.
 * @details Resets the output value to 0.
 * @param[out] obj Pointer to the slope limiter instance.
 */
GMP_STATIC_INLINE void ctl_clear_slope_limiter(ctl_slope_limiter_t* obj)
{
    obj->out = 0;
}

/**
 * @}
 */ // end of slope_limiter group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _SLOPE_LIMITER_H_
