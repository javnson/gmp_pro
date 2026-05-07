/**
 * @file repetitive_controller.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a discrete Repetitive Controller (RC) core module.
 * @version 1.0
 * @date 2025-08-07
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <ctl/component/intrinsic/discrete/biquad_filter.h>

#ifndef _REPETITIVE_CONTROLLER_H_
#define _REPETITIVE_CONTROLLER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Constant-Q Repetitive Controller (RC) Core Generator                      */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Constant-Q Repetitive Controller core.
 * @details A simplified version of FDRC where the Q(z) filter is replaced 
 * by a constant attenuation scalar (q_gain < 1.0).
 */
typedef struct _tag_repetitive_controller_t
{
    ctrl_gt* buffer;          //!< Circular buffer to store past outputs (the delay line).
    uint32_t buffer_capacity; //!< Total capacity of the allocated buffer array.
    uint32_t buffer_index;    //!< Current write index for the circular buffer.

    int32_t phase_lead_k; //!< Phase lead steps to compensate for plant delay.
    ctrl_gt q_gain;       //!< Constant Q factor (e.g., 0.95) for robustness.
    ctrl_gt k_rc;         //!< Learning gain of the RC.

    parameter_gt fs;          //!< Controller frequency, Hz
    parameter_gt f_min_rated; //!< Rated minimum frequency, Hz

    // Output Limits
    ctrl_gt out_max; //!< Maximum output limit for anti-windup.
    ctrl_gt out_min; //!< Minimum output limit for anti-windup.

    ctrl_gt output; //!< The last calculated controller output.

    // Control Flags
    fast_gt flag_enable_rc_integrating; //!< Flag to enable/disable integrating new errors.
} ctl_rc_t;

/*---------------------------------------------------------------------------*/
/* Inline Helper Functions                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Macro to calculate the absolute minimum required buffer capacity.
 * @details Adds a margin of 10 elements to prevent index boundary overflow 
 * during extreme frequency deviations and phase lead calculations.
 * @param fs System sampling frequency (Hz).
 * @param f_min Minimum expected grid/fundamental frequency (Hz).
 */
#ifndef CTL_RC_CALC_MIN_CAPACITY
#define CTL_RC_CALC_MIN_CAPACITY(fs, f_min) ((uint32_t)((fs) / (f_min)) + 10U)
#endif // CTL_RC_CALC_MIN_CAPACITY

/**
 * @brief Fast circular buffer index wrapping for ctl_rc_t.
 */
GMP_STATIC_INLINE uint32_t _ctl_rc_wrap_index(int32_t index, uint32_t capacity)
{
    if (index < 0)
    {
        return (uint32_t)(index + (int32_t)capacity);
    }
    else if ((uint32_t)index >= capacity)
    {
        return (uint32_t)(index - (int32_t)capacity);
    }
    return (uint32_t)index;
}

/*---------------------------------------------------------------------------*/
/* Function Prototypes and Inline Implementations                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the Constant-Q Repetitive Controller instance.
 * @param[out] obj Pointer to the RC controller instance.
 * @param[in] buffer Pointer to the externally allocated memory buffer.
 * @param[in] capacity Size of the provided buffer.
 * @param[in] fs Sampling frequency in Hz.
 * @param[in] f_min Minimum expected fundamental frequency in Hz.
 * @param[in] q_gain Constant Q attenuation factor (typically 0.90 ~ 0.99).
 * @param[in] k_rc Learning gain.
 * @param[in] phase_lead_k Phase lead step count.
 */
void ctl_init_rc(ctl_rc_t* obj, ctrl_gt* buffer, uint32_t capacity, parameter_gt fs, parameter_gt f_min,
                 parameter_gt q_gain, parameter_gt k_rc, int32_t phase_lead_k);

/**
 * @brief Clears the internal states and memory buffer of the RC.
 */
GMP_STATIC_INLINE void ctl_clear_rc(ctl_rc_t* obj)
{
    uint32_t i;

    obj->buffer_index = 0;
    obj->output = float2ctrl(0.0f);

    if (obj->buffer != NULL)
    {
        for (i = 0; i < obj->buffer_capacity; i++)
        {
            obj->buffer[i] = float2ctrl(0.0f);
        }
    }
}

/**
 * @brief Executes one step of the Constant-Q Repetitive Controller.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_rc(ctl_rc_t* obj, ctrl_gt error, parameter_gt measured_freq)
{
    if (measured_freq < obj->f_min_rated)
    {
        measured_freq = obj->f_min_rated;
    }

    // 1. Calculate real-time fractional depth
    parameter_gt n_real = obj->fs / measured_freq;
    int32_t n_int = (int32_t)n_real;
    parameter_gt d = n_real - (parameter_gt)n_int;

    ctrl_gt d_ctrl = float2ctrl(d);
    ctrl_gt one_minus_d = float2ctrl(1.0f) - d_ctrl;

    // 2. Calculate target depth considering phase lead compensation
    int32_t target_depth = n_int - obj->phase_lead_k;

    if (target_depth < 1)
    {
        target_depth = 1;
    }
    else if ((uint32_t)target_depth >= (obj->buffer_capacity - 1U))
    {
        target_depth = (int32_t)(obj->buffer_capacity - 2U);
    }

    // 3. Address resolution
    uint32_t read_idx_1 = _ctl_rc_wrap_index((int32_t)obj->buffer_index - target_depth, obj->buffer_capacity);
    uint32_t read_idx_2 = _ctl_rc_wrap_index((int32_t)obj->buffer_index - target_depth - 1, obj->buffer_capacity);

    // 4. Fractional linear interpolation
    ctrl_gt past_error = ctl_mul(one_minus_d, obj->buffer[read_idx_1]) + ctl_mul(d_ctrl, obj->buffer[read_idx_2]);

    // 5. Constant Q(z) filtering (Simple scalar multiplication)
    ctrl_gt q_out = ctl_mul(obj->q_gain, past_error);

    // 6. Output calculation and Anti-windup clamping
    ctrl_gt raw_output = ctl_mul(obj->k_rc, q_out);

    if (raw_output > obj->out_max)
    {
        obj->output = obj->out_max;
    }
    else if (raw_output < obj->out_min)
    {
        obj->output = obj->out_min;
    }
    else
    {
        obj->output = raw_output;
    }

    // 7. Buffer update logic
    if (obj->flag_enable_rc_integrating != 0)
    {
        obj->buffer[obj->buffer_index] = error + q_out;
    }
    else
    {
        obj->buffer[obj->buffer_index] = q_out;
    }

    // 8. Advance write pointer
    obj->buffer_index++;
    if (obj->buffer_index >= obj->buffer_capacity)
    {
        obj->buffer_index = 0;
    }

    return obj->output;
}

GMP_STATIC_INLINE void ctl_enable_rc_integrating(ctl_rc_t* obj)
{
    obj->flag_enable_rc_integrating = 1;
}
GMP_STATIC_INLINE void ctl_disable_rc_integrating(ctl_rc_t* obj)
{
    obj->flag_enable_rc_integrating = 0;
}
GMP_STATIC_INLINE ctrl_gt ctl_get_rc_output(ctl_rc_t* obj)
{
    return obj->output;
}
GMP_STATIC_INLINE void ctl_set_rc_saturation(ctl_rc_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    obj->out_min = out_min;
    obj->out_max = out_max;
}
GMP_STATIC_INLINE void ctl_set_rc_fs(ctl_rc_t* obj, parameter_gt fs)
{
    obj->fs = fs;
}

/**
 * @}
 */ // end of repetitive_controller group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _REPETITIVE_CONTROLLER_H_
