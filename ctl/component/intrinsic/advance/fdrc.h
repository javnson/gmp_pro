/**
 * @file rc_controller.h
 * @author GMP Library Contributors
 * @brief Fractional Delay Repetitive Controller (FDRC) with decoupled buffer management.
 * @details Implements a plug-in repetitive controller incorporating a Biquad filter 
 * as the Q(z) robustness filter. The delay line buffer is dynamically assigned 
 * to support both static and dynamic memory allocation.
 * 
 * @version 1.0
 * @date 2026-05-03
 *
 * @note 
 * <b>Mathematical Principle of the FDRC:</b>
 * 
 * The Fractional Delay Repetitive Controller (FDRC) is designed to track periodic 
 * references and reject periodic disturbances by embedding an internal model of the 
 * fundamental frequency into the control loop.
 * 
 * The discrete Z-domain transfer function of the plug-in RC is defined as:
 * @f[
 * U_{rc}(z) = \frac{K_{rc} Q(z) z^{-N+k}}{1 - Q(z) z^{-N}} E(z)
 * @f]
 * Where:
 * - @f$ U_{rc}(z) @f$ is the control output.
 * - @f$ E(z) @f$ is the tracking error.
 * - @f$ K_{rc} @f$ is the repetitive learning gain.
 * - @f$ N = f_s / f_{grid} @f$ is the delay length, calculated dynamically as a fractional number.
 * - @f$ k @f$ (`phase_lead_k`) is the phase lead step count, used to compensate for the 
 *   phase lag caused by computation delay, PWM update, and the physical plant (L/LCL).
 * - @f$ Q(z) @f$ is a robustness filter (implemented as a low-pass Biquad filter) that 
 *   attenuates the infinite gain at extremely high frequencies to ensure closed-loop stability.
 * 
 * To handle fractional @f$ N @f$ due to grid frequency variations, this module employs 
 * linear interpolation between adjacent samples @f$ z^{-\lfloor N \rfloor} @f$ and 
 * @f$ z^{-\lfloor N \rfloor - 1} @f$.
 * 
 * <b>Usage Guidelines in GMP Framework:</b>
 * 
 * 1. <b>Memory Allocation:</b>
 *    Calculate the maximum possible buffer size using `CTL_RC_CALC_MIN_CAPACITY`.
 *    Allocate an array of `ctrl_gt`, either globally, statically, or via dynamic allocation.
 *    @code
 *    #define MAX_SAMPLES CTL_RC_CALC_MIN_CAPACITY(20000.0f, 45.0f)
 *    static ctrl_gt rc_buffer[MAX_SAMPLES];
 *    ctl_fdrc_t rc_ctrl;
 *    @endcode
 * 
 * 2. <b>Initialization:</b>
 *    Call `ctl_init_fdrc` before the control loop starts. Provide the buffer pointer, 
 *    the expected low-pass cutoff for @f$ Q(z) @f$ (e.g., 1000Hz), and the phase lead @f$ k @f$.
 *    @code
 *    ctl_init_fdrc(&rc_ctrl, rc_buffer, MAX_SAMPLES, 20000.0f, 45.0f, 1000.0f, 0.8f, 4);
 *    ctl_set_fdrc_saturation(&rc_ctrl, -50.0f, 50.0f); // Restrict RC authority
 *    @endcode
 * 
 * 3. <b>Execution in High-Frequency ISR:</b>
 *    Execute the QPR (or PI) main controller first.
 *    Feed the tracking error and the real-time grid frequency from a SOGI-PLL to the FDRC.
 *    @code
 *    ctrl_gt rc_out = ctl_step_fdrc(&rc_ctrl, error, current_pll_freq);
 *    ctrl_gt total_out = base_out + rc_out;
 *    @endcode
 * 
 * 4. <b>Anti-Windup Strategy:</b>
 *    If the total calculated duty cycle exceeds the hardware physical limits (e.g., 
 *    DC bus voltage clipping), immediately disable the RC learning to prevent corrupt 
 *    transient data from polluting the delay line.
 *    @code
 *    if (is_hardware_saturated(total_out)) {
 *        ctl_disable_fdrc_integrating(&rc_ctrl);
 *    } else {
 *        ctl_enable_fdrc_integrating(&rc_ctrl);
 *    }
 *    @endcode
 * 
 * @copyright Copyright GMP(c) 2024-2026
 */

#include <ctl/component/intrinsic/discrete/biquad_filter.h>

#ifndef _RC_CONTROLLER_H_
#define _RC_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup RC_controller_api Repetitive Controller Library
 * @brief Fractional delay repetitive controller for periodic error compensation.
 * @{
 */

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

/*---------------------------------------------------------------------------*/
/* Repetitive Controller (RC) Core Generator                                 */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Repetitive Controller core, with Q(z) as a biquad filter.
 */
typedef struct _tag_repetitive_controller_filter_t
{
    ctrl_gt* buffer;          //!< Circular buffer to store past outputs (the delay line).
    uint32_t buffer_capacity; //!< Total capacity of the allocated buffer array.
    uint32_t buffer_index;    //!< Current write index for the circular buffer.

    int32_t phase_lead_k;         //!< Phase lead steps to compensate for plant delay.
    ctl_biquad_filter_t q_filter; //!< Q(z) filter for RC controller robustness.
    ctrl_gt k_rc;                 //!< Learning gain of the RC.

    parameter_gt fs;          //!< Controller frequency, Hz
    parameter_gt f_min_rated; //!< Rated minimum frequency, Hz

    // Output Limits
    ctrl_gt out_max; //!< Maximum output limit for anti-windup.
    ctrl_gt out_min; //!< Minimum output limit for anti-windup.

    ctrl_gt output; //!< The last calculated controller output.

    // Control Flags
    fast_gt flag_enable_rc_integrating; //!< Flag to enable/disable integrating new errors.
} ctl_fdrc_t;

/*---------------------------------------------------------------------------*/
/* Inline Helper Functions                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Fast circular buffer index wrapping.
 * @details Optimized for single-pass out-of-bounds correction, avoiding costly modulo operator.
 * @param[in] index The raw calculated index (can be negative).
 * @param[in] capacity The maximum capacity of the buffer.
 * @return uint32_t The securely wrapped index.
 */
GMP_STATIC_INLINE uint32_t _ctl_fdrc_wrap_index(int32_t index, uint32_t capacity)
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
 * @brief Initializes the Repetitive Controller instance.
 * @param[out] obj Pointer to the RC controller instance.
 * @param[in] buffer Pointer to the externally allocated memory buffer.
 * @param[in] capacity Size of the provided buffer.
 * @param[in] fs Sampling frequency in Hz.
 * @param[in] f_min Minimum expected fundamental frequency in Hz (used for assertion).
 * @param[in] q_fc Cutoff frequency for the internal Q(z) low-pass filter.
 * @param[in] k_rc Learning gain.
 * @param[in] phase_lead_k Phase lead step count.
 */
void ctl_init_fdrc(ctl_fdrc_t* obj, ctrl_gt* buffer, uint32_t capacity, parameter_gt fs, parameter_gt f_min,
                   parameter_gt q_fc, parameter_gt k_rc, int32_t phase_lead_k);

/**
 * @brief Clears the internal states and memory buffer of the RC.
 * @param[out] obj Pointer to the RC controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_fdrc(ctl_fdrc_t* obj)
{
    uint32_t i;

    obj->buffer_index = 0;
    obj->output = float2ctrl(0.0f);
    ctl_clear_biquad_filter(&obj->q_filter);

    if (obj->buffer != NULL)
    {
        for (i = 0; i < obj->buffer_capacity; i++)
        {
            obj->buffer[i] = float2ctrl(0.0f);
        }
    }
}

/**
 * @brief Executes one step of the Repetitive Controller (High-performance critical path).
 * @param[in,out] obj Pointer to the RC controller instance.
 * @param[in] error Current tracking error (Reference - Feedback).
 * @param[in] fs System sampling frequency (Hz).
 * @param[in] measured_freq Real-time fundamental frequency measured by PLL (Hz).
 * @return ctrl_gt The clamped compensation output from the RC.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_fdrc(ctl_fdrc_t* obj, ctrl_gt error, parameter_gt measured_freq)
{
    // Protect against division by zero or extremely low frequencies
    if (measured_freq < obj->f_min_rated)
    {
        measured_freq = obj->f_min_rated;
    }

    // 1. Calculate real-time fractional depth
    parameter_gt n_real = obj->fs / measured_freq;
    int32_t n_int = (int32_t)n_real;
    parameter_gt d = n_real - (parameter_gt)n_int;

    // Convert fractional parts to control generic type for interpolation
    ctrl_gt d_ctrl = float2ctrl(d);
    ctrl_gt one_minus_d = float2ctrl(1.0f) - d_ctrl;

    // 2. Calculate target depth considering phase lead compensation
    int32_t target_depth = n_int - obj->phase_lead_k;

    // Boundary protection for target depth
    if (target_depth < 1)
    {
        target_depth = 1;
    }
    else if ((uint32_t)target_depth >= (obj->buffer_capacity - 1U))
    {
        target_depth = (int32_t)(obj->buffer_capacity - 2U);
    }

    // 3. Address resolution using optimized wrap function
    uint32_t read_idx_1 = _ctl_fdrc_wrap_index((int32_t)obj->buffer_index - target_depth, obj->buffer_capacity);
    uint32_t read_idx_2 = _ctl_fdrc_wrap_index((int32_t)obj->buffer_index - target_depth - 1, obj->buffer_capacity);

    // 4. Fractional linear interpolation
    ctrl_gt past_error = ctl_mul(one_minus_d, obj->buffer[read_idx_1]) + ctl_mul(d_ctrl, obj->buffer[read_idx_2]);

    // 5. Q(z) low-pass filtering to ensure high-frequency robustness
    ctrl_gt q_out = ctl_step_biquad_filter(&obj->q_filter, past_error);

    // 6. Controller output calculation and Anti-windup clamping
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

    // 7. Buffer update logic based on learning flag
    if (obj->flag_enable_rc_integrating != 0)
    {
        // Active learning: Integrate new error into the delay line
        obj->buffer[obj->buffer_index] = error + q_out;
    }
    else
    {
        // Learning disabled/Saturated: Decay existing profile, reject new transient errors
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

/**
 * @brief Enables the error integration of the Repetitive Controller.
 * @details Resumes the learning process. The controller will start accumulating 
 * tracking errors into the delay line buffer to update the harmonic compensation profile.
 * @param[in,out] obj Pointer to the FDRC controller instance.
 */
GMP_STATIC_INLINE void ctl_enable_fdrc_integrating(ctl_fdrc_t* obj)
{
    obj->flag_enable_rc_integrating = 1;
}

/**
 * @brief Disables the error integration of the Repetitive Controller.
 * @details Freezes the learning process. Used primarily for Anti-Windup when the 
 * system output is saturated, or during severe grid transients. The controller 
 * will output the historical compensation profile with Q(z) decay, but will not 
 * absorb new corrupted errors.
 * @param[in,out] obj Pointer to the FDRC controller instance.
 */
GMP_STATIC_INLINE void ctl_disable_fdrc_integrating(ctl_fdrc_t* obj)
{
    obj->flag_enable_rc_integrating = 0;
}

/**
 * @brief Retrieves the last calculated output of the Repetitive Controller.
 * @param[in] obj Pointer to the FDRC controller instance.
 * @return ctrl_gt The most recent compensation control effort.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_fdrc_output(ctl_fdrc_t* obj)
{
    return obj->output;
}

/**
 * @brief Sets the output saturation limits for the Repetitive Controller.
 * @details Limits the maximum authority of the RC to prevent it from dominating 
 * the main controller (e.g., QPR) during transients.
 * @param[in,out] obj Pointer to the FDRC controller instance.
 * @param[in] out_min The minimum allowable output value.
 * @param[in] out_max The maximum allowable output value.
 */
GMP_STATIC_INLINE void ctl_set_fdrc_limit(ctl_fdrc_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    obj->out_min = out_min;
    obj->out_max = out_max;
}

/**
 * @brief Updates the system sampling frequency dynamically.
 * @details Usually called when the core ISR frequency is reconfigured at runtime.
 * @param[in,out] obj Pointer to the FDRC controller instance.
 * @param[in] fs New sampling frequency in Hz.
 */
GMP_STATIC_INLINE void ctl_set_fdrc_fs(ctl_fdrc_t* obj, parameter_gt fs)
{
    obj->fs = fs;
}

/** @} */

#ifdef __cplusplus
}
#endif

#endif // _RC_CONTROLLER_H_
