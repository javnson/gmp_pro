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

#ifndef _REPETITIVE_CONTROLLER_H_
#define _REPETITIVE_CONTROLLER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup repetitive_controller Repetitive Controller (RC)
 * @brief An internal model-based controller for eliminating periodic errors.
 * @details This file implements the core generator of a repetitive control system,
 * based on the Internal Model Principle. The module itself acts as a periodic
 * signal generator whose memory is updated by an external, compensated error
 * signal. It is designed to be combined with other controllers (like P or PR)
 * and a user-defined compensator to build a full high-performance repetitive
 * control system for tracking or rejecting periodic signals.
 *
 * The core transfer function implemented is:
 * @f[ G_gen(z) = 1 / (1 - q * z^-N) @f]
 * 
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Repetitive Controller (RC) Core Generator                                 */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Repetitive Controller core.
 */
typedef struct _tag_repetitive_controller_t
{
    // Configuration
    uint32_t period_samples; //!< Number of samples in one fundamental period (N).
    ctrl_gt q_filter_coeff;  //!< Stability factor (q), a value slightly less than 1.

    // State
    ctrl_gt* state_buffer; //!< Circular buffer to store past outputs (the delay line).
    uint32_t buffer_index; //!< Current index for the circular buffer.
    ctrl_gt output;        //!< The last calculated controller output.

    // Output Limits
    ctrl_gt out_max; //!< Maximum output limit.
    ctrl_gt out_min; //!< Minimum output limit.
} ctl_repetitive_controller_t;

/**
 * @brief Initializes the Repetitive Controller.
 * @details This function allocates memory for the internal state buffer. The user
 * is responsible for calling ctl_destroy_repetitive_controller to free this
 * memory when the controller is no longer needed.
 * @param[out] rc Pointer to the repetitive controller instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f_fund Fundamental frequency of the periodic signal (Hz).
 * @param[in] q_filter_coeff Stability factor (0 < q < 1). A value like 0.95 is a good start.
 * @return fast_gt Returns 1 on success (memory allocated), 0 on failure.
 */
fast_gt ctl_init_repetitive_controller(ctl_repetitive_controller_t* rc, parameter_gt fs, parameter_gt f_fund,
                                       parameter_gt q_filter_coeff);

/**
 * @brief Frees the memory allocated for the controller's buffer.
 * @param[in,out] rc Pointer to the repetitive controller instance.
 */
void ctl_destroy_repetitive_controller(ctl_repetitive_controller_t* rc);

/**
 * @brief Clears the internal states of the repetitive controller.
 * @details Resets the state buffer and output to zero.
 * @param[out] rc Pointer to the repetitive controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_repetitive_controller(ctl_repetitive_controller_t* rc)
{
    uint32_t i;

    if (rc->state_buffer != NULL)
    {
        for (i = 0; i < rc->period_samples; i++)
        {
            rc->state_buffer[i] = 0;
        }
    }
    rc->output = 0;
    rc->buffer_index = 0;
}

/**
 * @brief Sets the output limits for the RC controller.
 * @param[out] rc Pointer to the repetitive controller instance.
 * @param[in] limit_max The maximum output value.
 * @param[in] limit_min The minimum output value.
 */
GMP_STATIC_INLINE void ctl_set_repetitive_controller_limit(ctl_repetitive_controller_t* rc, ctrl_gt limit_max,
                                                           ctrl_gt limit_min)
{
    rc->out_max = limit_max;
    rc->out_min = limit_min;
}

/**
 * @brief Executes one step of the Repetitive Controller core logic.
 * @details Implements the difference equation: u_out(k) = q * u_out(k-N) + u_in(k).
 * The input `compensated_error` is expected to be pre-processed by a gain and
 * a phase lead compensator.
 * @param[in,out] rc Pointer to the repetitive controller instance.
 * @param[in] compensated_error The compensated error signal, C(z)*e(k).
 * @return ctrl_gt The calculated RC output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_repetitive_controller(ctl_repetitive_controller_t* rc, ctrl_gt compensated_error)
{
    // 1. Get the past output u_out(k-N) from the circular buffer
    ctrl_gt past_output = rc->state_buffer[rc->buffer_index];

    // 2. Calculate the new output: u_out(k) = q * u_out(k-N) + u_in(k)
    rc->output = ctl_mul(past_output, rc->q_filter_coeff) + compensated_error;

    // 3. Saturate the output
    rc->output = ctl_sat(rc->output, rc->out_max, rc->out_min);

    // 4. Update the buffer with the new output for the next cycle
    rc->state_buffer[rc->buffer_index] = rc->output;

    // 5. Advance the buffer index for the next sample
    rc->buffer_index++;
    if (rc->buffer_index >= rc->period_samples)
    {
        rc->buffer_index = 0;
    }

    return rc->output;
}

/**
 * @}
 */ // end of repetitive_controller group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _REPETITIVE_CONTROLLER_H_
