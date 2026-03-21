/**
 * @file fuzzy_pid.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a Fuzzy Logic PID controller with LUT-based parameter tuning.
 * @version 0.1
 * @date 2025-08-07
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FUZZY_PID_H_
#define _FUZZY_PID_H_

#include <ctl/component/intrinsic/advance/surf_search.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup fuzzy_pid_controller Fuzzy PID Controller
 * @brief A self-tuning PID controller using fuzzy logic look-up tables.
 * @details This file implements a fuzzy self-tuning PID controller. The controller
 * uses two inputs: the error (E) and the change in error (EC). These inputs
 * are used to look up tuning adjustments for the PID parameters (delta_Kp,
 * delta_Ki, delta_Kd) from three separate 2D look-up tables (surfaces).
 * These adjustments are then added to the base PID parameters, allowing the
 * controller to adapt to changing system dynamics in real-time.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Fuzzy Self-Tuning PID Controller                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Fuzzy PID controller.
 */
typedef struct _tag_fuzzy_pid_t
{
    // Core PID controller
    ctl_pid_t pid; //!< The underlying standard PID controller.

    // Base PID parameters
    ctrl_gt base_kp; //!< The base proportional gain.
    ctrl_gt base_ki; //!< The base integral time constant.
    ctrl_gt base_kd; //!< The base derivative time constant.

    // Fuzzy tuning surfaces (Look-Up Tables)
    ctl_lut2d_t d_kp_lut; //!< 2D LUT for delta_Kp adjustments.
    ctl_lut2d_t d_ki_lut; //!< 2D LUT for delta_Ki adjustments.
    ctl_lut2d_t d_kd_lut; //!< 2D LUT for delta_Kd adjustments.

    // Quantization factors
    ctrl_gt e_q_factor;  //!< Quantization factor for the error input.
    ctrl_gt ec_q_factor; //!< Quantization factor for the change-in-error input.

    // State variables
    ctrl_gt last_error;  //!< Stores the previous error for calculating the change in error.
    ctrl_gt fs_ctrl;     //!< fs in ctrl_gt, for scaling d_kd
    ctrl_gt inv_fs_ctrl; //!< The sampling frequency of the controller.

} ctl_fuzzy_pid_t;

/**
 * @brief Initializes the Fuzzy PID controller.
 * @param[out] fp Pointer to the Fuzzy PID instance.
 * @param[in] base_kp The base proportional gain.
 * @param[in] base_ti The base integral time constant (in seconds).
 * @param[in] base_td The base derivative time constant (in seconds).
 * @param[in] sat_max Maximum output saturation limit for the PID.
 * @param[in] sat_min Minimum output saturation limit for the PID.
 * @param[in] e_q_factor Quantization factor to scale the error before LUT lookup.
 * @param[in] ec_q_factor Quantization factor to scale the change-in-error before LUT lookup.
 * @param[in] d_kp_lut A fully initialized 2D LUT for delta_Kp.
 * @param[in] d_ki_lut A fully initialized 2D LUT for delta_Ki.
 * @param[in] d_kd_lut A fully initialized 2D LUT for delta_Kd.
 * @param[in] fs The sampling frequency (Hz).
 */
void ctl_init_fuzzy_pid(ctl_fuzzy_pid_t* fp, parameter_gt base_kp, parameter_gt base_ti, parameter_gt base_td,
                        ctrl_gt sat_max, ctrl_gt sat_min, parameter_gt e_q_factor, parameter_gt ec_q_factor,
                        ctl_lut2d_t d_kp_lut, ctl_lut2d_t d_ki_lut, ctl_lut2d_t d_kd_lut, parameter_gt fs);

/**
 * @brief Clears the internal states of the Fuzzy PID controller.
 * @param[out] fp Pointer to the Fuzzy PID instance.
 */
GMP_STATIC_INLINE void ctl_clear_fuzzy_pid(ctl_fuzzy_pid_t* fp)
{
    ctl_clear_pid(&fp->pid);
    fp->last_error = 0;
}

/**
 * @brief Executes one step of the fuzzy PID controller.
 * @details 经过极致的定点优化，ISR 中没有任何浮点转换和除法。
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_fuzzy_pid(ctl_fuzzy_pid_t* fp, ctrl_gt target, ctrl_gt feedback)
{
    // 1. Calculate error and change in error
    ctrl_gt error = target - feedback;
    ctrl_gt error_change = error - fp->last_error;
    fp->last_error = error;

    // 2. Quantize inputs for the fuzzy LUTs (纯定点乘法)
    ctrl_gt e_quantized = ctl_mul(error, fp->e_q_factor);
    ctrl_gt ec_quantized = ctl_mul(error_change, fp->ec_q_factor);

    // 3. Look up physical PID parameter increments (Delta Kp, Ki, Kd)
    ctrl_gt d_kp = ctl_step_interpolate_lut2d(&fp->d_kp_lut, e_quantized, ec_quantized);
    ctrl_gt d_ki = ctl_step_interpolate_lut2d(&fp->d_ki_lut, e_quantized, ec_quantized);
    ctrl_gt d_kd = ctl_step_interpolate_lut2d(&fp->d_kd_lut, e_quantized, ec_quantized);

    // 4. Update the underlying Parallel PID's runtime discrete coefficients
    // Kp_run = base_Kp + dKp
    fp->pid.kp = fp->base_kp + d_kp;

    // Ki_run = (base_Ki + dKi) / fs = base_Ki_run + dKi * (1/fs)
    fp->pid.ki = fp->base_ki + ctl_mul(d_ki, fp->inv_fs_ctrl);

    // Kd_run = (base_Kd + dKd) * fs = base_Kd_run + dKd * fs
    fp->pid.kd = fp->base_kd + ctl_mul(d_kd, fp->fs_ctrl);

    // Ensure parameters remain physically meaningful (>= 0)
    ctrl_gt zero_ctrl = float2ctrl(0.0f);
    if (fp->pid.kp < zero_ctrl)
        fp->pid.kp = zero_ctrl;
    if (fp->pid.ki < zero_ctrl)
        fp->pid.ki = zero_ctrl;
    if (fp->pid.kd < zero_ctrl)
        fp->pid.kd = zero_ctrl;

    // 5. Execute the underlying Parallel PID controller! (必须用 par 而非 ser)
    return ctl_step_pid_par(&fp->pid, error);
}

/**
 * @}
 */ // end of fuzzy_pid_controller group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _FUZZY_PID_H_
