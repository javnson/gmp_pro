/**
 * @file continuous_pid_aw.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides implementations for high-performance continuous-form discrete PID controllers with anti-windup.
 * @version 1.06
 * @date 2026-03-16
 *
 * @copyright Copyright GMP(c) 2026
 *
 * This module contains high-performance PID controllers featuring back-calculation 
 * anti-windup and derivative low-pass filtering. Both parallel and series forms are provided.
 * * @warning Initialization functions (e.g., ctl_init_pid_aw_par) MUST be strictly 
 * paired with their corresponding step functions (e.g., ctl_step_pid_aw_par).
 */

#ifndef _CONTINUOUS_PID_AW_H_
#define _CONTINUOUS_PID_AW_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup continuous_pid_aw_controllers Anti-Windup PID Controllers
 * @ingroup continuous_pid_controllers
 * @brief High-performance PID controllers with anti-windup and D-term filtering.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* PID Controller with Anti-Windup (High Performance)                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a PID controller with back-calculation anti-windup
 * and derivative low-pass filtering.
 */
typedef struct _tag_pid_anti_windup
{
    // Parameters
    ctrl_gt kp;      //!< Proportional gain coefficient.
    ctrl_gt ki;      //!< Integral gain coefficient.
    ctrl_gt kd;      //!< Derivative gain coefficient.
    ctrl_gt kc;      //!< Back-calculation gain for anti-windup.
    ctrl_gt alpha_d; //!< Low-pass filter coefficient for the derivative term.

    // Limits
    ctrl_gt out_max; //!< Maximum output limit.
    ctrl_gt out_min; //!< Minimum output limit.

    // State variables
    ctrl_gt out;             //!< The current, saturated controller output.
    ctrl_gt sn;              //!< The integrator sum.
    ctrl_gt dn;              //!< The previous input value for the derivative term, e(n-1).
    ctrl_gt d_term;          //!< The previous filtered derivative result.
    ctrl_gt out_without_sat; //!< The unsaturated output, used for back-calculation.
} ctl_pid_aw_t;

/**
 * @brief Initializes a parallel-form PID controller with anti-windup.
 * @warning MUST be used with `ctl_step_pid_aw_par` during execution.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] Tf Derivative low-pass filter time constant (seconds). Set to 0 to disable filtering.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pid_aw_par(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt Tf,
                         parameter_gt fs);

/**
 * @brief Initializes a series-form PID controller with anti-windup.
 * @warning MUST be used with `ctl_step_pid_aw_ser` during execution.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] Tf Derivative low-pass filter time constant (seconds). Set to 0 to disable filtering.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pid_aw_ser(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt Tf,
                         parameter_gt fs);

/**
 * @brief Executes one step of the parallel-form anti-windup PID controller.
 * @details Math: U = Kp*e + Ki/s*e + Kd*s/(Tf*s+1)*e
 * @param[in,out] hpid Pointer to the PID anti-windup instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated saturated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pid_aw_par(ctl_pid_aw_t* hpid, ctrl_gt input)
{
    // 1. Calculate filtered derivative term
    ctrl_gt d_raw = ctl_mul(input - hpid->dn, hpid->kd);
    hpid->d_term = hpid->d_term + ctl_mul(hpid->alpha_d, d_raw - hpid->d_term);

    // 2. Calculate unsaturated output (Parallel: additions are independent)
    hpid->out_without_sat = ctl_mul(input, hpid->kp) + hpid->sn + hpid->d_term;

    // 3. Saturate the output
    hpid->out = ctl_sat(hpid->out_without_sat, hpid->out_max, hpid->out_min);

    // 4. Update integrator sum with back-calculation anti-windup
    ctrl_gt back_calc_term = ctl_mul(hpid->out_without_sat - hpid->out, hpid->kc);
    hpid->sn = hpid->sn + ctl_mul(input, hpid->ki) - back_calc_term;

    // 5. Store current input for next derivative calculation
    hpid->dn = input;

    return hpid->out;
}

/**
 * @brief Executes one step of the series-form anti-windup PID controller.
 * @details Math: U = Kp * [ e + 1/(Ti*s)*e + Td*s/(Tf*s+1)*e ]
 * @param[in,out] hpid Pointer to the PID anti-windup instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated saturated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pid_aw_ser(ctl_pid_aw_t* hpid, ctrl_gt input)
{
    // 1. Calculate filtered derivative term
    ctrl_gt d_raw = ctl_mul(input - hpid->dn, hpid->kd);
    hpid->d_term = hpid->d_term + ctl_mul(hpid->alpha_d, d_raw - hpid->d_term);

    // 2. Calculate unsaturated output (Series: Kp multiplies everything)
    hpid->out_without_sat = ctl_mul(input + hpid->sn + hpid->d_term, hpid->kp);

    // 3. Saturate the output
    hpid->out = ctl_sat(hpid->out_without_sat, hpid->out_max, hpid->out_min);

    // 4. Update integrator sum with back-calculation anti-windup
    ctrl_gt back_calc_term = ctl_mul(hpid->out_without_sat - hpid->out, hpid->kc);
    hpid->sn = hpid->sn + ctl_mul(input, hpid->ki) - back_calc_term;

    // 5. Store current input for next derivative calculation
    hpid->dn = input;

    return hpid->out;
}

/**
 * @brief Clears the internal states of the anti-windup PID controller.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 */
GMP_STATIC_INLINE void ctl_clear_pid_aw(ctl_pid_aw_t* hpid)
{
    hpid->dn = float2ctrl(0.0f);
    hpid->sn = float2ctrl(0.0f);
    hpid->d_term = float2ctrl(0.0f);
    hpid->out = float2ctrl(0.0f);
    hpid->out_without_sat = float2ctrl(0.0f);
}

/**
 * @brief Sets the back-calculation gain for the anti-windup mechanism.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 * @param[in] back_gain The new back-calculation gain (kc).
 */
GMP_STATIC_INLINE void ctl_set_pid_aw_back_gain(ctl_pid_aw_t* hpid, ctrl_gt back_gain)
{
    hpid->kc = back_gain;
}

/**
 * @}
 */ // end of continuous_pid_aw_controllers group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _CONTINUOUS_PID_AW_H_
