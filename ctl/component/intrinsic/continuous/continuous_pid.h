/**
 * @file continuous_pid.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides implementations for continuous-form discrete PID controllers.
 * @version 1.05
 * @date 2025-03-19
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _CONTINUOUS_PID_H_
#define _CONTINUOUS_PID_H_

#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup continuous_pid_controllers Continuous-Form PID Controllers
 * @brief A library of discrete PID controllers based on the continuous-time formula.
 * @details This module contains implementations for standard and anti-windup PID
 * controllers. The controllers are based on the continuous PID formula, discretized
 * using simple Euler methods. Both parallel and series forms are provided.
 * The anti-windup version uses a back-calculation method to prevent integrator windup.
 *
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Standard PID Controller                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a standard parallel or series PID controller.
 */
typedef struct _tag_pid_regular_t
{
    // Parameters
    ctrl_gt kp; //!< Proportional gain coefficient.
    ctrl_gt ki; //!< Integral gain coefficient.
    ctrl_gt kd; //!< Derivative gain coefficient.

    // Limits
    ctrl_gt out_max;      //!< Maximum output limit.
    ctrl_gt out_min;      //!< Minimum output limit.
    ctrl_gt integral_max; //!< Maximum integrator limit.
    ctrl_gt integral_min; //!< Minimum integrator limit.

    // State variables
    ctrl_gt out; //!< The current controller output.
    ctrl_gt sn;  //!< The integrator sum.
    ctrl_gt dn;  //!< The previous input value for the derivative term.
} ctl_pid_t;

/**
 * @brief Initializes a parallel-form PID controller.
 * @details Calculates ki = Kp * T/Ti and kd = Kp * Td/T.
 * @param[out] hpid Pointer to the PID controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pid_par(ctl_pid_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt fs);

/**
 * @brief Initializes a series-form PID controller.
 * @details Calculates ki = Kp * T/Ti and kd = Kp * Td/T. Note that the gains are applied differently in the step function.
 * @param[out] hpid Pointer to the PID controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pid_ser(ctl_pid_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt fs);

/**
 * @brief Executes one step of the parallel-form PID controller.
 * @details Output = Kp*e(n) + Ki*sum(e(n)) + Kd*(e(n)-e(n-1)).
 * @param[in,out] hpid Pointer to the PID controller instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pid_par(ctl_pid_t* hpid, ctrl_gt input)
{
    // Update integrator sum with anti-windup
    hpid->sn = ctl_sat(hpid->sn + ctl_mul(input, hpid->ki), hpid->integral_max, hpid->integral_min);

    // Output = P_term + I_term + D_term
    hpid->out = ctl_mul(input, hpid->kp) + hpid->sn + ctl_mul((input - hpid->dn), hpid->kd);

    // Saturate final output
    hpid->out = ctl_sat(hpid->out, hpid->out_max, hpid->out_min);

    // Store current input for next derivative calculation
    hpid->dn = input;

    return hpid->out;
}

/**
 * @brief Executes one step of the series-form PID controller.
 * @details Output = Kp * [e(n) + 1/Ti * sum(e(n)) + Td * (e(n)-e(n-1))].
 * This is approximated by applying Kp to all terms.
 * @param[in,out] hpid Pointer to the PID controller instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pid_ser(ctl_pid_t* hpid, ctrl_gt input)
{
    // Proportional term is calculated first
    ctrl_gt p_out = ctl_mul(input, hpid->kp);

    // Update integrator sum based on the proportional output
    hpid->sn = ctl_sat(hpid->sn + ctl_mul(p_out, hpid->ki), hpid->integral_max, hpid->integral_min);

    // Output = P_term + I_term + D_term
    // Note: In a true series form, Kd would also be scaled by Kp.
    hpid->out = p_out + hpid->sn + ctl_mul((input - hpid->dn), hpid->kd);

    // Saturate final output
    hpid->out = ctl_sat(hpid->out, hpid->out_max, hpid->out_min);

    // Store current input for next derivative calculation
    hpid->dn = input;

    return hpid->out;
}

/**
 * @brief Clears the internal states of the PID controller.
 * @param[out] hpid Pointer to the PID controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_pid(ctl_pid_t* hpid)
{
    hpid->dn = 0;
    hpid->sn = 0;
    hpid->out = 0;
}

/**
 * @brief Set PID controller PID limit
 * @param[out] hpid Pointer to the PID controller instance.
 * @param[in] limit_min PID output minimum.
 * @param[in] limit_max PID output maximum.
 */
GMP_STATIC_INLINE void ctl_set_pid_limit(ctl_pid_t* hpid, ctrl_gt limit_max, ctrl_gt limit_min)
{
    hpid->out_max = limit_max;
    hpid->out_min = limit_min;
}

/**
 * @brief Set PID controller PID integrator item
 * @param[out] hpid Pointer to the PID controller instance.
 * @param[in] integrator target integrator current value.
 */
GMP_STATIC_INLINE void ctl_set_pid_integrator(ctl_pid_t* hpid, ctrl_gt integrator)
{
    hpid->sn = integrator;
}

/*---------------------------------------------------------------------------*/
/* PID Controller with Anti-Windup                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a PID controller with back-calculation anti-windup.
 */
typedef struct _tag_pid_anti_windup
{
    // Parameters
    ctrl_gt kp; //!< Proportional gain coefficient.
    ctrl_gt ki; //!< Integral gain coefficient.
    ctrl_gt kd; //!< Derivative gain coefficient.
    ctrl_gt kc; //!< Back-calculation gain for anti-windup.

    // Limits
    ctrl_gt out_max; //!< Maximum output limit.
    ctrl_gt out_min; //!< Minimum output limit.

    // State variables
    ctrl_gt out;             //!< The current, saturated controller output.
    ctrl_gt sn;              //!< The integrator sum.
    ctrl_gt dn;              //!< The previous input value for the derivative term.
    ctrl_gt out_without_sat; //!< The unsaturated output, used for back-calculation.
} ctl_pid_aw_t;

/**
 * @brief Initializes a parallel-form PID controller with anti-windup.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pid_aw_par(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt fs);

/**
 * @brief Initializes a series-form PID controller with anti-windup.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] Td Derivative time constant (seconds).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pid_aw_ser(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt fs);

/**
 * @brief Executes one step of the parallel-form anti-windup PID controller.
 * @param[in,out] hpid Pointer to the PID anti-windup instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pid_aw_par(ctl_pid_aw_t* hpid, ctrl_gt input)
{
    // Calculate unsaturated output
    hpid->out_without_sat = ctl_mul(input, hpid->kp) + hpid->sn + ctl_mul((input - hpid->dn), hpid->kd);

    // Saturate the output
    hpid->out = ctl_sat(hpid->out_without_sat, hpid->out_max, hpid->out_min);

    // Update integrator sum with back-calculation anti-windup
    ctrl_gt back_calc_term = ctl_mul(hpid->out_without_sat - hpid->out, hpid->kc);
    hpid->sn = hpid->sn + ctl_mul(input, hpid->ki) - back_calc_term;

    // Store current input for next derivative calculation
    hpid->dn = input;

    return hpid->out;
}

/**
 * @brief Executes one step of the series-form anti-windup PID controller.
 * @param[in,out] hpid Pointer to the PID anti-windup instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pid_aw_ser(ctl_pid_aw_t* hpid, ctrl_gt input)
{
    // Calculate unsaturated output
    hpid->out_without_sat = ctl_mul(input + hpid->sn + ctl_mul((input - hpid->dn), hpid->kd), hpid->kp);

    // Saturate the output
    hpid->out = ctl_sat(hpid->out_without_sat, hpid->out_max, hpid->out_min);

    // Update integrator sum with back-calculation anti-windup
    ctrl_gt back_calc_term = ctl_mul(hpid->out_without_sat - hpid->out, hpid->kc);
    hpid->sn = hpid->sn + ctl_mul(input, hpid->ki) - back_calc_term;

    // Store current input for next derivative calculation
    hpid->dn = input;

    return hpid->out;
}

/**
 * @brief Clears the internal states of the anti-windup PID controller.
 * @param[out] hpid Pointer to the PID anti-windup instance.
 */
GMP_STATIC_INLINE void ctl_clear_pid_aw(ctl_pid_aw_t* hpid)
{
    hpid->dn = 0;
    hpid->sn = 0;
    hpid->out = 0;
    hpid->out_without_sat = 0;
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
 */ // end of continuous_pid_controllers group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _CONTINUOUS_PID_H_
