/**
 * @file continuous_pi.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides implementations for continuous-form discrete PI and IP controllers.
 * @version 1.01
 * @date 2026-03-16
 *
 * @copyright Copyright GMP(c) 2026
 *
 * This module contains implementations for PI (Proportional-Integral) and 
 * IP (Integral-Proportional) controllers. IP controllers are particularly 
 * useful for eliminating proportional kick during setpoint step changes.
 */

#ifndef _CONTINUOUS_PI_H_
#define _CONTINUOUS_PI_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup continuous_pi_controllers Continuous-Form PI/IP Controllers
 * @brief A library of discrete PI and IP controllers.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Standard PI / IP Controller                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a standard parallel or series PI/IP controller.
 */
typedef struct _tag_pi_regular_t
{
    // Parameters
    ctrl_gt kp; //!< Proportional gain coefficient.
    ctrl_gt ki; //!< Integral gain coefficient.

    // Limits
    ctrl_gt out_max;      //!< Maximum output limit.
    ctrl_gt out_min;      //!< Minimum output limit.
    ctrl_gt integral_max; //!< Maximum integrator limit.
    ctrl_gt integral_min; //!< Minimum integrator limit.

    // State variables
    ctrl_gt p_term; //!< Current P term.
    ctrl_gt i_term; //!< Current I term (integrator accumulator).

    ctrl_gt out; //!< The current controller output.
} ctl_pi_t;

/**
 * @brief Initializes a series-form PI controller using time constants.
 * @details Calculates ki = 1 / (fs * Ti). The Kp scaling is handled in the step function.
 * @param[out] hpi Pointer to the PI controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] Ti Integral time constant (seconds).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pi_Tmode(ctl_pi_t* hpi, parameter_gt kp, parameter_gt Ti, parameter_gt fs);

/**
 * @brief Initializes a parallel-form PI controller using direct gains.
 * @details Calculates ki = Ki / fs.
 * @param[out] hpi Pointer to the PI controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] ki Integral gain.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_pi(ctl_pi_t* hpi, parameter_gt kp, parameter_gt ki, parameter_gt fs);

/**
 * @brief Executes one step of the parallel-form PI controller.
 * @details Math: U = Kp*e + Ki*sum(e)
 * @param[in,out] hpi Pointer to the PI controller instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated saturated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pi_par(ctl_pi_t* hpi, ctrl_gt input)
{
    hpi->p_term = ctl_mul(input, hpi->kp);
    hpi->i_term = ctl_sat(hpi->i_term + ctl_mul(input, hpi->ki), hpi->integral_max, hpi->integral_min);

    hpi->out = ctl_sat(hpi->p_term + hpi->i_term, hpi->out_max, hpi->out_min);
    return hpi->out;
}

/**
 * @brief Executes one step of the series-form (ideal) PI controller.
 * @details Math: U = Kp * [ e + 1/Ti * sum(e) ]
 * @param[in,out] hpi Pointer to the PI controller instance.
 * @param[in] input The current input error, e(n).
 * @return ctrl_gt The calculated saturated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pi_ser(ctl_pi_t* hpi, ctrl_gt input)
{
    hpi->p_term = ctl_mul(input, hpi->kp);
    // Accumulate the Kp-scaled input for the ideal series form
    hpi->i_term = ctl_sat(hpi->i_term + ctl_mul(hpi->p_term, hpi->ki), hpi->integral_max, hpi->integral_min);

    hpi->out = ctl_sat(hpi->p_term + hpi->i_term, hpi->out_max, hpi->out_min);
    return hpi->out;
}

/**
 * @brief Executes one step of the parallel-form IP controller.
 * @details P term acts on -feedback, I term acts on error. 
 * Prevents overshoot during setpoint changes.
 * @param[in,out] hpi Pointer to the PI controller instance.
 * @param[in] target The target reference value.
 * @param[in] feedback The current actual feedback value.
 * @return ctrl_gt The calculated saturated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ip_par(ctl_pi_t* hpi, ctrl_gt target, ctrl_gt feedback)
{
    ctrl_gt err = target - feedback;

    hpi->p_term = ctl_mul(-feedback, hpi->kp);
    hpi->i_term = ctl_sat(hpi->i_term + ctl_mul(err, hpi->ki), hpi->integral_max, hpi->integral_min);

    hpi->out = ctl_sat(hpi->p_term + hpi->i_term, hpi->out_max, hpi->out_min);
    return hpi->out;
}

/**
 * @brief Executes one step of the series-form (ideal) IP controller.
 * @details P term acts on -feedback, I term acts on error and is scaled by Kp.
 * @param[in,out] hpi Pointer to the PI controller instance.
 * @param[in] target The target reference value.
 * @param[in] feedback The current actual feedback value.
 * @return ctrl_gt The calculated saturated controller output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ip_ser(ctl_pi_t* hpi, ctrl_gt target, ctrl_gt feedback)
{
    ctrl_gt err = target - feedback;

    hpi->p_term = ctl_mul(-feedback, hpi->kp);

    // In series mode, the error going into the integrator must be scaled by Kp
    ctrl_gt err_scaled = ctl_mul(err, hpi->kp);
    hpi->i_term = ctl_sat(hpi->i_term + ctl_mul(err_scaled, hpi->ki), hpi->integral_max, hpi->integral_min);

    hpi->out = ctl_sat(hpi->p_term + hpi->i_term, hpi->out_max, hpi->out_min);
    return hpi->out;
}

/**
 * @brief Clears the internal states of the PI controller.
 * @param[out] hpi Pointer to the PI controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_pi(ctl_pi_t* hpi)
{
    hpi->p_term = float2ctrl(0.0f);
    hpi->i_term = float2ctrl(0.0f);
    hpi->out = float2ctrl(0.0f);
}

/**
 * @brief Set PI controller output limits.
 */
GMP_STATIC_INLINE void ctl_set_pi_limit(ctl_pi_t* hpi, ctrl_gt limit_max, ctrl_gt limit_min)
{
    hpi->out_max = limit_max;
    hpi->out_min = limit_min;
}

/**
 * @brief Set PI controller integral limits.
 */
GMP_STATIC_INLINE void ctl_set_pi_int_limit(ctl_pi_t* hpi, ctrl_gt limit_max, ctrl_gt limit_min)
{
    hpi->integral_max = limit_max;
    hpi->integral_min = limit_min;
}

/**
 * @brief Force the integrator state to a specific value.
 */
GMP_STATIC_INLINE void ctl_set_pi_integrator(ctl_pi_t* hpi, ctrl_gt integrator)
{
    hpi->i_term = integrator;
}

/**
 * @}
 */ // end of continuous_pi_controllers group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _CONTINUOUS_PI_H_
