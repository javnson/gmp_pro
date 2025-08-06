/**
 * @file continuous_sogi.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a continuous-form Second-Order Generalized Integrator (SOGI).
 * @version 0.2
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a SOGI based on the state-space representation
 * of the continuous-time transfer function, discretized using Forward Euler.
 * A SOGI is a frequency-adaptive filter that generates in-phase and quadrature
 * sinusoidal signals from a given input, making it ideal for PLLs and filtering.
 */

#ifndef _CONTINUOUS_SOGI_H_
#define _CONTINUOUS_SOGI_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup continuous_sogi Continuous-Form SOGI
 * @brief A SOGI implementation based on discretized continuous-time state equations.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Continuous-Form Second-Order Generalized Integrator (SOGI)                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a continuous-form SOGI.
 * @details Implements a SOGI based on the state-space equations:
 * @f[
 * \frac{d(y_d)}{dt} = \omega_r (k(u - y_d) - y_q)
 * @f]
 * @f[
 * \frac{d(y_q)}{dt} = \omega_r y_d
 * @f]
 * where @f$ y_d @f$ is the direct (in-phase) output and @f$ y_q @f$ is the
 * quadrature output.
 *
 * The corresponding transfer functions are:
 * Direct (Band-Pass) Output:
 * @f[
 * \frac{Y_d(s)}{U(s)} = \frac{k \omega_r s}{s^2 + k \omega_r s + \omega_r^2}
 * @f]
 * Quadrature (Low-Pass) Output:
 * @f[
 * \frac{Y_q(s)}{U(s)} = \frac{k \omega_r^2}{s^2 + k \omega_r s + \omega_r^2}
 * @f]
 */
typedef struct _tag_sogi_t
{
    // Parameters
    ctrl_gt k_damp; //!< Damping factor (k).
    ctrl_gt k_r;    //!< Resonant frequency gain (omega_r * T).
    ctrl_gt gain;   //!< Overall output gain.

    // State variables
    ctrl_gt d_integrate;         //!< Direct (in-phase) integrator state, y_d.
    ctrl_gt q_integrate;         //!< Quadrature integrator state, y_q.
    ctrl_gt integrate_reference; //!< Intermediate variable for calculation.

} ctl_sogi_t;

/**
 * @brief Initializes the continuous-form SOGI controller.
 * @param[out] sogi Pointer to the SOGI instance.
 * @param[in] gain Overall gain of the controller.
 * @param[in] freq_r Resonant frequency in Hz.
 * @param[in] damp Damping factor k (a value of sqrt(2) is common).
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_sogi(ctl_sogi_t* sogi, parameter_gt gain, parameter_gt freq_r, parameter_gt damp, parameter_gt fs);

/**
 * @brief Clears the internal states of the SOGI controller.
 * @param[out] sogi Pointer to the SOGI instance.
 */
GMP_STATIC_INLINE void ctl_clear_sogi(ctl_sogi_t* sogi)
{
    sogi->d_integrate = 0;
    sogi->q_integrate = 0;
    sogi->integrate_reference = 0;
}

/**
 * @brief Executes one step of the SOGI controller.
 * @param[in,out] sogi Pointer to the SOGI instance.
 * @param[in] input The current input signal.
 * @return ctrl_gt The gain-adjusted direct (in-phase) output.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sogi(ctl_sogi_t* sogi, ctrl_gt input)
{
    // err = u - y_d
    ctrl_gt err = input - sogi->d_integrate;
    // ref = k * err
    sogi->integrate_reference = ctl_mul(sogi->k_damp, err);
    // y_d(n) = y_d(n-1) + wr*T * (ref - y_q(n-1))
    sogi->d_integrate += ctl_mul(sogi->k_r, sogi->integrate_reference - sogi->q_integrate);
    // y_q(n) = y_q(n-1) + wr*T * y_d(n)
    sogi->q_integrate += ctl_mul(sogi->k_r, sogi->d_integrate);

    return ctl_mul(sogi->d_integrate, sogi->gain);
}

/**
 * @brief Gets the direct (in-phase) output before gain application.
 * @param[in] sogi Pointer to the SOGI instance.
 * @return ctrl_gt The direct signal component.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_sogi_direct_out(ctl_sogi_t* sogi)
{
    return sogi->d_integrate;
}

/**
 * @brief Gets the quadrature output.
 * @param[in] sogi Pointer to the SOGI instance.
 * @return ctrl_gt The quadrature signal component.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_sogi_quadrature_out(ctl_sogi_t* sogi)
{
    return sogi->q_integrate;
}

/**
 * @}
 */ // end of continuous_sogi group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _CONTINUOUS_SOGI_H_
