/**
 * @file discrete_sogi.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a discrete Second-Order Generalized Integrator (SOGI).
 * @version 0.1
 * @date 2024-08-06
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _DISCRETE_SOGI_H_
#define _DISCRETE_SOGI_H_

#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup discrete_sogi SOGI-based Quadrature Signal Generator
 * @brief Implements a SOGI to generate in-phase and quadrature-phase signals.
 * @details This module implements a discrete Second-Order Generalized Integrator (SOGI),
 * which is a versatile filter commonly used in power electronics and grid-tied
 * applications. Its primary function is to act as a frequency-adaptive band-pass
 * filter that can isolate the fundamental component of an input signal while
 * simultaneously generating a quadrature (90-degree phase-shifted) version of it.
 * This makes it essential for algorithms like Phase-Locked Loops (PLLs).
 * A SOGI provides two outputs from a single input R(s). The direct
 * output D(s) is in-phase with the input's fundamental component and acts as a
 * band-pass filter. The quadrature output Q(s) lags the direct output by 90
 * degrees and acts as a low-pass filter.
 *
 * Band-Pass Filter Transfer Function (Direct Output D(s)):
 * @f[ \frac{D(s)}{R(s)} = \frac{k \omega_0 s}{s^2 + k \omega_0 s + \omega_0^2} @f]
 *
 * Low-Pass Filter Transfer Function (Quadrature Output Q(s)):
 * @f[ \frac{Q(s)}{R(s)} = \frac{k \omega_0^2}{s^2 + k \omega_0 s + \omega_0^2} @f]
 * where @f$ \omega_0 @f$ is the center frequency and @f$ k @f$ is the damping coefficient.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Discrete Second-Order Generalized Integrator (SOGI)                       */
/*---------------------------------------------------------------------------*/

/**
 * @brief SOGI Continuous-Time Transfer Functions.
 * @details A SOGI provides two outputs from a single input R(s). The direct
 * output D(s) is in-phase with the input's fundamental component and acts as a
 * band-pass filter. The quadrature output Q(s) lags the direct output by 90
 * degrees and acts as a low-pass filter.
 *
 * Band-Pass Filter Transfer Function (Direct Output D(s)):
 * @f[ \frac{D(s)}{R(s)} = \frac{k \omega_0 s}{s^2 + k \omega_0 s + \omega_0^2} @f]
 *
 * Low-Pass Filter Transfer Function (Quadrature Output Q(s)):
 * @f[ \frac{Q(s)}{R(s)} = \frac{k \omega_0^2}{s^2 + k \omega_0 s + \omega_0^2} @f]
 * where @f$ \omega_0 @f$ is the center frequency and @f$ k @f$ is the damping coefficient.
 */
typedef struct _tag_discrete_sogi
{
    // Input buffer: u[0]=u(k), u[1]=u(k-1), u[2]=u(k-2)
    ctrl_gt u[3];

    // Direct (in-phase) signal output buffer: osg_u[0]=y_d(k), ...
    ctrl_gt osg_u[3];

    // Quadrature signal output buffer: osg_qu[0]=y_q(k), ...
    ctrl_gt osg_qu[3];

    // Filter coefficients derived from discretization
    ctrl_gt b0;  //!< Coefficient for the direct output difference equation.
    ctrl_gt b2;  //!< Coefficient for the direct output difference equation.
    ctrl_gt a1;  //!< Coefficient for both difference equations (shared denominator).
    ctrl_gt a2;  //!< Coefficient for both difference equations (shared denominator).
    ctrl_gt qb0; //!< Coefficient for the quadrature output difference equation.
    ctrl_gt qb1; //!< Coefficient for the quadrature output difference equation.
    ctrl_gt qb2; //!< Coefficient for the quadrature output difference equation.
} discrete_sogi_t;

/**
 * @brief Initializes the discrete SOGI module.
 * @details Calculates the difference equation coefficients based on the system
 * parameters and clears the internal states.
 * @param[out] sogi Pointer to the discrete_sogi_t instance.
 * @param[in] k_damp Damping coefficient (k), typically sqrt(2) or 1.
 * @param[in] fn Center frequency in Hz.
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_discrete_sogi(discrete_sogi_t* sogi, parameter_gt k_damp, parameter_gt fn, parameter_gt fs);

/**
 * @brief Clears the internal states of the SOGI module.
 * @details Resets all historical input and output buffers to zero.
 * @param[out] sogi Pointer to the discrete_sogi_t instance.
 */
GMP_STATIC_INLINE void ctl_clear_discrete_sogi(discrete_sogi_t* sogi)
{
    sogi->u[0] = 0;
    sogi->u[1] = 0;
    sogi->u[2] = 0;

    sogi->osg_u[0] = 0;
    sogi->osg_u[1] = 0;
    sogi->osg_u[2] = 0;

    sogi->osg_qu[0] = 0;
    sogi->osg_qu[1] = 0;
    sogi->osg_qu[2] = 0;
}

/**
 * @brief Executes one step of the SOGI calculation.
 * @param[in,out] sogi Pointer to the discrete_sogi_t instance.
 * @param[in] u The current input sample.
 */
GMP_STATIC_INLINE void ctl_step_discrete_sogi(discrete_sogi_t* sogi, ctrl_gt u)
{
    // Store current input
    sogi->u[0] = u;

    // Calculate the direct (in-phase) output y_d(k)
    sogi->osg_u[0] = ctl_mul(sogi->b0, (sogi->u[0] - sogi->u[2])) + ctl_mul(sogi->a1, sogi->osg_u[1]) +
                     ctl_mul(sogi->a2, sogi->osg_u[2]);

    // Calculate the quadrature output y_q(k)
    sogi->osg_qu[0] = ctl_mul(sogi->qb0, sogi->u[0]) + ctl_mul(sogi->qb1, sogi->u[1]) + ctl_mul(sogi->qb2, sogi->u[2]) +
                      ctl_mul(sogi->a1, sogi->osg_qu[1]) + ctl_mul(sogi->a2, sogi->osg_qu[2]);

    // Update state buffers for the next iteration
    sogi->u[2] = sogi->u[1];
    sogi->u[1] = sogi->u[0];

    sogi->osg_u[2] = sogi->osg_u[1];
    sogi->osg_u[1] = sogi->osg_u[0];

    sogi->osg_qu[2] = sogi->osg_qu[1];
    sogi->osg_qu[1] = sogi->osg_qu[0];
}

/**
 * @brief Gets the latest direct (in-phase) output.
 * @param[in] sogi Pointer to the discrete_sogi_t instance.
 * @return ctrl_gt The direct signal component, D(s).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_discrete_sogi_ds(discrete_sogi_t* sogi)
{
    return sogi->osg_u[0];
}

/**
 * @brief Gets the latest quadrature (90-degree shifted) output.
 * @param[in] sogi Pointer to the discrete_sogi_t instance.
 * @return ctrl_gt The quadrature signal component, Q(s).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_discrete_sogi_qs(discrete_sogi_t* sogi)
{
    return sogi->osg_qu[0];
}

/**
 * @defgroup discrete_sogi_dc SOGI-based Quadrature Signal Generator with DC Rejection
 * @brief Implements a SOGI with an additional loop to reject DC offset.
 * @details This module extends the standard SOGI by adding a DC estimation loop.
 * Ideally, the alpha channel of a standard SOGI rejects DC (gain=0 at 0Hz). 
 * However, any DC offset in the input passes through to the beta channel (integrator),
 * causing low-frequency oscillation in the DQ frame (50/60Hz ripple).
 *
 * This module estimates the DC component and subtracts it from the input before
 * feeding it to the SOGI core, ensuring pure AC signals for alpha and beta outputs.
 * 
 * generally, @f[ k_{dc} \approx 0.5 ~ 1.0 @f]
 *
 * Transfer Functions (Simplified structural view):
 *
 * Direct Output (Alpha) D(s):
 * @f[ \frac{D(s)}{R(s)} = \frac{k \omega_0 s^2}{s^3 + (k + k_{dc})\omega_0 s^2 + \omega_0^2 s + k_{dc}\omega_0^3} @f]
 *
 * Quadrature Output (Beta) Q(s):
 * @f[ \frac{Q(s)}{R(s)} = \frac{k \omega_0^2 s}{s^3 + (k + k_{dc})\omega_0 s^2 + \omega_0^2 s + k_{dc}\omega_0^3} @f]
 *
 * DC Estimation Output DC(s):
 * @f[ \frac{DC(s)}{R(s)} = \frac{k_{dc} \omega_0 (s^2 + \omega_0^2)}{s^3 + (k + k_{dc})\omega_0 s^2 + \omega_0^2 s + k_{dc}\omega_0^3} @f]
 *
 * where @f$ k_{dc} @f$ is the DC rejection gain.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Discrete SOGI with DC Rejection (SOGI-DC)                                 */
/*---------------------------------------------------------------------------*/

typedef struct _tag_discrete_sogi_dc
{
    // --- Sub-modules ---
    discrete_sogi_t core; //!< The standard SOGI core handling the AC dynamics.

    // --- Internal States ---
    ctrl_gt v_dc_est;   //!< Estimated DC offset voltage.
    ctrl_gt input_prev; //!< Previous input value (for loop calculation).

    // --- Parameters ---
    ctrl_gt dc_integ_gain; //!< Pre-calculated gain for DC integrator: Ts * omega0 * k_dc.

} discrete_sogi_dc_t;

/**
 * @brief Initializes the SOGI-DC module.
 * @param[out] sogi_dc Pointer to the SOGI-DC instance.
 * @param[in] k_damp SOGI damping coefficient (typically 1.414).
 * @param[in] k_dc DC rejection gain (typically 0.5 to 1.0). Lower is slower but more stable.
 * @param[in] fn Center frequency in Hz.
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_discrete_sogi_dc(discrete_sogi_dc_t* sogi_dc, parameter_gt k_damp, parameter_gt k_dc, parameter_gt fn,
                               parameter_gt fs);

/**
 * @brief Clears the internal states of the SOGI-DC module.
 */
GMP_STATIC_INLINE void ctl_clear_discrete_sogi_dc(discrete_sogi_dc_t* sogi_dc)
{
    // Clear Core
    ctl_clear_discrete_sogi(&sogi_dc->core);

    // Clear DC States
    sogi_dc->v_dc_est = 0;
    sogi_dc->input_prev = 0;
}

/**
 * @brief Executes one step of the SOGI-DC calculation.
 * @details
 * 1. Update DC estimate based on previous error.
 * 2. Subtract DC from current Input.
 * 3. Step standard SOGI.
 * @param[in,out] sogi_dc Pointer to the instance.
 * @param[in] u The current raw input sample (containing AC + DC).
 */
GMP_STATIC_INLINE void ctl_step_discrete_sogi_dc(discrete_sogi_dc_t* sogi_dc, ctrl_gt u)
{
    // 1. DC Estimation Loop (Integrator)
    // Error = (Input_clean) - Alpha
    // Input_clean was (Input_raw - v_dc_est)
    // So Error = Input_raw - v_dc_est - Alpha
    // Note: We use previous values to avoid algebraic loop in simplest implementation.

    ctrl_gt alpha_prev = ctl_get_discrete_sogi_ds(&sogi_dc->core);

    // Calculate the "leakage" that the SOGI core didn't catch (which is DC)
    // Error signal driving the DC integrator
    ctrl_gt dc_loop_err = (sogi_dc->input_prev - sogi_dc->v_dc_est) - alpha_prev;

    // Update DC estimate: v_dc[k] = v_dc[k-1] + Gain * Error
    sogi_dc->v_dc_est += ctl_mul(dc_loop_err, sogi_dc->dc_integ_gain);

    // 2. Clean Input (Remove DC)
    ctrl_gt u_clean = u - sogi_dc->v_dc_est;

    // 3. Step SOGI Core
    ctl_step_discrete_sogi(&sogi_dc->core, u_clean);

    // Store current input for next step
    sogi_dc->input_prev = u;
}

/**
 * @brief Gets the latest direct (alpha) output (DC-free).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_discrete_sogi_dc_alpha(discrete_sogi_dc_t* sogi_dc)
{
    return ctl_get_discrete_sogi_ds(&sogi_dc->core);
}

/**
 * @brief Gets the latest quadrature (beta) output (DC-free).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_discrete_sogi_dc_beta(discrete_sogi_dc_t* sogi_dc)
{
    return ctl_get_discrete_sogi_qs(&sogi_dc->core);
}

/**
 * @brief Gets the estimated DC offset.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_discrete_sogi_dc_offset(discrete_sogi_dc_t* sogi_dc)
{
    return sogi_dc->v_dc_est;
}

/**
 * @}
 */ // end of discrete_sogi group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _DISCRETE_SOGI_H_
