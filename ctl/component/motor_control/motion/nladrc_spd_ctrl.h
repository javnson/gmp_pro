/**
 * @file nladrc_controller.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a Nonlinear Active Disturbance Rejection Controller (NLADRC) for a second-order system.
 * @version 1.0
 * @date 2025-08-08
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _NLADRC_CONTROLLER_H_
#define _NLADRC_CONTROLLER_H_

#include <ctl/math_block/gmp_math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup nladrc_controller Nonlinear ADRC
 * @brief An advanced controller for systems with uncertainties and disturbances.
 * @details This file implements a complete second-order NLADRC, which is ideal
 * for applications like motor speed control. The controller consists of three main parts:
 * 1.  Tracking Differentiator (TD): Arranges a smooth transition for the setpoint
 * and provides its differential signal.
 * 2.  Extended State Observer (ESO): Estimates the system states (e.g., speed and acceleration)
 * and the "total disturbance" (including internal dynamics and external loads).
 * 3.  Nonlinear State Error Feedback (NLSEF): Calculates the control output based on
 * the errors between the setpoint trajectory and the estimated states.
 *
 * This implementation is based on the standard discrete-time formulas for NLADRC.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Helper Functions                                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief A helper function to get the sign of a value.
 */
GMP_STATIC_INLINE ctrl_gt _nladrc_sign(ctrl_gt val)
{
    if (val > 0.0f)
        return 1.0f;
    if (val < 0.0f)
        return -1.0f;
    return 0.0f;
}

/**
 * @brief The core nonlinear function 'fal' used in NLADRC.
 * @details Implements fal(e, alpha, delta) = |e|^alpha * sign(e) for |e| > delta,
 * and e / delta^(1-alpha) for |e| <= delta. This function provides high gain
 * for small errors and smaller gain for large errors, improving performance.
 * @param e The error input.
 * @param alpha The power exponent (typically 0 < alpha < 1).
 * @param delta The width of the linear region around zero.
 * @return The result of the fal function.
 */
GMP_STATIC_INLINE ctrl_gt _nladrc_fal(ctrl_gt e, ctrl_gt alpha, ctrl_gt delta)
{
    if (ctl_abs(e) > delta)
    {
        return ctl_pow(ctl_abs(e), alpha) * _nladrc_sign(e);
    }
    else
    {
        return e / ctl_pow(delta, 1.0f - alpha);
    }
}

/*---------------------------------------------------------------------------*/
/* Nonlinear Active Disturbance Rejection Controller (NLADRC)                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Tracking Differentiator (TD).
 */
typedef struct
{
    ctrl_gt x1; // State 1: Tracked setpoint (v1)
    ctrl_gt x2; // State 2: Differentiated setpoint (v2)
    ctrl_gt r;  // Speed factor
    ctrl_gt h;  // Sampling period (Ts)
} ctl_nladrc_td_t;

/**
 * @brief Data structure for the Extended State Observer (ESO).
 */
typedef struct
{
    ctrl_gt z1; // State 1: Estimated output (e.g., speed)
    ctrl_gt z2; // State 2: Estimated derivative (e.g., acceleration)
    ctrl_gt z3; // State 3: Estimated total disturbance (f)

    ctrl_gt beta01; // Observer gain 1
    ctrl_gt beta02; // Observer gain 2
    ctrl_gt beta03; // Observer gain 3

    ctrl_gt h;  // Sampling period (Ts)
    ctrl_gt b0; // Control gain estimate
} ctl_nladrc_eso_t;

/**
 * @brief Data structure for the Nonlinear State Error Feedback (NLSEF).
 */
typedef struct
{
    ctrl_gt kp; // Proportional gain (derived from beta_01)
    ctrl_gt kd; // Derivative gain (derived from beta_02)

    // Nonlinear gain parameters
    ctrl_gt alpha1; // Power exponent for e1
    ctrl_gt alpha2; // Power exponent for e2
    ctrl_gt delta;  // Linear region width for fal()
} ctl_nladrc_nlsef_t;

/**
 * @brief Main data structure for the NLADRC controller.
 */
typedef struct _tag_nladrc_controller_t
{
    ctl_nladrc_td_t td;
    ctl_nladrc_eso_t eso;
    ctl_nladrc_nlsef_t nlsef;

    ctrl_gt output; // Final control output
    ctrl_gt out_max;
    ctrl_gt out_min;
} ctl_nladrc_controller_t;

/**
 * @brief Initializes the NLADRC controller.
 * @param[out] adrc Pointer to the NLADRC instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] r TD speed factor. Larger r means faster tracking.
 * @param[in] omega_o Observer bandwidth in rad/s. Determines how fast the ESO converges.
 * @param[in] omega_c Controller bandwidth in rad/s. Determines the closed-loop response speed.
 * @param[in] b0 Estimate of the control gain of the plant. This is a critical tuning parameter.
 * @param[in] alpha1 NLSEF nonlinear power for position error (typically 0.5-0.75).
 * @param[in] alpha2 NLSEF nonlinear power for velocity error (typically > 1, e.g., 1.5).
 */
void ctl_init_nladrc(ctl_nladrc_controller_t* adrc, parameter_gt fs, parameter_gt r, parameter_gt omega_o,
                     parameter_gt omega_c, parameter_gt b0, parameter_gt alpha1, parameter_gt alpha2);

/**
 * @brief Sets the output limits for the NLADRC controller.
 * @param[out] adrc Pointer to the NLADRC instance.
 * @param[in] limit_max The maximum output value.
 * @param[in] limit_min The minimum output value.
 */
GMP_STATIC_INLINE void ctl_set_nladrc_limit(ctl_nladrc_controller_t* adrc, ctrl_gt limit_max, ctrl_gt limit_min)
{
    adrc->out_max = limit_max;
    adrc->out_min = limit_min;
}

/**
 * @brief Executes one step of the NLADRC controller.
 * @param[in,out] adrc Pointer to the NLADRC instance.
 * @param[in] target The speed setpoint (target).
 * @param[in] feedback The measured speed from the system (feedback).
 * @return ctrl_gt The calculated control output (e.g., current reference).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_nladrc(ctl_nladrc_controller_t* adrc, ctrl_gt target, ctrl_gt feedback)
{
    // --- 1. Tracking Differentiator (TD) ---
    // Calculate the target trajectory v1 and its derivative v2
    ctrl_gt fh = _nladrc_fal(adrc->td.x1 - target, 0.5f, adrc->td.h * adrc->td.r);
    adrc->td.x1 += adrc->td.h * adrc->td.x2;
    adrc->td.x2 += adrc->td.h * fh;

    // --- 2. Extended State Observer (ESO) ---
    // Update state estimates using the previous control input u(k-1) and the current system output y(k)
    ctrl_gt e = adrc->eso.z1 - feedback;            // Observation error
    ctrl_gt fe = _nladrc_fal(e, 0.5f, adrc->eso.h); // Nonlinear function
    ctrl_gt fe1 = _nladrc_fal(e, 0.25f, adrc->eso.h);

    adrc->eso.z1 += adrc->eso.h * (adrc->eso.z2 - adrc->eso.beta01 * e);
    adrc->eso.z2 +=
        adrc->eso.h * (adrc->eso.z3 - adrc->eso.beta02 * fe + adrc->eso.b0 * adrc->output); // u(k-1) is adrc->output
    adrc->eso.z3 += adrc->eso.h * (-adrc->eso.beta03 * fe1);

    // --- 3. Nonlinear State Error Feedback (NLSEF) ---
    // Calculate state errors
    ctrl_gt e1 = adrc->td.x1 - adrc->eso.z1; // Position/speed error
    ctrl_gt e2 = adrc->td.x2 - adrc->eso.z2; // Velocity/acceleration error

    // Calculate the nonlinear combination u0
    ctrl_gt u0 = adrc->nlsef.kp * _nladrc_fal(e1, adrc->nlsef.alpha1, adrc->nlsef.delta) +
                 adrc->nlsef.kd * _nladrc_fal(e2, adrc->nlsef.alpha2, adrc->nlsef.delta);

    // --- Key Point Analysis 4: Total Disturbance Compensation ---
    // This is the essence of ADRC: subtract the total disturbance z3 estimated by the ESO from the controller output u0, then divide by the control gain b0.
    // This operation actively cancels out all internal and external disturbances in real-time.
    adrc->output = (u0 - adrc->eso.z3) / adrc->eso.b0;

    // Output saturation
    adrc->output = ctl_sat(adrc->output, adrc->out_max, adrc->out_min);

    return adrc->output;
}

/**
 * @}
 */ // end of nladrc_controller group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _NLADRC_CONTROLLER_H_
