/**
 * @file acm.pos_calc.h
 * @brief Provides a rotor flux position estimator for AC Induction Motors (ACM).
 * @details This module implements the indirect field-oriented control (IFOC) method
 * for estimating the rotor flux angle. It uses the motor's d-q axis currents
 * and mechanical speed to calculate the slip speed and integrates the resulting
 * stator frequency to determine the electrical angle for sensorless FOC.
 *
 * //tex:
 * // The estimation is based on the following key equations:
 * // 1. Rotor flux linkage (first-order low-pass filter):
 * //    T_r \frac{d\psi_{dr}}{dt} + \psi_{dr} = L_m i_{ds}
 * // 2. Slip frequency calculation:
 * //    \omega_{slip} = \frac{L_m}{T_r \psi_{dr}} i_{qs} = \frac{R_r}{L_r} \frac{L_m}{\psi_{dr}} i_{qs}
 * // 3. Stator frequency (field angle velocity):
 * //    \omega_s = \omega_e = \omega_r + \omega_{slip}
 *
 */

#ifndef _FILE_ACM_POS_CALC_H_
#define _FILE_ACM_POS_CALC_H_

#include <ctl/component/motor_control/basic/encoder.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* AC Induction Motor (ACM) Position and Slip Calculator                     */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ACM_POS_CALC ACM Position Calculator
 * @brief A module for estimating rotor flux angle in an AC Induction Motor.
 * @details Implements the core calculations for indirect field-oriented control (IFOC).
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Holds the state and parameters for the ACM slip and angle calculator.
 */
typedef struct _tag_im_spd_calc
{
    // --- Outputs ---
    ctrl_gt slip;     ///< Calculated motor slip speed (p.u.).
    rotation_ift enc; ///< Output encoder interface providing the estimated electrical angle.

    // --- Internal State Variables ---
    ctrl_gt imds;    ///< Estimated d-axis magnetizing current, proportional to rotor flux (p.u.).
    ctrl_gt omega_s; ///< Calculated stator electrical frequency (p.u.).

    // --- Pre-calculated Parameters ---
    ctrl_gt kr;     ///< Magnetizing current filter constant, related to rotor time constant Tr.
    ctrl_gt kt;     ///< Slip calculation constant, related to rotor time constant Tr.
    ctrl_gt ktheta; ///< Rotor flux angle integration constant.
    ctrl_gt ksync;  ///< Constant to convert mechanical speed (p.u.) to electrical frequency (p.u.).

} ctl_im_spd_calc_t;

//================================================================================
// Function Prototypes
//================================================================================

/**
 * @brief Initializes the ACM speed and position calculator object.
 * @param[out] calc Pointer to the calculator structure to be initialized.
 * @param[in] Rr Rotor resistance (Ohm).
 * @param[in] Lr Rotor inductance (H).
 * @param[in] rotor_base Base rotor speed for per-unit conversion (RPM).
 * @param[in] pole_pairs Number of motor pole pairs.
 * @param[in] freq_base Base electrical frequency for per-unit conversion (Hz).
 * @param[in] isr_freq The frequency of the controller's ISR (Hz).
 */
void ctl_init_im_spd_calc(ctl_im_spd_calc_t* calc, parameter_gt Rr, parameter_gt Lr, parameter_gt rotor_base,
                          uint16_t pole_pairs, parameter_gt freq_base, parameter_gt isr_freq);

/**
 * @brief Clears the internal states of the calculator.
 * @details Resets slip, frequencies, and angle to zero. This should be called
 * when stopping the motor.
 * @param[out] calc Pointer to the calculator structure.
 */
GMP_STATIC_INLINE void ctl_clear_im_spd_calc(ctl_im_spd_calc_t* calc)
{
    calc->imds = float2ctrl(0.0);
    calc->slip = float2ctrl(0.0);
    calc->omega_s = float2ctrl(0.0);
    calc->enc.elec_position = float2ctrl(0.0);
    calc->enc.position = float2ctrl(0.0);
}

/**
 * @brief Executes one step of the slip and angle calculation.
 * @details This function should be called periodically in the main control ISR.
 * It takes the d-q currents and rotor speed as inputs and updates the
 * estimated electrical angle.
 * @param[out] calc     Pointer to the calculator structure.
 * @param[in]  isd      The measured d-axis stator current (p.u.).
 * @param[in]  isq      The measured q-axis stator current (p.u.).
 * @param[in]  omega_r  The measured rotor mechanical speed (p.u.).
 * @return The newly calculated electrical angle (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_im_spd_calc(ctl_im_spd_calc_t* calc, ctrl_gt isd, ctrl_gt isq, ctrl_gt omega_r)
{
    // 1. Estimate the rotor magnetizing current (proportional to rotor flux)
    // This is a discrete-time low-pass filter: imds(k) = (1-a)*imds(k-1) + a*isd(k)
    calc->imds += ctl_mul(calc->kr, (isd - calc->imds));

    // 2. Calculate the slip speed.
    // To prevent division by zero at startup, a fallback is used.
    if (ctl_abs(calc->imds) < float2ctrl(0.005) || ctl_abs(calc->imds) > -float2ctrl(0.005))
    {
        // If magnetizing current is near zero, set slip to a high value to avoid instability.
        calc->slip = float2ctrl(1.0);
    }
    else
    {
        calc->slip = ctl_div(ctl_mul(calc->kt, isq), calc->imds);
    }

    // 3. Calculate the stator electrical frequency.
    // omega_s = omega_r_elec + slip_speed
    // A small constant boost is added to improve low-speed performance.
    calc->omega_s = ctl_mul(omega_r, calc->ksync) + calc->slip + float2ctrl(0.03);

    // 4. Integrate the stator frequency to get the electrical angle.
    calc->enc.elec_position += ctl_mul(calc->ktheta, calc->omega_s);

    // 5. Wrap the angle to the range [0, 1.0)
    calc->enc.elec_position = ctrl_mod_1(float2ctrl(1.0) + calc->enc.elec_position);

    // The mechanical position is assumed to be the same as electrical for this estimator's purpose.
    calc->enc.position = calc->enc.elec_position;

    return calc->enc.elec_position;
}

/** @} */ // end of ACM_POS_CALC group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ACM_POS_CALC_H_
