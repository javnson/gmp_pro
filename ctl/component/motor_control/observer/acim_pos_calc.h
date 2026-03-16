/**
 * @file im_pos_calc.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements the Rotor Flux Position & Slip Estimator for ACIM (IFOC).
 * @details This module computes the slip frequency and electrical synchronous 
 * angle required for Indirect Field-Oriented Control (IFOC) when a physical 
 * speed sensor is available. It utilizes the motor's d-q axis currents and 
 * mechanical speed.
 * * * **Core Mathematical Architecture (Per-Unitized):**
 * - **Magnetizing Current LPF:** @f$ i_{md}[k] = i_{md}[k-1] + \frac{T_s}{\tau_r} (i_{sd}[k] - i_{md}[k-1]) @f$
 * - **Slip Frequency:** @f$ \omega_{slip} = \frac{i_{sq}}{\tau_r \cdot i_{md} \cdot \Omega_{base}} @f$
 * - **Synchronous Frequency:** @f$ \omega_{sync} = \omega_{r\_elec} + \omega_{slip} @f$
 * - **Angle Integration:** @f$ \theta_e[k] = \theta_e[k-1] + \omega_{sync} \cdot \frac{\Omega_{base} T_s}{2\pi} @f$
 *
 * @version 2.0
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#ifndef _FILE_IM_POS_CALC_H_
#define _FILE_IM_POS_CALC_H_

#include <gmp_core.h>

// --- Standard Math & Component Includes ---
#include <ctl/component/motor_control/consultant/im_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/math_block/gmp_math.h>

#include "math_ctrl_const.h"
#include "math_param_const.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* AC Induction Motor (IM) Position and Slip Calculator (IFOC)               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup IM_POS_CALC IM IFOC Position & Slip Calculator
 * @brief Sensored rotor flux angle estimator for Induction Motors.
 * @{
 */

/**
 * @brief Raw initialization structure for the IM Position Calculator.
 */
typedef struct _tag_im_pos_calc_init_t
{
    /** * @name Core Scale Factors (Derived from physical PU bases) */
    ///@{
    parameter_gt sf_lpf_kr;       //!< LPF constant for magnetizing current: @f$ T_s / \tau_r @f$.
    parameter_gt sf_slip_const;   //!< Slip calculation constant: @f$ 1 / (\tau_r \cdot \Omega_{base}) @f$.
    parameter_gt sf_mech_to_elec; //!< Ratio converting PU mechanical speed to PU electrical speed.
    parameter_gt sf_w_to_angle;   //!< Integration constant: @f$ \Omega_{base} T_s / 2\pi @f$.
    ///@}

    parameter_gt i_md_min_limit_pu; //!< Minimum magnetizing current threshold to prevent div-by-zero.

} ctl_im_pos_calc_init_t;

/**
 * @brief Main state structure for the IM IFOC Position Calculator.
 */
typedef struct _tag_im_pos_calc_t
{
    // --- Outputs ---
    rotation_ift enc_out; //!< Output interface providing the estimated synchronous angle.
    ctrl_gt w_slip_pu;    //!< Calculated motor slip speed (PU).
    ctrl_gt w_sync_pu;    //!< Calculated stator electrical (synchronous) frequency (PU).

    // --- Internal State Variables ---
    ctrl_gt i_md_pu; //!< Estimated d-axis magnetizing current (PU).

    // --- Pre-calculated Scale Factors ---
    ctrl_gt sf_lpf_kr;
    ctrl_gt sf_slip_const;
    ctrl_gt sf_mech_to_elec;
    ctrl_gt sf_w_to_angle;
    ctrl_gt i_md_min_limit;

    // --- Flags ---
    fast_gt flag_enable;

} ctl_im_pos_calc_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Core initialization function using explicit scale factors.
 * @param[out] calc Pointer to the calculator instance.
 * @param[in]  init Pointer to the raw initialization structure.
 */
void ctl_init_im_pos_calc(ctl_im_pos_calc_t* calc, const ctl_im_pos_calc_init_t* init);

/**
 * @brief Advanced initialization function utilizing the IM Consultant models.
 * @details Automatically calculates slip constants and integrators based on the 
 * rotor time constant (@f$ \tau_r @f$) and PU base values.
 * @param[out] calc  Pointer to the calculator instance.
 * @param[in]  motor Pointer to the IM physical model consultant.
 * @param[in]  pu    Pointer to the Per-Unit base model consultant.
 * @param[in]  fs    Controller execution frequency (Hz).
 */
void ctl_init_im_pos_calc_consultant(ctl_im_pos_calc_t* calc, const ctl_consultant_im_t* motor,
                                     const ctl_consultant_pu_im_t* pu, parameter_gt fs);

/**
 * @brief Safely clears all internal integrators and angle states.
 */
GMP_STATIC_INLINE void ctl_clear_im_pos_calc(ctl_im_pos_calc_t* calc)
{
    calc->i_md_pu = float2ctrl(0.0f);
    calc->w_slip_pu = float2ctrl(0.0f);
    calc->w_sync_pu = float2ctrl(0.0f);
    calc->enc_out.elec_position = float2ctrl(0.0f);
}

GMP_STATIC_INLINE void ctl_enable_im_pos_calc(ctl_im_pos_calc_t* calc)
{
    calc->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_im_pos_calc(ctl_im_pos_calc_t* calc)
{
    calc->flag_enable = 0;
}

/**
 * @brief Executes one high-frequency step of the IFOC Slip and Angle calculation.
 * @param[in,out] calc          Pointer to the calculator instance.
 * @param[in]     i_sd          Measured d-axis stator current (PU).
 * @param[in]     i_sq          Measured q-axis stator current (PU).
 * @param[in]     omega_mech_pu Measured mechanical rotor speed from encoder (PU).
 * @return The newly calculated electrical synchronous angle (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_im_pos_calc(ctl_im_pos_calc_t* calc, ctrl_gt i_sd, ctrl_gt i_sq,
                                               ctrl_gt omega_mech_pu)
{
    if (!calc->flag_enable)
        return calc->enc_out.elec_position;

    // ========================================================================
    // 1. Magnetizing Current Estimation (Rotor Flux Proxy)
    // ========================================================================
    // Low-pass filter: i_md[k] = i_md[k-1] + (Ts/tau_r) * (i_sd - i_md[k-1])
    calc->i_md_pu += ctl_mul(calc->sf_lpf_kr, i_sd - calc->i_md_pu);

    // ========================================================================
    // 2. Slip Speed Calculation (with robust Div-0 protection)
    // ========================================================================
    ctrl_gt abs_imd = (calc->i_md_pu > float2ctrl(0.0f)) ? calc->i_md_pu : -calc->i_md_pu;

    if (abs_imd < calc->i_md_min_limit)
    {
        // When unmagnetized (e.g., startup), slip calculation is singular.
        // Force slip to 0 instead of a massive transient spike.
        calc->w_slip_pu = float2ctrl(0.0f);
    }
    else
    {
        // Slip = sf_slip_const * (i_sq / i_md)
        calc->w_slip_pu = ctl_mul(calc->sf_slip_const, ctl_div(i_sq, calc->i_md_pu));
    }

    // ========================================================================
    // 3. Stator Synchronous Frequency
    // ========================================================================
    // Convert mechanical speed to electrical speed
    ctrl_gt w_r_elec = ctl_mul(omega_mech_pu, calc->sf_mech_to_elec);

    // Synchronous speed = Rotor Electrical Speed + Slip
    calc->w_sync_pu = w_r_elec + calc->w_slip_pu;

    // ========================================================================
    // 4. Flux Angle Integration
    // ========================================================================
    // Theta = Theta + w_sync * (W_base * Ts / 2PI)
    calc->enc_out.elec_position += ctl_mul(calc->w_sync_pu, calc->sf_w_to_angle);

    // Fast O(1) wrapping to [0.0, 1.0) PU
    calc->enc_out.elec_position = ctrl_mod_1(calc->enc_out.elec_position);

    return calc->enc_out.elec_position;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_POS_CALC_H_
