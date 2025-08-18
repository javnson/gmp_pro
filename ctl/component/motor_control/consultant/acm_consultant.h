/**
 * @file acm_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a data structure and utility functions for AC induction motor parameters.
 * @version 0.3
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_IM_CONSULTANT_H_
#define _FILE_IM_CONSULTANT_H_

#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h> // Include for per-unit system base values
#include <ctl/component/motor_control/consultant/motor_unit_calculator.h> // Include for physical constants and unit conversions

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Induction Motor Design Consultant Structure                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_IM_CONSULTANT Induction Motor Design Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief A structure to hold all design parameters for an induction motor.
 * @details This module provides a "consultant" structure for key electrical and mechanical
 * parameters of an AC induction motor, along with functions to calculate derived
 * constants, estimate controller tuning parameters, and convert parameters to the per-unit system.

 */

/**
 * @brief Data structure for holding AC Induction Motor (IM) design parameters.
 *
 * This structure aggregates all the necessary nameplate and design parameters
 * for an induction motor, which are required for simulation and advanced
 * control algorithms.
 */
typedef struct _tag_im_dsn_consultant
{
    // Electrical Parameters
    uint16_t pole_pair; /**< @brief Number of motor pole pairs. */
    parameter_gt Rs;    /**< @brief Stator resistance in Ohms (ohm). */
    parameter_gt Ls;    /**< @brief Stator inductance in Henrys (H). */
    parameter_gt Rr;    /**< @brief Rotor resistance in Ohms (ohm), referred to the stator. */
    parameter_gt Lr;    /**< @brief Rotor inductance in Henrys (H), referred to the stator. */
    parameter_gt Lm;    /**< @brief Mutual inductance in Henrys (H). */

    // Mechanical Parameters
    parameter_gt inertia; /**< @brief Moment of inertia (J) in kgm2. */
    parameter_gt damp;    /**< @brief Damping factor (B) or friction coefficient in Nms. */

} im_dsn_consultant_t;

/**
 * @brief Initializes the induction motor design consultant structure with provided parameters.
 *
 * @param[out] im_dsn Pointer to the induction motor consultant structure to be initialized.
 * @param[in] pole_pair Number of motor pole pairs.
 * @param[in] Rs Stator resistance (Ω).
 * @param[in] Ls Stator inductance (H).
 * @param[in] Rr Rotor resistance (Ω).
 * @param[in] Lr Rotor inductance (H).
 * @param[in] Lm Mutual inductance (H).
 * @param[in] inertia Moment of inertia (kg·m²).
 * @param[in] damp Damping factor.
 */
void ctl_setup_im_dsn_consultant(im_dsn_consultant_t* im_dsn, uint16_t pole_pair, parameter_gt Rs, parameter_gt Ls,
                                 parameter_gt Rr, parameter_gt Lr, parameter_gt Lm, parameter_gt inertia,
                                 parameter_gt damp);

/**
 * @brief Initializes the induction motor design consultant structure with experiment.
 *
 * @param[out] im_dsn Pointer to the induction motor consultant structure to be initialized.
 * @param[in] noload_data noload experiment result
 * @param[in] lockedrotor_data rotor locked experiment result
 * @param[in] pole_pair Number of motor pole pairs.
 * @param[in] test_req_Hz test frequency
 * @param[in] R_dc_line_to_line RDC from line to line.
 */
fast_gt ctl_init_im_dsn_consultant_by_experiment(im_dsn_consultant_t* acm, motor_test_data_t* noload_data,
                                                 motor_test_data_t* lockedrotor_data, uint16_t pole_pairs,
                                                 parameter_gt test_freq_Hz, parameter_gt R_dc_line_to_line)
{
    // --- Input Validation ---
    if (!acm || !noload_data || !lockedrotor_data || test_freq_Hz <= 0)
    {
        return false;
    }
    if (noload_data->I_line <= 0 || lockedrotor_data->I_line <= 0)
    {
        return false; // Avoid division by zero
    }

    // --- Basic Parameters ---
    acm->pole_pair = pole_pairs;
    parameter_gt omega_e = 2.0f * M_PI * test_freq_Hz;

    // --- Step 1: Stator Resistance (from DC Test) ---
    // Assuming a Y-connected motor, phase resistance is half the line-to-line resistance.
    acm->Rs = R_dc_line_to_line / 2.0f;

    // --- Step 2: Locked-Rotor Test Analysis (to find Rr and leakage inductances) ---
    parameter_gt Vlr_ph = lockedrotor_data->V_line / (parameter_gt)sqrtf(3.0);
    parameter_gt Ilr_ph = lockedrotor_data->I_line;

    // Total resistance and impedance from the locked-rotor test
    parameter_gt R_lr = lockedrotor_data->P_total / (3.0f * Ilr_ph * Ilr_ph);
    parameter_gt Z_lr = Vlr_ph / Ilr_ph;

    // Calculate rotor resistance (Rr')
    acm->Rr = R_lr - acm->Rs;

    // Calculate total leakage reactance (Xlk = Xls + Xr')
    parameter_gt X_lk = (parameter_gt)sqrtf(Z_lr * Z_lr - R_lr * R_lr);

    // --- Step 3: No-Load Test Analysis (to find magnetizing inductance) ---
    parameter_gt Vnl_ph = noload_data->V_line / (parameter_gt)sqrtf(3.0);
    parameter_gt Inl_ph = noload_data->I_line;

    // Calculate apparent and reactive power from no-load test
    parameter_gt S_nl = 3.0f * Vnl_ph * Inl_ph;
    parameter_gt Q_nl = (parameter_gt)sqrtf(S_nl * S_nl - noload_data->P_total * noload_data->P_total);

    // The no-load reactance is approximately Xls + Xm
    parameter_gt X_nl = Q_nl / (3.0f * Inl_ph * Inl_ph);

    // --- Step 4: Separate Inductances and Finalize ---
    // An assumption is required to split the total leakage reactance X_lk into
    // stator (Xls) and rotor (Xr') components. A common IEEE standard assumption
    // for NEMA Class A/B motors is to split them equally.
    parameter_gt X_ls = X_lk / 2.0f;
    parameter_gt X_r_prime = X_lk / 2.0f;

    // Now find the magnetizing reactance
    parameter_gt X_m = X_nl - X_ls;

    // Calculate final inductance values in Henrys
    parameter_gt L_ls = X_ls / omega_e;
    parameter_gt L_r_prime = X_r_prime / omega_e;
    acm->Lm = X_m / omega_e;

    // Populate Ls and Lr as self-inductances (leakage + mutual)
    acm->Ls = L_ls + acm->Lm;
    acm->Lr = L_r_prime + acm->Lm;

    // --- Mechanical Parameters ---
    // Note: These electrical tests cannot determine mechanical parameters.
    // They should be initialized to zero or determined by other means.
    acm->inertia = 0.0f;
    acm->damp = 0.0f;

    return true;
}

/** @} */ // end of MC_IM_CONSULTANT group

/*---------------------------------------------------------------------------*/
/* Motor Parameter Calculation Functions                                     */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_IM_CALCULATORS Induction Motor Parameter Calculators
 * @ingroup CTL_MC_COMPONENT
 * @brief A collection of functions to calculate derived parameters from the base motor model.
 * @{
 */

/**
 * @brief Calculates the rotor time constant (Tr).
 * @details Formula: @f( T_r = L_r / R_r @f)
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @return The rotor time constant in seconds (s). Returns 0 if Rr is zero.
 */
GMP_STATIC_INLINE parameter_gt ctl_acm_get_rotor_time_constant_s(const im_dsn_consultant_t* im_dsn)
{
    return (im_dsn->Rr > 1e-9) ? (im_dsn->Lr / im_dsn->Rr) : 0.0;
}

/**
 * @brief Calculates the stator time constant (Ts).
 * @details Formula: @f( T_s = L_s / R_s @f)
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @return The stator time constant in seconds (s). Returns 0 if Rs is zero.
 */
GMP_STATIC_INLINE parameter_gt ctl_acm_get_stator_time_constant_s(const im_dsn_consultant_t* im_dsn)
{
    return (im_dsn->Rs > 1e-9) ? (im_dsn->Ls / im_dsn->Rs) : 0.0;
}

/**
 * @brief Calculates the total leakage coefficient (sigma).
 * @details This is a crucial parameter in field-oriented control.
 * Formula: @f( \sigma = 1 - \frac{L_m^2}{L_s L_r} @f)
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @return The dimensionless leakage coefficient. Returns 1 if Ls or Lr is zero.
 */
GMP_STATIC_INLINE parameter_gt ctl_acm_get_leakage_coefficient(const im_dsn_consultant_t* im_dsn)
{
    parameter_gt LsLr = im_dsn->Ls * im_dsn->Lr;
    return (LsLr > 1e-9) ? (1.0 - (im_dsn->Lm * im_dsn->Lm) / LsLr) : 1.0;
}

/**
 * @brief Estimates the required slip speed for a given torque-producing current (Isq).
 * @details This is used in indirect field-oriented control (IFOC) to calculate the slip.
 * Formula: @f( \omega_{sl} = \frac{R_r}{L_r} \frac{L_m i_{sq}}{\lambda_{dr}} @f)
 * Note: This simplified version assumes rotor flux is aligned with d-axis, so @f( \lambda_r = \lambda_{dr} @f)
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @param[in] i_sq The target q-axis (torque-producing) current in Amperes (A).
 * @param[in] rotor_flux_mag The target rotor flux magnitude in Weber (Wb).
 * @return The estimated slip speed in electrical rad/s.
 */
GMP_STATIC_INLINE parameter_gt ctl_acm_estimate_slip_speed_rads(const im_dsn_consultant_t* im_dsn, parameter_gt i_sq,
                                                                parameter_gt rotor_flux_mag)
{
    if (im_dsn->Lr < 1e-9 || rotor_flux_mag < 1e-9)
        return 0.0;
    return (im_dsn->Rr / im_dsn->Lr) * (im_dsn->Lm * i_sq / rotor_flux_mag);
}

/**
 * @brief Estimates PI controller parameters for the current loop.
 * @details Uses the bandwidth method for tuning. The plant is modeled as a first-order
 * system with transient inductance @f( \sigma L_s @f) and resistance @f( R_s @f).
 * Formulas:
 * @f[ K_p = \alpha_c \sigma L_s @f]
 * @f[ K_i = \alpha_c R_s        @f]
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @param[in] bandwidth_hz The desired closed-loop bandwidth in Hertz (Hz).
 * @param[out] kp Pointer to store the calculated proportional gain.
 * @param[out] ki Pointer to store the calculated integral gain.
 */
GMP_STATIC_INLINE void ctl_acm_tune_current_pi_params(const im_dsn_consultant_t* im_dsn, parameter_gt bandwidth_hz,
                                                      parameter_gt* kp, parameter_gt* ki)
{
    parameter_gt alpha_c = M_2_PI * bandwidth_hz; // Convert Hz to rad/s
    parameter_gt sigma = ctl_acm_get_leakage_coefficient(im_dsn);
    parameter_gt transient_inductance = sigma * im_dsn->Ls;

    *kp = alpha_c * transient_inductance;
    *ki = alpha_c * im_dsn->Rs;
}

/**
 * @brief Estimates PI controller parameters for the speed loop.
 * @details Uses the bandwidth method. Assumes the current loop is significantly faster.
 * The torque constant Kt is calculated internally.
 * Formulas:
 * @f[ K_t = 1.5 \cdot p \cdot \frac{L_m}{L_r} \lambda_r @f]
 * @f[ K_p = \frac{\alpha_s J}{K_t}                      @f]
 * @f[ K_i = \frac{\alpha_s B}{K_t}                      @f]
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @param[in] bandwidth_hz The desired closed-loop bandwidth in Hertz (Hz).
 * @param[in] rotor_flux_mag The target rotor flux magnitude in Weber (Wb) for calculating Kt.
 * @param[out] kp Pointer to store the calculated proportional gain.
 * @param[out] ki Pointer to store the calculated integral gain.
 */
GMP_STATIC_INLINE void ctl_acm_tune_speed_pi_params(const im_dsn_consultant_t* im_dsn, parameter_gt bandwidth_hz,
                                                    parameter_gt rotor_flux_mag, parameter_gt* kp, parameter_gt* ki)
{
    if (im_dsn->Lr < 1e-9)
    {
        *kp = 0.0;
        *ki = 0.0;
        return;
    }

    parameter_gt alpha_s = M_2_PI * bandwidth_hz; // Convert Hz to rad/s

    // Calculate torque constant Kt
    parameter_gt kt = 1.5 * im_dsn->pole_pair * (im_dsn->Lm / im_dsn->Lr) * rotor_flux_mag;

    if (kt < 1e-9)
    {
        *kp = 0.0;
        *ki = 0.0;
        return;
    }

    *kp = (alpha_s * im_dsn->inertia) / kt;
    *ki = (alpha_s * im_dsn->damp) / kt; // Note: This depends on an accurate damping/friction parameter 'B'.
}

/**
 * @brief Estimates the motor slip based on its design parameters and a given load torque.
 * @details This function uses the Kloss equation to provide a good approximation of the slip
 * for a given steady-state load torque. It calculates the breakdown torque and the corresponding
 *
 * critical slip first, then solves for the operational slip.
 * Kloss equation:
 * @f[ \frac{T_e}{T_{max}} = \frac{2}{s/s_k + s_k/s} @f]
 *
 * @param acm Pointer to the induction motor design parameters structure.
 * @param V_phase_rms The operating RMS phase voltage [V].
 * @param freq_Hz The operating synchronous frequency [Hz].
 * @param T_load_Nm The load torque applied to the motor shaft [N·m].
 * @return The estimated slip (a value between 0 and 1). Returns a negative value if the load
 * torque exceeds the motor's breakdown torque.
 */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_estimate_slip(im_dsn_consultant_t* acm, parameter_gt V_phase_rms,
                                                             parameter_gt freq_Hz, parameter_gt T_load_Nm)
{
    if (!acm || freq_Hz <= 0 || V_phase_rms <= 0 || T_load_Nm < 0)
    {
        return -1.0; // Invalid input
    }

    // Step 1: Calculate synchronous speed and reactances at the operating frequency
    parameter_gt omega_e = 2.0f * M_PI * freq_Hz; // Electrical angular velocity [rad/s]

    // Note: Using Ls and Lr directly as stator and rotor reactance components.
    // This is a common simplification where the magnetizing branch is moved to the terminals.
    parameter_gt Xs = omega_e * acm->Ls; // Stator reactance
    parameter_gt Xr = omega_e * acm->Lr; // Rotor reactance

    // Step 2: Calculate the critical slip (slip at maximum torque)
    parameter_gt X_total_sq = (Xs + Xr) * (Xs + Xr);
    parameter_gt Rs_sq = acm->Rs * acm->Rs;
    parameter_gt sk_denominator = (parameter_gt)sqrtf(Rs_sq + X_total_sq);
    if (sk_denominator == 0)
        return -1.0;
    parameter_gt sk = acm->Rr / sk_denominator; // Critical slip (s_k)

    // Step 3: Calculate the maximum (breakdown) torque
    parameter_gt Tmax_denominator = 2.0f * omega_e * (acm->Rs + sk_denominator);
    if (Tmax_denominator == 0)
        return -1.0;
    parameter_gt T_max = (3.0f * V_phase_rms * V_phase_rms) / Tmax_denominator;

    // Step 4: Check if the load torque is achievable
    if (T_load_Nm > T_max)
    {
        return -2.0; // Load torque exceeds breakdown torque, motor would stall.
    }

    if (T_load_Nm == 0)
    {
        return 0.0; // No load, no slip.
    }

    // Step 5: Solve the Kloss equation for slip 's'
    // T_load / T_max = 2 / (s/sk + sk/s)  =>  s^2 - (2 * T_max / T_load * sk) * s + sk^2 = 0
    // This is a quadratic equation: a*s^2 + b*s + c = 0
    parameter_gt a = 1.0f;
    parameter_gt b = -(2.0f * T_max / T_load_Nm) * sk;
    parameter_gt c = sk * sk;

    // Use the quadratic formula to solve for s: s = [-b ± sqrt(b^2 - 4ac)] / 2a
    parameter_gt discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0)
    {
        return -3.0; // Should not happen if T_load <= T_max, but good practice to check
    }

    // We need the smaller root for stable operation (motor region)
    parameter_gt s = (-b - (parameter_gt)sqrtf(discriminant)) / (2.0f * a);

    return s;
}

/** @} */ // end of MC_IM_CALCULATORS group

/*---------------------------------------------------------------------------*/
/* Per-Unit Conversion Functions                                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ACM_PU_CONVERTERS Per-Unit System Converters for ACM
 * @ingroup CTL_MC_COMPONENT
 * @brief A collection of functions to convert PI controller gains to the per-unit system for an ACM.
 * @{
 */

/**
 * @brief Converts current controller PI gains from SI units to per-unit for an ACM.
 * @details The current controller's input is current error and output is voltage.
 * Formulas:
 * @f[ K_{p,pu} = \frac{K_{p,real}}{Z_{base}}                     @f]
 * @f[ K_{i,pu} = \frac{K_{i,real}}{Z_{base} \cdot \omega_{base}} @f]
 * @param[in] pu Pointer to the populated per-unit consultant structure.
 * @param[in] kp_real The real-world proportional gain (V/A).
 * @param[in] ki_real The real-world integral gain (V/(As)).
 * @param[out] kp_pu Pointer to store the per-unit proportional gain.
 * @param[out] ki_pu Pointer to store the per-unit integral gain.
 */
GMP_STATIC_INLINE void ctl_acm_convert_current_pi_to_pu(const ctl_per_unit_consultant_t* pu, parameter_gt kp_real,
                                                        parameter_gt ki_real, parameter_gt* kp_pu, parameter_gt* ki_pu)
{
    if (pu->base_impedence < 1e-9 || pu->base_omega < 1e-9)
    {
        *kp_pu = 0.0;
        *ki_pu = 0.0;
        return;
    }
    *kp_pu = kp_real / pu->base_impedence;
    *ki_pu = ki_real / (pu->base_impedence * pu->base_omega);
}

/**
 * @brief Converts speed controller PI gains from SI units to per-unit for an ACM.
 * @details The speed controller's input is speed error and output is current.
 * Formulas:
 * @f[ K_{p,pu} = \frac{K_{p,real} \cdot \omega_{base}}{I_{base}} @f]
 * @f[ K_{i,pu} = \frac{K_{i,real}}{I_{base}}                     @f]
 * @param[in] pu Pointer to the populated per-unit consultant structure.
 * @param[in] kp_real The real-world proportional gain (A/(rad/s)).
 * @param[in] ki_real The real-world integral gain (A/rad).
 * @param[out] kp_pu Pointer to store the per-unit proportional gain.
 * @param[out] ki_pu Pointer to store the per-unit integral gain.
 */
GMP_STATIC_INLINE void ctl_acm_convert_speed_pi_to_pu(const ctl_per_unit_consultant_t* pu, parameter_gt kp_real,
                                                      parameter_gt ki_real, parameter_gt* kp_pu, parameter_gt* ki_pu)
{
    if (pu->base_current < 1e-9)
    {
        *kp_pu = 0.0;
        *ki_pu = 0.0;
        return;
    }
    *kp_pu = (kp_real * pu->base_omega) / pu->base_current;
    *ki_pu = ki_real / pu->base_current;
}

/**
 * @}
 */ // end of MC_ACM_PU_CONVERTERS group

/*---------------------------------------------------------------------------*/
/* Asynchronous Motor Design Parameter Conversion Functions                  */
/*---------------------------------------------------------------------------*/

/**
 * @name Asynchronous Motor Parameter Converters
 * @ingroup CTL_MC_COMPONENT
 * @brief A set of inline functions to convert physical motor design parameters to per-unit.
 * @{
 */

//
// Electrical Parameter Conversions
//

/** @brief Converts the stator resistance [ohm] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Rs_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // R_pu = R_phy / Z_base
    return acm->Rs / pu->base_impedence;
}

/** @brief Converts the stator inductance [H] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Ls_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // L_pu = L_phy / L_base
    return acm->Ls / pu->base_inductance;
}

/** @brief Converts the rotor resistance (referred) [ohm] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Rr_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // R_pu = R_phy / Z_base
    return acm->Rr / pu->base_impedence;
}

/** @brief Converts the rotor inductance (referred) [H] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Lr_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // L_pu = L_phy / L_base
    return acm->Lr / pu->base_inductance;
}

/** @brief Converts the mutual inductance [H] to its per-unit equivalent. */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Lm_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // L_pu = L_phy / L_base
    return acm->Lm / pu->base_inductance;
}

//
// Mechanical Parameter Conversions
//

/** * @brief Converts the moment of inertia (J) [kg m^2] to its per-unit equivalent. 
 * @details The per-unit inertia is derived from the mechanical equation T = J * domega/dt.
 * In per-unit, T_pu = J_pu * d(omega_pu)/d(t_pu).
 * Base time is defined as t_base = 1 / omega_e_base.
 * This leads to the base inertia J_base = T_base / (omega_m_base * omega_e_base).
 */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Inertia_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // Calculate the base inertia on-the-fly
    parameter_gt base_inertia = pu->base_torque / (pu->base_speed * pu->base_omega);
    return acm->inertia / base_inertia;
}

/** * @brief Converts the damping factor (B) [Nms] to its per-unit equivalent. 
 * @details The per-unit damping is derived from the friction torque equation T_damp = B * ω_m.
 * In per-unit, T_damp_pu = B_pu * ω_m_pu.
 * This leads to the base damping B_base = T_base / omega_m_base.
 */
GMP_STATIC_INLINE parameter_gt ctl_consult_acm_Damp_pu(ctl_per_unit_consultant_t* pu, im_dsn_consultant_t* acm)
{
    // Calculate the base damping factor on-the-fly
    parameter_gt base_damp = pu->base_torque / pu->base_speed;
    return acm->damp / base_damp;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_CONSULTANT_H_
