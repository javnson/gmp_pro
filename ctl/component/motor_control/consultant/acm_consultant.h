/**
 * @file acm_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a data structure and utility functions for AC induction motor parameters.
 * @details This file provides a "consultant" structure for key electrical and mechanical
 * parameters of an AC induction motor, along with functions to calculate derived
 * constants, estimate controller tuning parameters, and convert parameters to the per-unit system.
 * @version 0.3
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_IM_CONSULTANT_H_
#define _FILE_IM_CONSULTANT_H_

#include "motor_per_unit_consultant.h" // Include for per-unit system base values
#include "motor_unit_calculator.h"     // Include for physical constants and unit conversions

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Forward declaration of the main structure
struct _tag_im_dsn_consultant;

/*---------------------------------------------------------------------------*/
/* Induction Motor Design Consultant Structure                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_IM_CONSULTANT Induction Motor Design Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief A structure to hold all design parameters for an induction motor.
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
    parameter_gt Rs;    /**< @brief Stator resistance in Ohms (Ω). */
    parameter_gt Ls;    /**< @brief Stator inductance in Henrys (H). */
    parameter_gt Rr;    /**< @brief Rotor resistance in Ohms (Ω), referred to the stator. */
    parameter_gt Lr;    /**< @brief Rotor inductance in Henrys (H), referred to the stator. */
    parameter_gt Lm;    /**< @brief Mutual inductance in Henrys (H). */

    // Mechanical Parameters
    parameter_gt inertia; /**< @brief Moment of inertia (J) in kg·m². */
    parameter_gt damp;    /**< @brief Damping factor (B) or friction coefficient in N·m·s. */

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
 * @details Formula: //tex: T_r = L_r / R_r
 * @param[in] im_dsn Pointer to the populated motor consultant structure.
 * @return The rotor time constant in seconds (s). Returns 0 if Rr is zero.
 */
GMP_STATIC_INLINE parameter_gt ctl_acm_get_rotor_time_constant_s(const im_dsn_consultant_t* im_dsn)
{
    return (im_dsn->Rr > 1e-9) ? (im_dsn->Lr / im_dsn->Rr) : 0.0;
}

/**
 * @brief Calculates the stator time constant (Ts).
 * @details Formula: //tex: T_s = L_s / R_s
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
 * Formula: //tex: \sigma = 1 - \frac{L_m^2}{L_s L_r}
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
 * Formula: //tex: \omega_{sl} = \frac{R_r}{L_r} \frac{L_m i_{sq}}{\lambda_{dr}}
 * Note: This simplified version assumes rotor flux is aligned with d-axis, so //tex:\lambda_r = \lambda_{dr}
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
 * system with transient inductance //tex:\sigma L_s and resistance //tex:R_s.
 * Formulas:
 * //tex: K_p = \alpha_c \sigma L_s
 * //tex: K_i = \alpha_c R_s
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
 * //tex: K_t = 1.5 \cdot p \cdot \frac{L_m}{L_r} \lambda_r
 * //tex: K_p = \frac{\alpha_s J}{K_t}
 * //tex: K_i = \frac{\alpha_s B}{K_t}
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
 * //tex: K_{p,pu} = \frac{K_{p,real}}{Z_{base}}
 * //tex: K_{i,pu} = \frac{K_{i,real}}{Z_{base} \cdot \omega_{base}}
 * @param[in] pu Pointer to the populated per-unit consultant structure.
 * @param[in] kp_real The real-world proportional gain (V/A).
 * @param[in] ki_real The real-world integral gain (V/(A·s)).
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
 * //tex: K_{p,pu} = \frac{K_{p,real} \cdot \omega_{base}}{I_{base}}
 * //tex: K_{i,pu} = \frac{K_{i,real}}{I_{base}}
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

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_CONSULTANT_H_
