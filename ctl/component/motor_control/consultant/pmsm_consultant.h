/**
 * @file pmsm_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines data structures and utility functions for PMSM motor parameters.
 * @details This file provides "consultant" structures for key nameplate and design
 * parameters of a PMSM, along with functions to calculate derived constants,
 * estimate controller tuning parameters, and convert parameters to the per-unit system.
 * @version 0.3
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_PMSM_CONSULTANT_H_
#define _FILE_PMSM_CONSULTANT_H_

#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h> // Include for per-unit system base values
#include <ctl/component/motor_control/consultant/motor_unit_calculator.h> // Include for physical constants and unit conversions

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Forward declaration of structures
struct _tag_pmsm_nameplate_consultant_t;
struct _tag_pmsm_dsn_consultant_t;

/*---------------------------------------------------------------------------*/
/* PMSM Nameplate Parameter Definitions                                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_NAMEPLATE PMSM Nameplate Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief Holds the rated (nameplate) parameters of a PMSM.
 * @{
 */

/**
 * @brief Data structure for holding PMSM nameplate parameters.
 * @details These are the values typically found on the motor's label.
 * //tex:
 * Motor speed: $n_N$, motor power: $P_N$, Motor Voltage: $V_N$,
 * Motor Current: $I_N$, motor frequency: $f_N$, motor torque: $T_N$.
 */
typedef struct _tag_pmsm_nameplate_consultant_t
{
    uint16_t pole_pairs;             /**< @brief Number of motor pole pairs. */
    parameter_gt rated_output_power; /**< @brief Rated output power in Watts (W). */
    parameter_gt rated_voltage;      /**< @brief Rated line-to-line voltage in Volts RMS. */
    parameter_gt rated_current;      /**< @brief Rated line current in Amps RMS. */
    parameter_gt power_factor;       /**< @brief Rated power factor (dimensionless). */
    parameter_gt rated_freq;         /**< @brief Rated electrical frequency in Hertz (Hz). */
    parameter_gt rated_speed_rpm;    /**< @brief Rated mechanical speed in RPM. */
    parameter_gt rated_torque;       /**< @brief Rated torque in Newton-meters (N·m). */
    parameter_gt eta;                /**< @brief Rated efficiency (dimensionless). */
} ctl_pmsm_nameplate_consultant_t;

/**
 * @brief Initializes the PMSM nameplate consultant structure.
 * @param[out] np Pointer to the nameplate consultant structure.
 */
void ctl_init_pmsm_nameplate_consultant(ctl_pmsm_nameplate_consultant_t* np);

/**
 * @brief Sets up the PMSM nameplate consultant structure with specified values.
 * @param[out] np Pointer to the nameplate consultant structure.
 * @param[in] rated_output_power Rated power in Watts (W).
 * @param[in] rated_voltage Rated line-to-line voltage (V RMS).
 * @param[in] rated_current Rated line current (A RMS).
 * @param[in] rated_freq Rated frequency (Hz).
 * @param[in] rated_speed Rated speed (RPM).
 * @param[in] rated_torque Rated torque (N·m).
 * @param[in] power_factor Rated power factor.
 * @param[in] efficiency Rated efficiency.
 */
void ctl_setup_pmsm_nameplate_consultant(ctl_pmsm_nameplate_consultant_t* np, parameter_gt rated_output_power,
                                         parameter_gt rated_voltage, parameter_gt rated_current,
                                         parameter_gt rated_freq, parameter_gt rated_speed, parameter_gt rated_torque,
                                         parameter_gt power_factor, parameter_gt efficiency);

/** @} */ // end of MC_PMSM_NAMEPLATE group

/*---------------------------------------------------------------------------*/
/* PMSM Design Parameter Definitions                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_DESIGN PMSM Design Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief Holds the design (model) parameters of a PMSM.
 * @{
 */

/**
 * @brief Data structure for holding PMSM design (model) parameters.
 * @details These parameters are used in the mathematical model of the motor for FOC.
 * //tex:
 * Pole pairs: $p$, Stator Resistance: $R_s$, d-axis Inductance: $L_d$,
 * q-axis Inductance: $L_q$, PM Flux: $\psi_f$, Inertia: $J$, Damping: $B$.
 */
typedef struct _tag_pmsm_dsn_consultant
{
    uint16_t pole_pair;   /**< @brief Number of motor pole pairs. */
    parameter_gt Rs;      /**< @brief Stator resistance per phase in Ohms (Ω). */
    parameter_gt Ld;      /**< @brief D-axis inductance in Henrys (H). */
    parameter_gt Lq;      /**< @brief Q-axis inductance in Henrys (H). */
    parameter_gt flux;    /**< @brief Permanent magnet flux linkage in Weber (Wb). */
    parameter_gt inertia; /**< @brief Moment of inertia (J) in kg·m². */
    parameter_gt damp;    /**< @brief Damping factor (B) in N·m·s. */
} ctl_pmsm_dsn_consultant_t;

/**
 * @brief Initializes the PMSM design consultant structure.
 * @param[out] pmsm_dsn Pointer to the design consultant structure.
 */
void ctl_init_pmsm_dsn_consultant(ctl_pmsm_dsn_consultant_t* pmsm_dsn);

/**
 * @brief Sets up the PMSM design consultant structure with specified values.
 * @param[out] pmsm_dsn Pointer to the design consultant structure.
 * @param[in] pole_pair Number of motor pole pairs.
 * @param[in] Rs Stator resistance (Ω).
 * @param[in] Ld D-axis inductance (H).
 * @param[in] Lq Q-axis inductance (H).
 * @param[in] flux PM flux linkage (Wb).
 * @param[in] inertia Moment of inertia (kg·m²).
 * @param[in] damp Damping factor (N·m·s).
 */
void ctl_setup_pmsm_dsn_consultant(ctl_pmsm_dsn_consultant_t* pmsm_dsn, uint16_t pole_pair, parameter_gt Rs,
                                   parameter_gt Ld, parameter_gt Lq, parameter_gt flux, parameter_gt inertia,
                                   parameter_gt damp);

/** @} */ // end of MC_PMSM_DESIGN group

/*---------------------------------------------------------------------------*/
/* PMSM Parameter Calculation Functions                                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_CALCULATORS PMSM Parameter Calculators
 * @ingroup CTL_MC_COMPONENT
 * @brief A collection of functions to calculate derived parameters and tune controllers.
 * @{
 */

/**
 * @brief Calculates the PMSM torque constant (Kt).
 * @details Formula: //tex: K_t = 1.5 \cdot p \cdot \psi_f
 * @param[in] pmsm_dsn Pointer to the populated motor design consultant structure.
 * @return The torque constant in N·m / Amp_peak.
 */
GMP_STATIC_INLINE parameter_gt ctl_pmsm_get_Kt(const ctl_pmsm_dsn_consultant_t* pmsm_dsn)
{
    return (1.5f * pmsm_dsn->pole_pair * pmsm_dsn->flux);
}

/**
 * @brief Calculates the PMSM back-EMF constant (Ke).
 * @details This is the peak line-to-neutral voltage per electrical rad/s.
 * Formula: //tex: K_e = p \cdot \psi_f
 * @param[in] pmsm_dsn Pointer to the populated motor design consultant structure.
 * @return The back-EMF constant in V_peak / (rad/s_elec).
 */
GMP_STATIC_INLINE parameter_gt ctl_pmsm_get_Ke_Vpeak_per_rads(const ctl_pmsm_dsn_consultant_t* pmsm_dsn)
{
    return (pmsm_dsn->pole_pair * pmsm_dsn->flux);
}

/**
 * @brief Calculates the PMSM back-EMF constant (Ke) in V_RMS / kRPM.
 * @details This is a common unit found on motor datasheets.
 * Formula: //tex: K_e [V_{RMS,L-L}/kRPM] = p \cdot \psi_f \cdot \frac{\pi \sqrt{3}}{3}
 * @param[in] pmsm_dsn Pointer to the populated motor design consultant structure.
 * @return The back-EMF constant in V_RMS_LL / kRPM.
 */
GMP_STATIC_INLINE parameter_gt ctl_pmsm_get_Ke_Vrms_per_krpm(const ctl_pmsm_dsn_consultant_t* pmsm_dsn)
{
    // Ke(Vp/rads) * (rads/s / RPM) * (RPM / kRPM) * Vrms/Vp
    // Ke(Vp/rads) * (2*pi/60) * 1000 * (1/sqrt(2)) * sqrt(3) for L-L
    return ctl_pmsm_get_Ke_Vpeak_per_rads(pmsm_dsn) * (CTL_PARAM_CONST_PI * 100.0f / 3.0f) *
           CTL_PARAM_CONST_SQRT3_OVER_SQRT2;
}

/**
 * @brief Estimates PI controller parameters for the d-q axis current loops.
 * @details Uses the bandwidth method (pole placement) for tuning.
 * The plant is modeled as a first-order system.
 * Formulas:
 * //tex: K_p = \alpha_c L, \quad K_i = \alpha_c R_s
 * where //tex: \alpha_c is the desired bandwidth.
 * @param[in] pmsm_dsn Pointer to the populated motor design consultant structure.
 * @param[in] bandwidth_hz The desired closed-loop bandwidth in Hertz (Hz).
 * @param[out] kp_d Pointer to store the d-axis proportional gain.
 * @param[out] ki_d Pointer to store the d-axis integral gain.
 * @param[out] kp_q Pointer to store the q-axis proportional gain.
 * @param[out] ki_q Pointer to store the q-axis integral gain.
 */
GMP_STATIC_INLINE void ctl_pmsm_tune_current_pi_params(const ctl_pmsm_dsn_consultant_t* pmsm_dsn, parameter_gt bandwidth_hz,
                                                   parameter_gt* kp_d, parameter_gt* ki_d, parameter_gt* kp_q,
                                                   parameter_gt* ki_q)
{
    parameter_gt alpha_c = CTL_PARAM_CONST_2PI * bandwidth_hz; // Convert Hz to rad/s

    *kp_d = alpha_c * pmsm_dsn->Ld;
    *ki_d = alpha_c * pmsm_dsn->Rs;

    *kp_q = alpha_c * pmsm_dsn->Lq;
    *ki_q = alpha_c * pmsm_dsn->Rs;
}

/**
 * @brief Estimates PI controller parameters for the speed loop.
 * @details Uses the bandwidth method. Assumes the current loop is significantly faster.
 * Formulas:
 * //tex: K_p = \frac{\alpha_s J}{K_t}, \quad K_i = \frac{\alpha_s B}{K_t}
 * where //tex: \alpha_s is the desired bandwidth.
 * @param[in] pmsm_dsn Pointer to the populated motor design consultant structure.
 * @param[in] bandwidth_hz The desired closed-loop bandwidth in Hertz (Hz).
 * @param[out] kp Pointer to store the calculated proportional gain.
 * @param[out] ki Pointer to store the calculated integral gain.
 */
GMP_STATIC_INLINE void ctl_pmsm_tune_speed_pi_params(const ctl_pmsm_dsn_consultant_t* pmsm_dsn, parameter_gt bandwidth_hz,
                                                 parameter_gt* kp, parameter_gt* ki)
{
    parameter_gt kt = ctl_pmsm_get_Kt(pmsm_dsn);
    if (kt < 1e-9)
    {
        *kp = 0.0;
        *ki = 0.0;
        return;
    }

    parameter_gt alpha_s = CTL_PARAM_CONST_2PI * bandwidth_hz; // Convert Hz to rad/s

    *kp = (alpha_s * pmsm_dsn->inertia) / kt;
    // Note: Ki calculation depends on an accurate damping/friction parameter 'B'.
    // If B is unknown, Ki might be set to a small value or tuned experimentally.
    *ki = (alpha_s * pmsm_dsn->damp) / kt;
}

/** @} */ // end of MC_PMSM_CALCULATORS group

/*---------------------------------------------------------------------------*/
/* Per-Unit Conversion Functions                                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_PU_CONVERTERS Per-Unit System Converters
 * @ingroup CTL_MC_COMPONENT
 * @brief A collection of functions to convert PI controller gains to the per-unit system.
 * @{
 */

/**
 * @brief Converts current controller PI gains from SI units to per-unit.
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
GMP_STATIC_INLINE void ctl_pmsm_convert_current_pi_to_pu(const ctl_per_unit_consultant_t* pu, parameter_gt kp_real,
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
 * @brief Converts speed controller PI gains from SI units to per-unit.
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
GMP_STATIC_INLINE void ctl_pmsm_convert_speed_pi_to_pu(const ctl_per_unit_consultant_t* pu, parameter_gt kp_real,
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

/** @} */ // end of MC_PMSM_PU_CONVERTERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CONSULTANT_H_
