/**
 * @file consultant_pmsm.h
 * @brief Implements the Permanent Magnet Synchronous Motor (PMSM) physical model.
 *
 * @version 1.0
 * @date 2024-10-27
 */

#ifndef _FILE_CONSULTANT_PMSM_H_
#define _FILE_CONSULTANT_PMSM_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Consultant: PMSM Physical Model                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CONSULTANT_PMSM PMSM Model Consultant
 * @brief Manages the electrical and magnetic physical parameters of a PMSM.
 * @details Acts as a standardized parameter provider for inner loop controllers 
 * (FOC, MPC, DTC). Calculates intrinsic derived parameters such as characteristic 
 * current and saliency ratio. Purely physical (no Per-Unit math here).
 * @{
 */

/**
 * @brief Standardized structure for PMSM physical parameters.
 */
typedef struct _tag_consultant_pmsm
{
    // --- Nameplate & Basic Electrical Parameters ---
    uint32_t pole_pairs;       //!< Number of pole pairs (p).
    parameter_gt Rs;           //!< Stator phase resistance (Ohm).
    parameter_gt Ld;           //!< D-axis synchronous inductance (H).
    parameter_gt Lq;           //!< Q-axis synchronous inductance (H).
    parameter_gt flux_linkage; //!< Permanent magnet flux linkage RMS (Weber).

    // --- Derived Intrinsic Properties (Calculated automatically) ---
    parameter_gt saliency_ratio; //!< Lq / Ld (1.0 for SPM, >1.0 for IPM).
    parameter_gt char_current;   //!< Characteristic current: flux / (Lq - Ld) (A). Infinite for SPM.

    fast_gt is_ipm; //!< Flag: 1 if Interior PM (Lq > Ld), 0 if Surface PM (Lq == Ld).

} ctl_consultant_pmsm_t;

//================================================================================
// Function Prototypes & Inline APIs
//================================================================================

/**
 * @brief Initializes the PMSM model and validates all physical parameters.
 * @param[out] motor Pointer to the PMSM consultant instance.
 * @param[in]  pp    Pole pairs.
 * @param[in]  rs    Stator resistance (Ohm).
 * @param[in]  ld    D-axis inductance (H).
 * @param[in]  lq    Q-axis inductance (H).
 * @param[in]  flux  PM flux linkage (Wb).
 */
void ctl_consultant_pmsm_init(ctl_consultant_pmsm_t* motor, uint32_t pp, parameter_gt rs, parameter_gt ld,
                              parameter_gt lq, parameter_gt flux);

/**
 * @brief Calculates the actual electromagnetic torque based on physical currents.
 * @details Te = 1.5 * p * (Flux * Iq + (Ld - Lq) * Id * Iq)
 * @param[in] motor Pointer to the PMSM consultant instance.
 * @param[in] id    D-axis current (A).
 * @param[in] iq    Q-axis current (A).
 * @return parameter_gt Electromagnetic torque (Nm).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_calc_torque(const ctl_consultant_pmsm_t* motor, parameter_gt id,
                                                               parameter_gt iq)
{
    parameter_gt reluctance_term = (motor->Ld - motor->Lq) * id * iq;
    parameter_gt magnet_term = motor->flux_linkage * iq;

    return 1.5f * (parameter_gt)motor->pole_pairs * (magnet_term + reluctance_term);
}

/**
 * @brief Calculates the magnitude of the stator flux linkage vector.
 * @details |Psi_s| = sqrt( (Flux + Ld*Id)^2 + (Lq*Iq)^2 )
 * Useful for voltage limit predictions and DTC monitoring.
 * @param[in] motor Pointer to the PMSM consultant instance.
 * @param[in] id    D-axis current (A).
 * @param[in] iq    Q-axis current (A).
 * @return parameter_gt Stator flux magnitude (Wb).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_calc_flux_mag(const ctl_consultant_pmsm_t* motor, parameter_gt id,
                                                                 parameter_gt iq)
{
    parameter_gt psi_d = motor->flux_linkage + motor->Ld * id;
    parameter_gt psi_q = motor->Lq * iq;

    // Note: This is in physical float space, so standard sqrtf is used
    return sqrtf(psi_d * psi_d + psi_q * psi_q);
}

//================================================================================
// 1. Time Constants (电气时间常数)
//================================================================================

/**
 * @brief Calculates the D-axis electrical time constant.
 * @return parameter_gt Tau_d in seconds.
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_tau_d(const ctl_consultant_pmsm_t* motor)
{
    return motor->Ld / motor->Rs;
}

/**
 * @brief Calculates the Q-axis electrical time constant.
 * @return parameter_gt Tau_q in seconds.
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_tau_q(const ctl_consultant_pmsm_t* motor)
{
    return motor->Lq / motor->Rs;
}

//================================================================================
// 2. Torque Constant (力矩常数)
//================================================================================

/**
 * @brief Calculates the nominal Torque Constant (Kt) in SI units (Nm / A_peak).
 * @details Represents the torque produced per unit of peak Q-axis current, 
 * assuming Id = 0 (Standard for SPM, or nominal for IPM without reluctance torque).
 * Formula: Kt = 1.5 * p * Flux
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_calc_Kt_SI(const ctl_consultant_pmsm_t* motor)
{
    return 1.5f * (parameter_gt)motor->pole_pairs * motor->flux_linkage;
}

//================================================================================
// 3. Back-EMF Constant (反电动势常数 - 正向计算)
//================================================================================

/**
 * @brief Calculates the Back-EMF Constant (Ke) in strict SI units.
 * @details Unit: V_peak(phase-neutral) / (rad/s_mech).
 * Formula: Ke_SI = p * Flux
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_calc_Ke_SI(const ctl_consultant_pmsm_t* motor)
{
    return (parameter_gt)motor->pole_pairs * motor->flux_linkage;
}

/**
 * @brief Calculates the Back-EMF Constant (Ke) in common engineering units.
 * @details Unit: V_rms(Line-to-Line) / krpm.
 * Formula: Ke_Vrms_krpm = sqrt(3/2) * p * Flux * (1000 * 2PI / 60)
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_calc_Ke_Vrms_krpm(const ctl_consultant_pmsm_t* motor)
{
    // sqrt(3)/sqrt(2) = 1.22474487f
    // (1000 * 2*PI) / 60 = 104.719755f
    return 1.22474487f * (parameter_gt)motor->pole_pairs * motor->flux_linkage * 104.719755f;
}

//================================================================================
// 4. Parameter Translators (反向推导工具：从铭牌到物理量)
//================================================================================

/**
 * @brief Estimates Permanent Magnet Flux Linkage from a Datasheet Ke value.
 * @details Solves for Flux (Weber) given Ke in V_rms(Line-to-Line) / krpm.
 * Useful when populating the init structure from a motor datasheet.
 * @param[in] pole_pairs The number of pole pairs (p).
 * @param[in] ke_vrms_krpm The back-EMF constant from the datasheet.
 * @return parameter_gt Estimated Flux Linkage (Weber).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_flux_from_Ke(uint32_t pole_pairs, parameter_gt ke_vrms_krpm)
{
    // Protect against division by zero if pole_pairs is accidentally 0
    parameter_gt pp_safe = (pole_pairs > 0) ? (parameter_gt)pole_pairs : 1.0f;

    // Flux = Ke / ( sqrt(3/2) * p * 104.719755 )
    return ke_vrms_krpm / (1.22474487f * pp_safe * 104.719755f);
}

/**
 * @brief Estimates Permanent Magnet Flux Linkage from a Datasheet Kt value.
 * @details Solves for Flux (Weber) given Kt in Nm / A_rms.
 * @param[in] pole_pairs The number of pole pairs (p).
 * @param[in] kt_nm_arms The torque constant from the datasheet.
 * @return parameter_gt Estimated Flux Linkage (Weber).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_pmsm_flux_from_Kt_Arms(uint32_t pole_pairs, parameter_gt kt_nm_arms)
{
    parameter_gt pp_safe = (pole_pairs > 0) ? (parameter_gt)pole_pairs : 1.0f;

    // T = 1.5 * p * Flux * Iq_peak
    // Iq_peak = sqrt(2) * Iq_rms
    // Kt_rms = T / Iq_rms = 1.5 * sqrt(2) * p * Flux
    // Flux = Kt_rms / (2.12132034f * p)
    return kt_nm_arms / (2.12132034f * pp_safe);
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONSULTANT_PMSM_H_
