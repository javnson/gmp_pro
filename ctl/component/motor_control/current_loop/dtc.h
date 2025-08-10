/**
 * @file pmsm_dtc.h
 * @brief Implements a classic Direct Torque Control (DTC) scheme for PMSM.
 * @details This module provides a complete DTC controller. Unlike FOC, DTC does
 * not use inner current loops or a PWM modulator. Instead, it estimates the
 * stator flux and electromagnetic torque and uses two hysteresis controllers
 * to keep them within desired bands. Based on the outputs of the hysteresis
 * controllers and the current sector of the stator flux vector, an optimal
 * voltage vector is selected from a switching table and applied directly to
 * the motor for the entire control period.
 *
 * @version 1.0
 * @date 2025-08-06
 *
 * //tex:
 * // The controller is based on the following estimations:
 * // 1. Stator Flux Estimation (Voltage Model):
 * //    \vec{\psi_s} = \int (\vec{v_s} - R_s \vec{i_s}) dt
 * // 2. Electromagnetic Torque Estimation:
 * //    T_e = \frac{3}{2} p (\psi_{s\alpha} i_{s\beta} - \psi_{s\beta} i_{s\alpha})
 *
 */

#ifndef _FILE_PMSM_DTC_H_
#define _FILE_PMSM_DTC_H_

#include <ctl/component/intrinsic/basic/hysteresis_controller.h> // Dependency for flux and torque control

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Direct Torque Controller (DTC)                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup DTC_CONTROLLER Direct Torque Controller (DTC)
 * @brief A high-performance controller based on direct flux and torque regulation.
 * @{
 */

//================================================================================
// Type Defines, Enums & Data
//================================================================================

/**
 * @brief The optimal voltage vector switching table for a 2-level DTC scheme.
 * @details Rows correspond to the flux vector sector (1-6).
 * Columns correspond to the combined hysteresis outputs [Flux, Torque]:
 * [0,0]->[Dec,Dec], [0,1]->[Dec,Inc], [1,0]->[Inc,Dec], [1,1]->[Inc,Inc]
 */
extern uint8_t DTC_SWITCH_TABLE[6][4];
;

/**
 * @brief Table to convert voltage vector index (0-7) to alpha-beta voltages.
 * @details Assumes Vdc is the DC bus voltage. The values are scaled by 2/3.
 */
extern ctrl_gt V_ALPHA_BETA_TABLE[8][2];

/**
 * @brief Initialization parameters for the DTC module.
 */
typedef struct
{
    // --- Motor & System Parameters ---
    parameter_gt Rs;     ///< Stator Resistance (Ohm).
    parameter_gt f_ctrl; ///< Controller execution frequency (Hz).
    uint16_t pole_pairs; ///< Number of motor pole pairs.

    // --- Controller Parameters ---
    ctrl_gt flux_hyst_width;   ///< The half-width of the flux hysteresis band.
    ctrl_gt torque_hyst_width; ///< The half-width of the torque hysteresis band.

} ctl_dtc_init_t;

/**
 * @brief Main structure for the DTC controller.
 */
typedef struct
{
    // --- Outputs ---
    uint8_t voltage_vector_index; ///< The selected output voltage vector (0-7).

    // --- Estimated Variables ---
    ctl_vector2_t stator_flux; ///< Estimated stator flux vector [psi_alpha, psi_beta]^T.
    ctrl_gt flux_mag_est;      ///< Magnitude of the estimated stator flux.
    ctrl_gt torque_est;        ///< Estimated electromagnetic torque.
    uint8_t flux_sector;       ///< Current sector of the stator flux vector (1-6).

    // --- Hysteresis Controllers ---
    ctl_hysteresis_controller_t flux_hcc;   ///< Hysteresis controller for flux.
    ctl_hysteresis_controller_t torque_hcc; ///< Hysteresis controller for torque.

    // --- Pre-calculated Parameters ---
    ctrl_gt rs;         ///< Stator Resistance.
    ctrl_gt ts;         ///< Control period (1 / f_ctrl).
    ctrl_gt pole_pairs; ///< Number of pole pairs.

} ctl_dtc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the DTC controller.
 * @param[out] dtc  Pointer to the DTC structure.
 * @param[in]  init Pointer to the initialization parameters structure.
 */
void ctl_init_dtc(ctl_dtc_controller_t* dtc, const ctl_dtc_init_t* init);

/**
 * @brief Sets the reference (target) values for the flux and torque controllers.
 * @param[out] dtc Pointer to the DTC structure.
 * @param[in]  flux_ref The target stator flux magnitude (Wb).
 * @param[in]  torque_ref The target electromagnetic torque (Nm).
 */
GMP_STATIC_INLINE void ctl_set_dtc_references(ctl_dtc_controller_t* dtc, ctrl_gt flux_ref, ctrl_gt torque_ref)
{
    ctl_set_hysteresis_target(&dtc->flux_hcc, flux_ref);
    ctl_set_hysteresis_target(&dtc->torque_hcc, torque_ref);
}

/**
 * @brief Executes one step of the Direct Torque Control algorithm.
 * @param[out] dtc      Pointer to the DTC structure.
 * @param[in]  i_alpha  The measured alpha-axis current.
 * @param[in]  i_beta   The measured beta-axis current.
 * @param[in]  udc      The measured DC bus voltage.
 */
GMP_STATIC_INLINE void ctl_step_dtc(ctl_dtc_controller_t* dtc, ctrl_gt i_alpha, ctrl_gt i_beta, ctrl_gt udc)
{
    // 1. Determine the applied alpha-beta voltages from the PREVIOUS cycle's vector.
    ctrl_gt u_alpha = V_ALPHA_BETA_TABLE[dtc->voltage_vector_index][0] * udc;
    ctrl_gt u_beta = V_ALPHA_BETA_TABLE[dtc->voltage_vector_index][1] * udc;

    // 2. Estimate Stator Flux (Voltage Model Integration).
    dtc->stator_flux.dat[0] += dtc->ts * (u_alpha - dtc->rs * i_alpha);
    dtc->stator_flux.dat[1] += dtc->ts * (u_beta - dtc->rs * i_beta);

    // 3. Calculate Flux Magnitude and Electromagnetic Torque.
    dtc->flux_mag_est = ctl_sqrt(ctl_mul(dtc->stator_flux.dat[0], dtc->stator_flux.dat[0]) +
                                 ctl_mul(dtc->stator_flux.dat[1], dtc->stator_flux.dat[1]));
    dtc->torque_est = 1.5f * dtc->pole_pairs * (dtc->stator_flux.dat[0] * i_beta - dtc->stator_flux.dat[1] * i_alpha);

    // 4. Determine the Flux Vector Sector (1-6).
    ctrl_gt angle = ctl_atan2(dtc->stator_flux.dat[1], dtc->stator_flux.dat[0]); // Angle in radians
    if (angle < 0)
        angle += 2.0f * CTL_CTRL_CONST_PI;
    dtc->flux_sector = (uint8_t)(angle / (CTL_CTRL_CONST_PI / 3.0f)) + 1;
    if (dtc->flux_sector > 6)
        dtc->flux_sector = 6; // Clamp to sector 6

    // 5. Run Hysteresis Controllers.
    fast_gt flux_status = ctl_step_hysteresis_controller(&dtc->flux_hcc, dtc->flux_mag_est);
    fast_gt torque_status = ctl_step_hysteresis_controller(&dtc->torque_hcc, dtc->torque_est);

    // 6. Look up the Optimal Voltage Vector from the Switching Table.
    fast_gt table_col = (flux_status << 1) | torque_status;
    dtc->voltage_vector_index = DTC_SWITCH_TABLE[dtc->flux_sector - 1][table_col];
}

/**
 * @brief Gets the selected voltage vector index.
 * @param[in] dtc Pointer to the DTC structure.
 * @return The index of the selected voltage vector (0-7).
 */
GMP_STATIC_INLINE uint8_t ctl_get_dtc_vector_index(const ctl_dtc_controller_t* dtc)
{
    return dtc->voltage_vector_index;
}

/** @} */ // end of DTC_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_DTC_H_
