/**
 * @file three_phase_additional.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset three-phase DC/AC inverter additional controller.
 * @version 1.0
 * @date 2026-01-26
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_TOPOLOGY_GFL_INV_H_API Three-Phase GFL Inverter Topology API (Header)
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive three-phase inverter, including harmonic compensation, droop control,
 * and multiple operating modes.
 */

#ifndef _FILE_THREE_PHASE_ADDITIONAL_
#define _FILE_THREE_PHASE_ADDITIONAL_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#include <ctl/component/intrinsic/discrete/biquad_filter.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

//////////////////////////////////////////////////////////////////////////
// DQ Harmonic controller
//

/**
 * @brief DQ Harmonic Compensation Module (HCM).
 * @details 
 * This module operates in the Synchronous Reference Frame (DQ).
 * Due to frequency aliasing in the SRF:
 * - The **6th order** resonator suppresses physical **-5th (negative seq)** and **+7th (positive seq)** harmonics.
 * - The **12th order** resonator suppresses physical **-11th (negative seq)** and **+13th (positive seq)** harmonics.
 * * **Usage Strategy:**
 * The output `dq_out` can be injected in two locations:
 * 1. **Current Loop Input**: Add to current reference to clean up current waveform.
 * 2. **Voltage Output (Feedforward)**: Add directly to final voltage command for high-bandwidth voltage cleaning (requires care!).
 */
typedef struct _tag_inv_dq_hcm
{
    //
    // --- Input Ports (Pointers) ---
    //
    ctl_vector2_t* dq_meas; //!< RO: Pointer to measured DQ quantity (Current or Voltage).
    ctl_vector2_t* dq_set;  //!< RO: Pointer to target DQ setpoint (Usually points to a Zero vector).

    //
    // --- Output Ports ---
    //
    ctl_vector2_t dq_out; //!< RO: Compensated output vector.

    //
    // --- Internal Controllers ---
    //

    // D-Axis Resonators
    qr_ctrl_t qr_d_6th;  //!< D-axis 6th harmonic controller.
    qr_ctrl_t qr_d_12th; //!< D-axis 12th harmonic controller.

    // Q-Axis Resonators
    qr_ctrl_t qr_q_6th;  //!< Q-axis 6th harmonic controller.
    qr_ctrl_t qr_q_12th; //!< Q-axis 12th harmonic controller.

    // Saturation Blocks (Safety)
    ctl_saturation_t sat_d; //!< Saturation for D-axis output.
    ctl_saturation_t sat_q; //!< Saturation for Q-axis output.

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable_6th;  //!< 1: Enable 6th harmonic compensation.
    fast_gt flag_enable_12th; //!< 1: Enable 12th harmonic compensation.

} inv_dq_hcm_t;

/**
 * @brief Clears internal states of all resonators.
 */
GMP_STATIC_INLINE void ctl_clear_dq_hcm(inv_dq_hcm_t* hcm)
{
    ctl_clear_qr_controller(&hcm->qr_d_6th);
    ctl_clear_qr_controller(&hcm->qr_d_12th);
    ctl_clear_qr_controller(&hcm->qr_q_6th);
    ctl_clear_qr_controller(&hcm->qr_q_12th);

    ctl_vector2_clear(&hcm->dq_out);
}

/**
 * @brief Initialization parameters for the DQ Harmonic Compensation Module.
 */
typedef struct _tag_inv_dq_hcm_init
{
    parameter_gt fs;        //!< Sampling frequency (Hz).
    parameter_gt freq_base; //!< Fundamental grid frequency (Hz) - used to derive 6th/12th.

    // --- 6th Harmonic (Physical 5th & 7th) ---
    parameter_gt kr_6th; //!< Resonant gain for 6th harmonic (300Hz/360Hz).
    parameter_gt bw_6th; //!< Bandwidth for 6th harmonic (Hz), usually 2-5Hz.

    // --- 12th Harmonic (Physical 11th & 13th) ---
    parameter_gt kr_12th; //!< Resonant gain for 12th harmonic (600Hz/720Hz).
    parameter_gt bw_12th; //!< Bandwidth for 12th harmonic (Hz), usually 2-5Hz.

    // --- Safety ---
    parameter_gt out_limit; //!< Output saturation limit (p.u.). Crucial for feedforward safety.

} inv_dq_hcm_init_t;

/**
 * @brief Initializes the HCM module.
 * @details Calculates QR coefficients using Frequency Pre-warping.
 */
void ctl_init_dq_hcm(inv_dq_hcm_t* hcm, const inv_dq_hcm_init_t* init);

/**
 * @brief Updates the resonant parameters.
 * @param hcm Pointer to HCM object.
 * @param new_freq_base New fundamental frequency (Hz).
 * @param init Original init structure (to retrieve Kr and BW settings).
 */
void ctl_update_dq_hcm_freq(inv_dq_hcm_t* hcm, const inv_dq_hcm_init_t* init);

/**
 * @brief Executes one step of the Harmonic Compensation.
 * @param hcm Pointer to HCM object.
 */
GMP_STATIC_INLINE void ctl_step_dq_hcm(inv_dq_hcm_t* hcm)
{
    // Safety Assertions
    gmp_base_assert(hcm->dq_meas);
    gmp_base_assert(hcm->dq_set);

    ctrl_gt err_d = hcm->dq_set->dat[phase_d] - hcm->dq_meas->dat[phase_d];
    ctrl_gt err_q = hcm->dq_set->dat[phase_q] - hcm->dq_meas->dat[phase_q];

    ctrl_gt out_d = 0.0f;
    ctrl_gt out_q = 0.0f;

    // --- 6th Harmonic (Physical 5th & 7th) ---
    if (hcm->flag_enable_6th)
    {
        out_d += ctl_step_qr_controller(&hcm->qr_d_6th, err_d);
        out_q += ctl_step_qr_controller(&hcm->qr_q_6th, err_q);
    }

    // --- 12th Harmonic (Physical 11th & 13th) ---
    if (hcm->flag_enable_12th)
    {
        out_d += ctl_step_qr_controller(&hcm->qr_d_12th, err_d);
        out_q += ctl_step_qr_controller(&hcm->qr_q_12th, err_q);
    }

    // --- Saturation (Critical Safety) ---
    // Limits the total harmonic injection authority
    hcm->dq_out.dat[phase_d] = ctl_step_saturation(&hcm->sat_d, out_d);
    hcm->dq_out.dat[phase_q] = ctl_step_saturation(&hcm->sat_q, out_q);
}

/**
 * @brief Attaches the HCM to data sources.
 * @param hcm Pointer to HCM object.
 * @param dq_meas Pointer to the measured value (e.g., &gfl->idq).
 * @param dq_set Pointer to the setpoint (e.g., &zero_vector).
 */
GMP_STATIC_INLINE void ctl_attach_dq_hcm(inv_dq_hcm_t* hcm, ctl_vector2_t* dq_meas, ctl_vector2_t* dq_set)
{
    hcm->dq_meas = dq_meas;
    hcm->dq_set = dq_set;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_ADDITIONAL_

/**
 * @}
 */
