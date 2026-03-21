/**
 * @file three_phase_GFL.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset three-phase DC/AC grid following inverter controller.
 * @version 1.0
 * @date 2026-01-11
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

#ifndef _FILE_INV_GFL_PQ_CTRL_
#define _FILE_INV_GFL_PQ_CTRL_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>

#include <ctl/component/digital_power/inv/pll_srf.h>
#include <ctl/component/digital_power/inv/pll_dsogi.h>
#include <ctl/component/digital_power/inv/gfl_core.h>

//////////////////////////////////////////////////////////////////////////
// PQ controller
//

/**
 * @brief P-Q Grid-Following Power Controller.
 * * This controller sits on top of the Current Controller.
 * It regulates Active Power (P) and Reactive Power (Q) by adjusting 
 * the d-axis and q-axis current references.
 * * Topology:
 * P_ref ---(-)--> [PID_P] ----> Id_ref
 * Q_ref ---(-)--> [PID_Q] ----> Iq_ref
 */
typedef struct _tag_gfl_pq_ctrl
{
    //
    // --- Input Interfaces (Pointers) ---
    //
    ctl_vector2_t* vdq_meas; //!< PTR: Feedback grid voltage vector (d,q), output voltage is positive.
    ctl_vector2_t* idq_meas; //!< PTR: Feedback grid current vector (d,q), output current is positive.

    //
    // --- Output Interface ---
    //
    ctl_vector2_t idq_set_out; //!< RO: Calculated current command {Id*, Iq*} to be sent to inner loop.

    //
    // --- Setpoints (User Settings) ---
    //
    ctl_vector2_t pq_set; //!< WR: Power Setpoints {P_ref, Q_ref} in pu.

    //
    // --- Measurement & State (Read-Only) ---
    //
    ctl_vector2_t pq_meas; //!< RO: Calculated instantaneous Active/Reactive Power.
    ctrl_gt s_mag_sq;      //!< RO: Magnitude squared of apparent power (debugging).

    //
    // --- Controllers & Limits ---
    //
    ctl_pid_t pid_p; //!< CTRL: PID for Active Power. Output is Id_ref.
    ctl_pid_t pid_q; //!< CTRL: PID for Reactive Power. Output is Iq_ref.

    ctrl_gt max_i2_mag; //!< PARAM: Maximum allowable current magnitude square (current limit protection).

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable; //!< 1: Enable PQ control (Closed Loop), 0: Disable (Output 0 or Hold).

} gfl_pq_ctrl_t;

/**
 * @brief Initialize the PQ controller with parameters.
 * @param[out] pq Pointer to the PQ controller instance.
 * @param[in] init Initialization parameters.
 */
void ctl_init_gfl_pq(gfl_pq_ctrl_t* pq, parameter_gt p_kp, parameter_gt p_ki, parameter_gt q_kp, parameter_gt q_ki,
                     parameter_gt i_out_max, parameter_gt fs);

/**
 * @brief Reset the PQ controller (clear integrators).
 * @param[in,out] pq Pointer to the PQ controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_gfl_pq(gfl_pq_ctrl_t* pq)
{
    ctl_clear_pid(&pq->pid_p);
    ctl_clear_pid(&pq->pid_q);

    pq->idq_set_out.dat[phase_d] = 0;
    pq->idq_set_out.dat[phase_q] = 0;
}

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq(gfl_pq_ctrl_t* pq, ctl_vector2_t* vdq, ctl_vector2_t* idq);

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq_to_core(gfl_pq_ctrl_t* pq, gfl_inv_ctrl_t* core);

/**
 * @brief Execute one step of the PQ control loop.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @note This should run at a slower rate than the current loop (e.g., 1kHz - 5kHz).
 */
GMP_STATIC_INLINE void ctl_step_gfl_pq(gfl_pq_ctrl_t* pq)
{
    // Safety check for pointers
    if (!pq->vdq_meas || !pq->idq_meas)
        return;

    // Local variables for readability
    ctrl_gt vd = pq->vdq_meas->dat[phase_d];
    ctrl_gt vq = pq->vdq_meas->dat[phase_q];
    ctrl_gt id = pq->idq_meas->dat[phase_d];
    ctrl_gt iq = pq->idq_meas->dat[phase_q];

    // -----------------------------------------------------------
    // 1. Calculate Instantaneous Power (Per-Unit assumption)
    //    P = vd*id + vq*iq
    //    Q = vq*id - vd*iq  (Standard convention, verify with your grid standard)
    // -----------------------------------------------------------
    // Note: If Vq is strictly regulated to 0 by PLL, P ~= Vd*Id, Q ~= -Vd*Iq

    pq->pq_meas.dat[0] = ctl_mul(vd, id) + ctl_mul(vq, iq); // Active Power P
    pq->pq_meas.dat[1] = ctl_mul(vq, id) - ctl_mul(vd, iq); // Reactive Power Q

    // -----------------------------------------------------------
    // 2. Main Control Loop
    // -----------------------------------------------------------
    if (pq->flag_enable)
    {
        // --- Active Power Control (P -> Id) ---
        // Error = Setpoint - Measure
        ctrl_gt p_err = pq->pq_set.dat[0] - pq->pq_meas.dat[0];

        // PID Output is Id reference
        // Source convention: P>0 means discharging to grid.
        pq->idq_set_out.dat[phase_d] = ctl_step_pid_ser(&pq->pid_p, p_err);

        // --- Reactive Power Control (Q -> Iq) ---
        // Error = Setpoint - Measure
        ctrl_gt q_err = pq->pq_set.dat[1] - pq->pq_meas.dat[1];

        // PID Output is Iq reference
        // Source convention Q>0 means discharging inductive reactive power to grid.
        pq->idq_set_out.dat[phase_q] = ctl_step_pid_ser(&pq->pid_q, q_err);

        // -----------------------------------------------------------
        // 3. Current Limiting (Circular Saturation)
        //    Prevent the reference from exceeding converter capability.
        // -----------------------------------------------------------
        ctrl_gt id_ref = pq->idq_set_out.dat[phase_d];
        ctrl_gt iq_ref = pq->idq_set_out.dat[phase_q];

        ctrl_gt i_mag_sq = ctl_mul(id_ref, id_ref) + ctl_mul(iq_ref, iq_ref);
        ctrl_gt i2_limit = pq->max_i2_mag;

        if (i_mag_sq > i2_limit)
        {
            // Simple scaling to keep vector direction but limit magnitude
            ctrl_gt scaler = ctl_sqrt(ctl_div(i2_limit, i_mag_sq));
            pq->idq_set_out.dat[phase_d] *= ctl_mul(scaler, pq->idq_set_out.dat[phase_d]);
            pq->idq_set_out.dat[phase_q] *= ctl_mul(scaler, pq->idq_set_out.dat[phase_q]);
        }
    }
    else
    {
        // If disabled, reset integrators and zero output
        pq->idq_set_out.dat[phase_d] = 0;
        pq->idq_set_out.dat[phase_q] = 0;
    }
}

/** @brief Enable PQ controller */
GMP_STATIC_INLINE void ctl_enable_gfl_pq_ctrl(gfl_pq_ctrl_t* pq)
{
    pq->flag_enable = 1;
}

/** @brief Disable PQ controller */
GMP_STATIC_INLINE void ctl_disable_gfl_pq_ctrl(gfl_pq_ctrl_t* pq)
{
    pq->flag_enable = 0;
}


/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_INV_GFL_PQ_CTRL_

