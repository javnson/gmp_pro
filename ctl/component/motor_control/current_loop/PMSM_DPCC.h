/**
 * @file pmsm_dpcc.h
 * @brief Implements a Dead-beat Predictive Current Controller (DPCC) for PMSM.
 * @details This module provides a model-based current controller that aims to
 * eliminate current error in a single control step (dead-beat). It uses the
 * discrete-time model of the PMSM to predict the current at the next time
 * step based on the previously applied voltage. It then calculates the
 * required voltage for the current step to make the predicted current match
 * the reference current at the next step. This approach can offer faster
 * dynamic response than traditional PI controllers.
 *
 * @version 0.2
 * @date 2025-08-09
 *
 * //tex:
 * // The controller is based on the discretized PMSM voltage equations:
 * // i_d(k+1) = i_d(k)+\frac{T_s}{L_d}\left[u_d(k)-R_si_d (k)+ \omega_e L_qi_q(k)\right]
 * // i_q(k+1) = i_q(k)+\frac{T_s}{L_q}\left[ u_q(k)-R_si_q (k)- \omega_e (L_di_d(k)+\Psi_f)\right]
 * // By setting i_d(k+1) and i_q(k+1) to their reference values, we can solve for the required voltage u_d(k) and u_q(k).
 * // To compensate for the one-step digital delay, a prediction step is used.
 *
 */

#ifndef _FILE_PMSM_DPCC_H_
#define _FILE_PMSM_DPCC_H_

#include <ctl/math_block/gmp_math.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Dead-beat Predictive Current Controller (DPCC)                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup DPCC_CONTROLLER Dead-beat Predictive Current Controller
 * @brief A model-based current controller for fast dynamic response.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the DPCC module.
 */
typedef struct
{
    // --- Motor Parameters (SI units) ---
    parameter_gt Rs;    ///< Stator Resistance (Ohm).
    parameter_gt Ld;    ///< D-axis Inductance (H).
    parameter_gt Lq;    ///< Q-axis Inductance (H).
    parameter_gt psi_f; ///< Permanent magnet flux linkage (Wb).

    // --- System Parameters ---
    parameter_gt f_ctrl; ///< Controller execution frequency (Hz).

} ctl_dpcc_init_t;

/**
 * @brief Main structure for the DPCC controller.
 */
typedef struct
{
    // --- Outputs ---
    ctl_vector2_t udq_out; ///< The calculated output voltage vector [ud, uq]^T.

    // --- Internal State ---
    ctl_vector2_t udq_last;      ///< The output voltage from the previous step, used for prediction.
    ctl_vector2_t idq_predicted; ///< The predicted current for the next step [id_pred, iq_pred]^T.

    // --- Pre-calculated Coefficients ---
    ctrl_gt ts_over_ld; ///< Ts / Ld
    ctrl_gt ts_over_lq; ///< Ts / Lq
    ctrl_gt ld_over_ts; ///< Ld / Ts
    ctrl_gt lq_over_ts; ///< Lq / Ts
    ctrl_gt rs;         ///< Stator Resistance
    ctrl_gt ld;         ///< D-axis Inductance
    ctrl_gt lq;         ///< Q-axis Inductance
    ctrl_gt psi_f;      ///< Flux Linkage

} ctl_dpcc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Clears the internal state variables of the DPCC module.
 * @param[out] dpcc Pointer to the DPCC structure.
 */
GMP_STATIC_INLINE void ctl_clear_dpcc(ctl_dpcc_controller_t* dpcc)
{
    // Clear state variables
    ctl_vector2_clear(&dpcc->udq_out);
    ctl_vector2_clear(&dpcc->udq_last);
    ctl_vector2_clear(&dpcc->idq_predicted);
}

/**
 * @brief Initializes the DPCC module with motor and system parameters.
 * @details This function pre-calculates all necessary coefficients from the
 * motor parameters to optimize the real-time step function.
 * @param[out] dpcc Pointer to the DPCC structure.
 * @param[in]  init Pointer to the initialization parameters structure.
 */
void ctl_init_dpcc(ctl_dpcc_controller_t* dpcc, const ctl_dpcc_init_t* init);

/**
 * @brief Executes one step of the Dead-beat Predictive Current Control algorithm.
 * @param[out] dpcc     Pointer to the DPCC structure.
 * @param[in]  idq_ref  The reference d-q current vector [id_ref, iq_ref]^T.
 * @param[in]  idq_fbk  The feedback d-q current vector from the Park transform.
 * @param[in]  omega_e  The current electrical speed (rad/s).
 */
GMP_STATIC_INLINE void ctl_step_dpcc(ctl_dpcc_controller_t* dpcc, const ctl_vector2_t* idq_ref,
                                     const ctl_vector2_t* idq_fbk, ctrl_gt omega_e)
{
    ctrl_gt id_k = idq_fbk->dat[0];
    ctrl_gt iq_k = idq_fbk->dat[1];

    // --- 1. One-step Current Prediction ---
    // Predict the current at the end of this step, i(k+1), using the voltage applied in the previous step, u(k-1).
    // This compensates for the one-step digital control delay.
    ctrl_gt id_pred = id_k + ctl_mul(dpcc->ts_over_ld, dpcc->udq_last.dat[0] - ctl_mul(dpcc->rs, id_k) +
                                                           ctl_mul(omega_e, ctl_mul(dpcc->lq, iq_k)));

    ctrl_gt iq_pred = iq_k + ctl_mul(dpcc->ts_over_lq, dpcc->udq_last.dat[1] - ctl_mul(dpcc->rs, iq_k) -
                                                           ctl_mul(omega_e, (ctl_mul(dpcc->ld, id_k) + dpcc->psi_f)));

    dpcc->idq_predicted.dat[0] = id_pred;
    dpcc->idq_predicted.dat[1] = iq_pred;

    // --- 2. Dead-beat Voltage Calculation ---
    // Calculate the voltage u(k) required to make the current at the next step, i(k+2), equal to the reference i_ref(k+1).
    // We assume i_ref(k+1) is the same as the current i_ref(k).
    dpcc->udq_out.dat[0] = ctl_mul(dpcc->ld_over_ts, (idq_ref->dat[0] - id_pred)) + ctl_mul(dpcc->rs, id_pred) -
                           ctl_mul(omega_e, ctl_mul(dpcc->lq, iq_pred));

    dpcc->udq_out.dat[1] = ctl_mul(dpcc->lq_over_ts, (idq_ref->dat[1] - iq_pred)) + ctl_mul(dpcc->rs, iq_pred) +
                           ctl_mul(omega_e, (ctl_mul(dpcc->ld, id_pred) + dpcc->psi_f));

    // --- 3. Store the calculated voltage for the next prediction step ---
    ctl_vector2_copy(&dpcc->udq_last, &dpcc->udq_out);
}

/**
 * @brief Gets the calculated d-axis output voltage.
 * @param[in] dpcc Pointer to the DPCC structure.
 * @return The calculated d-axis voltage reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dpcc_ud(const ctl_dpcc_controller_t* dpcc)
{
    return dpcc->udq_out.dat[0];
}

/**
 * @brief Gets the calculated q-axis output voltage.
 * @param[in] dpcc Pointer to the DPCC structure.
 * @return The calculated q-axis voltage reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_dpcc_uq(const ctl_dpcc_controller_t* dpcc)
{
    return dpcc->udq_out.dat[1];
}

/** @} */ // end of DPCC_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_DPCC_H_
