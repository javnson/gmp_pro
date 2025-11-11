/**
 * @file acm_mpc.h
 * @brief Implements a Finite Control Set Model Predictive Controller (FCS-MPC) for ACM.
 *
 * @version 1.0
 * @date 2025-08-07
 *
 */

#ifndef _FILE_ACM_MPC_H_
#define _FILE_ACM_MPC_H_

#include <ctl/math_block/coordinate/coord_trans.h> // For Park/Inverse Park transforms
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Finite Control Set - Model Predictive Controller for ACM (FCS-MPC)        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MPC_CONTROLLER_ACM FCS-MPC Current & Flux Controller for ACM
 * @details This module provides an advanced model-based controller for AC Induction Motors.
 * It uses a discrete-time model of the ACM to predict the future state (stator currents
 * and rotor flux) for each of the 8 possible inverter voltage vectors. It then
 * selects the vector that minimizes a cost function, which penalizes both the
 * current tracking error and any deviation from the desired rotor flux magnitude.
 * This provides very fast torque and flux response.
 * The controller uses a discretized ACM model to predict the next state:
 * @f[ \mathbf{x}(k+1) = \mathbf{A}_d(\omega_r) \mathbf{x}(k) + \mathbf{B}_d \mathbf{u}(k) @f]
 * where the state vector is @f( \mathbf{x} = [\mathbf{i}_s, \mathbf{\psi}_r]^T @f).
 * A cost function is evaluated for each possible voltage vector \mathbf{u}_j:
 * @f[ J_j = || \mathbf{i}_{s,ref}(k+1) - \mathbf{i}_{s,pred,j}(k+1) ||^2 + \lambda (\psi_{r,ref}^2 - ||\mathbf{\psi}_{r,pred,j}(k+1)||^2)^2 @f]
 * The voltage vector @f( \mathbf{u}_j @f) that minimizes J is selected and applied.
 * @brief A model-predictive controller for fast torque and flux response in ACMs.
 * @{
 */

//================================================================================
// Type Defines & Data
//================================================================================

/**
 * @brief Table of the 8 standard voltage vectors in the alpha-beta frame.
 * @details The values are normalized and must be scaled by (2/3 * Udc).
 */
ctl_vector2_t MPC_VOLTAGE_VECTORS_NORMALIZED_ACM[8];

/**
 * @brief Initialization parameters for the ACM MPC module.
 */
typedef struct
{
    // --- Motor Parameters (SI units) ---
    parameter_gt Rs;     ///< Stator Resistance (Ohm).
    parameter_gt Rr;     ///< Rotor Resistance (Ohm).
    parameter_gt Ls;     ///< Stator Inductance (H).
    parameter_gt Lr;     ///< Rotor Inductance (H).
    parameter_gt Lm;     ///< Mutual Inductance (H).
    uint16_t pole_pairs; ///< Number of motor pole pairs.

    // --- System Parameters ---
    parameter_gt f_ctrl; ///< Controller execution frequency (Hz).

} ctl_acm_mpc_init_t;

/**
 * @brief Main structure for the ACM MPC controller.
 */
typedef struct
{
    // --- Output ---
    uint8_t optimal_vector_index; ///< The index (0-7) of the best voltage vector.

    // --- Internal State ---
    ctl_vector2_t psi_r_est; ///< Estimated rotor flux vector [psi_r_alpha, psi_r_beta]^T.

    // --- Setpoints ---
    ctrl_gt flux_ref_sq; ///< The squared reference value for the rotor flux magnitude.

    // --- Tuning ---
    ctrl_gt lambda_flux; ///< Weighting factor for the flux error in the cost function.

    // --- Pre-calculated Model Constants ---
    ctrl_gt Ts;         ///< Sampling Time (1 / f_ctrl).
    ctrl_gt c_iss;      ///< Coefficient for i_s(k) term in i_s prediction.
    ctrl_gt c_is_u;     ///< Coefficient for u_s(k) term in i_s prediction.
    ctrl_gt c_is_pr;    ///< Coefficient for psi_r(k) term in i_s prediction.
    ctrl_gt c_is_pr_w;  ///< Coefficient for omega_r * psi_r(k) term in i_s prediction.
    ctrl_gt c_pr_is;    ///< Coefficient for i_s(k) term in psi_r prediction.
    ctrl_gt c_pr_pr;    ///< Coefficient for psi_r(k) term in psi_r prediction.
    ctrl_gt pole_pairs; ///< Number of pole pairs.

} ctl_acm_mpc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the ACM MPC module with motor and system parameters.
 * @param[out] mpc  Pointer to the MPC structure.
 * @param[in]  init Pointer to the initialization parameters structure.
 */
void ctl_init_acm_mpc(ctl_acm_mpc_controller_t* mpc, const ctl_acm_mpc_init_t* init);

/**
 * @brief Sets the reference magnitude for the rotor flux.
 * @param[out] mpc      Pointer to the MPC structure.
 * @param[in]  flux_ref The desired rotor flux magnitude in Webers (Wb).
 */
GMP_STATIC_INLINE void ctl_set_acm_mpc_flux_ref(ctl_acm_mpc_controller_t* mpc, ctrl_gt flux_ref)
{
    mpc->flux_ref_sq = flux_ref * flux_ref;
}

/**
 * @brief Executes one step of the ACM FCS-MPC algorithm.
 * @param[out] mpc      Pointer to the MPC structure.
 * @param[in]  idq_ref  The reference d-q stator current vector.
 * @param[in]  is_ab_fbk The feedback alpha-beta stator current vector.
 * @param[in]  omega_r  The current rotor mechanical speed (rad/s).
 * @param[in]  udc      The measured DC bus voltage (V).
 */
GMP_STATIC_INLINE void ctl_step_acm_mpc(ctl_acm_mpc_controller_t* mpc, ctl_vector2_t* idq_ref,
                                        ctl_vector2_t* is_ab_fbk, ctrl_gt omega_r, ctrl_gt _udc)
{
    int i;

    ctl_vector2_t is_ab_ref, is_pred, psi_r_pred;
    ctl_vector2_t J_psi_r_est = {{mpc->psi_r_est.dat[1], -mpc->psi_r_est.dat[0]}}; // Rotated by -90 deg
    ctrl_gt min_cost = 1e12f;
    uint8_t best_vector_idx = 0;
    ctrl_gt voltage_scale = (2.0f / 3.0f) * _udc;
    ctrl_gt omega_e_rotor = omega_r * mpc->pole_pairs;

    // 1. Transform current reference from d-q frame to stationary alpha-beta frame
    // The d-q frame is aligned with the estimated rotor flux from the previous step.
    ctl_vector2_t flux_phasor;
    ctl_vector2_normalize(&flux_phasor, &mpc->psi_r_est);
    ctl_ct_ipark2(idq_ref, &flux_phasor, &is_ab_ref);

    // 2. Iterate through all 8 possible voltage vectors
    for (i = 0; i < 8; ++i)
    {
        ctl_vector2_t u_ab;
        ctl_vector2_scale(&u_ab, &MPC_VOLTAGE_VECTORS_NORMALIZED_ACM[i], voltage_scale);

        ctl_vector2_t temp1, temp2;

        // 3. Predict next state (current and flux) using the discrete model
        //tex:
        // $$ i_s(k+1) = c_{iss}*i_s(k) + c_i s_u*u_s(k) + c_i s_pr \times \psi_r(k) + c_i s_p r_w \times omega_r \times J \times psi_r(k) $$
        ctl_vector2_scale(&temp1, is_ab_fbk, mpc->c_iss);
        ctl_vector2_scale(&temp2, &u_ab, mpc->c_is_u);
        ctl_vector2_add(&is_pred, &temp1, &temp2);
        ctl_vector2_scale(&temp1, &mpc->psi_r_est, mpc->c_is_pr);
        ctl_vector2_add(&is_pred, &is_pred, &temp1);
        ctl_vector2_scale(&temp2, &J_psi_r_est, ctl_mul(mpc->c_is_pr_w, omega_e_rotor));
        ctl_vector2_add(&is_pred, &is_pred, &temp2);

        //tex:
        // $$ \psi_r(k+1) = c_pr_is*i_s(k) + c_pr_pr\times \psi_r(k) - Ts\times \omega_r*J\times \psi_r(k) $$
        ctl_vector2_scale(&temp1, is_ab_fbk, mpc->c_pr_is);
        ctl_vector2_scale(&temp2, &mpc->psi_r_est, mpc->c_pr_pr);
        ctl_vector2_add(&psi_r_pred, &temp1, &temp2);
        ctl_vector2_scale(&temp1, &J_psi_r_est, -ctl_mul(mpc->Ts, omega_e_rotor));
        ctl_vector2_add(&psi_r_pred, &psi_r_pred, &temp1);

        // 4. Calculate cost function
        ctl_vector2_t current_error;
        ctl_vector2_sub(&current_error, &is_ab_ref, &is_pred);
        ctrl_gt flux_error = mpc->flux_ref_sq - ctl_vector2_mag_sq(&psi_r_pred);
        ctrl_gt cost = ctl_vector2_mag_sq(&current_error) + mpc->lambda_flux * (flux_error * flux_error);

        // 5. Find the vector that minimizes the cost
        if (cost < min_cost)
        {
            min_cost = cost;
            best_vector_idx = i;
        }
    }

    // 6. Set the optimal vector index as the output
    mpc->optimal_vector_index = best_vector_idx;

    // 7. Update the internal flux observer state for the next cycle
    // using the prediction generated by the chosen optimal vector.
    ctl_vector2_t u_ab_opt;
    ctl_vector2_scale(&u_ab_opt, &MPC_VOLTAGE_VECTORS_NORMALIZED_ACM[best_vector_idx], voltage_scale);
    ctl_vector2_t temp1, temp2;
    ctl_vector2_scale(&temp1, is_ab_fbk, mpc->c_pr_is);
    ctl_vector2_scale(&temp2, &mpc->psi_r_est, mpc->c_pr_pr);
    ctl_vector2_add(&mpc->psi_r_est, &temp1, &temp2);
    ctl_vector2_scale(&temp1, &J_psi_r_est, -ctl_mul(mpc->Ts, omega_e_rotor));
    ctl_vector2_add(&mpc->psi_r_est, &mpc->psi_r_est, &temp1);
}

/**
 * @brief Gets the index of the optimal voltage vector calculated by the MPC.
 * @param[in] mpc Pointer to the MPC structure.
 * @return The index (0-7) of the optimal voltage vector.
 */
GMP_STATIC_INLINE uint8_t ctl_get_acm_mpc_optimal_vector_index(const ctl_acm_mpc_controller_t* mpc)
{
    return mpc->optimal_vector_index;
}

/** 
 * @} 
 */ // end of MPC_CONTROLLER_ACM group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ACM_MPC_H_
