/**
 * @file pmsm_mpc.h
 * @brief Implements a Finite Control Set Model Predictive Current Controller (FCS-MPC) for PMSM.
 * @details This module provides an advanced model-based current controller. Instead of
 * traditional PI loops, it uses a discrete-time model of the PMSM to predict the
 * current response for each of the 8 possible inverter voltage vectors. It then
 * selects the vector that minimizes a cost function, typically the squared error
 * between the predicted current and the reference current. This approach offers
 * very fast dynamic response and handles multivariable control and constraints
 * intuitively.
 *
 * @version 1.0
 * @date 2025-08-07
 *
 * //tex:
 * // The controller uses the discretized PMSM voltage equations to predict the
 * // current at the next time step k+1:
 * // \mathbf{i}(k+1) = \mathbf{A}(\omega_e) \mathbf{i}(k) + \mathbf{B} \mathbf{u}(k) + \mathbf{E}(\omega_e)
 * // A cost function is then evaluated for each possible voltage vector \mathbf{u}_j:
 * // J_j = || \mathbf{i}_{ref}(k+1) - \mathbf{i}_{pred, j}(k+1) ||^2
 * // The voltage vector \mathbf{u}_j that minimizes J is selected and applied.
 *
 */

#ifndef _FILE_PMSM_MPC_H_
#define _FILE_PMSM_MPC_H_

#include <ctl/math_block/coordinate/coord_trans.h> // For Park/Inverse Park transforms
#include <ctl/math_block/vector_lite/matrix2.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Finite Control Set - Model Predictive Current Controller (FCS-MPC)        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MPC_CONTROLLER FCS-MPC Current Controller
 * @brief A model-predictive current controller for fast dynamic response.
 * @{
 */

//================================================================================
// Type Defines & Data
//================================================================================

/**
 * @brief Table of the 8 standard voltage vectors in the alpha-beta frame.
 * @details The values are normalized and must be scaled by (2/3 * Udc).
 * V0 and V7 are zero vectors.
 */
static const ctl_vector2_t MPC_VOLTAGE_VECTORS_NORMALIZED[8] = {
    {{0.0f, 0.0f}},        // V0
    {{1.0f, 0.0f}},        // V1
    {{0.5f, 0.866025f}},   // V2
    {{-0.5f, 0.866025f}},  // V3
    {{-1.0f, 0.0f}},       // V4
    {{-0.5f, -0.866025f}}, // V5
    {{0.5f, -0.866025f}},  // V6
    {{0.0f, 0.0f}}         // V7
};

/**
 * @brief Initialization parameters for the MPC module.
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

} ctl_mpc_init_t;

/**
 * @brief Main structure for the MPC controller.
 */
typedef struct
{
    // --- Output ---
    uint8_t optimal_vector_index; ///< The index (0-7) of the best voltage vector.

    // --- Pre-calculated Model Matrices (Constant Parts) ---
    ctl_matrix2_t A_const; ///< Constant part of the state matrix A.
    ctl_matrix2_t B;       ///< Input matrix B.

    // --- Model Parameters ---
    ctrl_gt Ld;    ///< D-axis Inductance.
    ctrl_gt Lq;    ///< Q-axis Inductance.
    ctrl_gt psi_f; ///< Flux Linkage.
    ctrl_gt Ts;    ///< Sampling Time (1 / f_ctrl).

} ctl_mpc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the MPC module with motor and system parameters.
 * @details This function pre-calculates the discrete-time system matrices
 * based on the provided motor parameters to optimize the real-time step function.
 * @param[out] mpc  Pointer to the MPC structure.
 * @param[in]  init Pointer to the initialization parameters structure.
 */
GMP_STATIC_INLINE void ctl_init_mpc(ctl_mpc_controller_t* mpc, const ctl_mpc_init_t* init)
{
    mpc->optimal_vector_index = 0; // Default to zero vector
    mpc->Ld = (ctrl_gt)init->Ld;
    mpc->Lq = (ctrl_gt)init->Lq;
    mpc->psi_f = (ctrl_gt)init->psi_f;
    mpc->Ts = 1.0f / (ctrl_gt)init->f_ctrl;

    // Pre-calculate the constant parts of the discrete-time model matrices (A and B)
    // i(k+1) = A*i(k) + B*u(k) + E
    // A = [[1 - Ts*Rs/Ld, Ts*we*Lq/Ld], [-Ts*we*Ld/Lq, 1 - Ts*Rs/Lq]]
    // B = [[Ts/Ld, 0], [0, Ts/Lq]]
    // E = [0, -Ts*we*psi_f/Lq]

    ctrl_gt ts_rs_over_ld = mpc->Ts * (ctrl_gt)init->Rs / mpc->Ld;
    ctrl_gt ts_rs_over_lq = mpc->Ts * (ctrl_gt)init->Rs / mpc->Lq;

    ctl_matrix2_set(&mpc->A_const, 0, 0, 1.0f - ts_rs_over_ld);
    ctl_matrix2_set(&mpc->A_const, 0, 1, 0.0f); // Speed dependent term
    ctl_matrix2_set(&mpc->A_const, 1, 0, 0.0f); // Speed dependent term
    ctl_matrix2_set(&mpc->A_const, 1, 1, 1.0f - ts_rs_over_lq);

    ctl_matrix2_set(&mpc->B, 0, 0, mpc->Ts / mpc->Ld);
    ctl_matrix2_set(&mpc->B, 0, 1, 0.0f);
    ctl_matrix2_set(&mpc->B, 1, 0, 0.0f);
    ctl_matrix2_set(&mpc->B, 1, 1, mpc->Ts / mpc->Lq);
}

/**
 * @brief Executes one step of the FCS-MPC algorithm.
 * @param[out] mpc      Pointer to the MPC structure.
 * @param[in]  idq_ref  The reference d-q current vector [id_ref, iq_ref]^T.
 * @param[in]  idq_fbk  The feedback d-q current vector from the Park transform.
 * @param[in]  phasor   The current rotor angle phasor {sin(theta), cos(theta)}.
 * @param[in]  omega_e  The current electrical speed (rad/s).
 * @param[in]  udc      The measured DC bus voltage (V).
 */
GMP_STATIC_INLINE void ctl_step_mpc(ctl_mpc_controller_t* mpc, const ctl_vector2_t* idq_ref,
                                    const ctl_vector2_t* idq_fbk, const ctl_vector2_t* phasor, ctrl_gt omega_e,
                                    ctrl_gt udc)
{
    ctl_matrix2_t A_full;
    ctl_vector2_t E;
    ctl_vector2_t i_pred;
    ctrl_gt min_cost = 1e12f; // Initialize with a very large number
    uint8_t best_vector_idx = 0;
    ctrl_gt voltage_scale = (2.0f / 3.0f) * udc;

    // 1. Construct the speed-dependent parts of the model matrices
    A_full = mpc->A_const;
    ctl_matrix2_set(&A_full, 0, 1, mpc->Ts * omega_e * mpc->Lq / mpc->Ld);
    ctl_matrix2_set(&A_full, 1, 0, -mpc->Ts * omega_e * mpc->Ld / mpc->Lq);

    E.dat[0] = 0.0f;
    E.dat[1] = -mpc->Ts * omega_e * mpc->psi_f / mpc->Lq;

    // 2. Iterate through all 8 possible voltage vectors
    for (uint8_t i = 0; i < 8; ++i)
    {
        // Get the test voltage vector in the alpha-beta frame
        ctl_vector2_t u_ab = ctl_vector2_scale(MPC_VOLTAGE_VECTORS_NORMALIZED[i], voltage_scale);

        // Transform the test voltage to the d-q frame
        ctl_vector2_t u_dq;
        ctl_ct_park_vec(&u_ab, phasor, &u_dq);

        // 3. Predict the next state: i(k+1) = A*i(k) + B*u(k) + E
        ctl_vector2_t Ax = ctl_matrix2_mul_vector(A_full, *idq_fbk);
        ctl_vector2_t Bu = ctl_matrix2_mul_vector(mpc->B, u_dq);
        i_pred = ctl_vector2_add(ctl_vector2_add(Ax, Bu), E);

        // 4. Calculate the cost function: J = (id_ref - id_pred)^2 + (iq_ref - iq_pred)^2
        ctl_vector2_t error_vec = ctl_vector2_sub(*idq_ref, i_pred);
        ctrl_gt cost = ctl_vector2_mag_sq(error_vec);

        // 5. Find the vector that minimizes the cost
        if (cost < min_cost)
        {
            min_cost = cost;
            best_vector_idx = i;
        }
    }

    // 6. Set the optimal vector index as the output
    mpc->optimal_vector_index = best_vector_idx;
}

/**
 * @brief Gets the index of the optimal voltage vector calculated by the MPC.
 * @param[in] mpc Pointer to the MPC structure.
 * @return The index (0-7) of the optimal voltage vector.
 */
GMP_STATIC_INLINE uint8_t ctl_get_mpc_optimal_vector_index(const ctl_mpc_controller_t* mpc)
{
    return mpc->optimal_vector_index;
}

/** @} */ // end of MPC_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_MPC_H_
