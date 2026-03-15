/**
 * @file ctl_motor_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// MPC

#include <ctl/component/motor_control/current_loop/pmsm_mpc.h>

// Table of the 8 standard voltage vectors in the alpha-beta frame.
ctl_vector2_t MPC_VOLTAGE_VECTORS_NORMALIZED[8] = {
    {{float2ctrl(0.0f), float2ctrl(0.0f)}},        // V0
    {{float2ctrl(1.0f), float2ctrl(0.0f)}},        // V1
    {{float2ctrl(0.5f), float2ctrl(0.866025f)}},   // V2
    {{float2ctrl(-0.5f), float2ctrl(0.866025f)}},  // V3
    {{float2ctrl(-1.0f), float2ctrl(0.0f)}},       // V4
    {{float2ctrl(-0.5f), float2ctrl(-0.866025f)}}, // V5
    {{float2ctrl(0.5f), float2ctrl(-0.866025f)}},  // V6
    {{float2ctrl(0.0f), float2ctrl(0.0f)}}         // V7
};

void ctl_init_mpc(ctl_mpc_controller_t* mpc, const ctl_mpc_init_t* init)
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
