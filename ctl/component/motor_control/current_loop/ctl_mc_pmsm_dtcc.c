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
// PMSM DPCC init

#include <ctl/component/motor_control/current_loop/PMSM_DPCC.h>

void ctl_init_dpcc(ctl_dpcc_controller_t* dpcc, const ctl_dpcc_init_t* init)
{
    parameter_gt Ts = 1.0f / init->f_ctrl;

    // Store base parameters
    dpcc->rs = float2ctrl(init->Rs);
    dpcc->ld = float2ctrl(init->Ld);
    dpcc->lq = float2ctrl(init->Lq);
    dpcc->psi_f = float2ctrl(init->psi_f);

    // Pre-calculate coefficients to avoid divisions in the control loop
    dpcc->ts_over_ld = float2ctrl(Ts / init->Ld);
    dpcc->ts_over_lq = float2ctrl(Ts / init->Lq);
    dpcc->ld_over_ts = float2ctrl(init->Ld / Ts);
    dpcc->lq_over_ts = float2ctrl(init->Lq / Ts);

    // Initialize all state variables to zero
    ctl_clear_dpcc(dpcc);
}
