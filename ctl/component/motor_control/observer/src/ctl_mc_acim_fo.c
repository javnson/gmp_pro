/**
 * @file im_fo.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the State-Feedback Flux Observer (FO) for IM.
 *
 * @version 1.1
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include <gmp_core.h>

#include <ctl/component/motor_control/observer/im_fo.h>

/**
 * @brief Core initialization function using explicit scale factors.
 */
void ctl_init_im_fo(ctl_im_fo_t* fo, const ctl_im_fo_init_t* init)
{
    // 1. Assign Pre-calculated Scale Factors
    fo->sf_cm_k1 = float2ctrl(init->sf_cm_k1);
    fo->sf_cm_k2 = float2ctrl(init->sf_cm_k2);
    fo->sf_rs = float2ctrl(init->sf_rs);
    fo->sf_lm_over_lr = float2ctrl(init->sf_lm_over_lr);
    fo->sf_sigma_ls = float2ctrl(init->sf_sigma_ls);
    fo->sf_lr_over_lm = float2ctrl(init->sf_lr_over_lm);
    fo->sf_v_int = float2ctrl(init->sf_v_int);
    fo->sf_slip_const = float2ctrl(init->sf_slip_const);
    fo->sf_torque_const = float2ctrl(init->sf_torque_const);

    // 2. Sub-module Initialization (PI Compensators for Voltage Model)
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt u_limit = (init->u_comp_limit_pu > 1e-4f) ? init->u_comp_limit_pu : 0.5f;

    ctl_init_pid(&fo->pi_comp[0], float2ctrl(init->kp_comp_pu), float2ctrl(init->ki_comp_pu), float2ctrl(0.0f),
                 fs_safe);
    ctl_set_pid_limit(&fo->pi_comp[0], float2ctrl(u_limit), float2ctrl(-u_limit));
    ctl_set_pid_int_limit(&fo->pi_comp[0], float2ctrl(u_limit), float2ctrl(-u_limit));

    ctl_init_pid(&fo->pi_comp[1], float2ctrl(init->kp_comp_pu), float2ctrl(init->ki_comp_pu), float2ctrl(0.0f),
                 fs_safe);
    ctl_set_pid_limit(&fo->pi_comp[1], float2ctrl(u_limit), float2ctrl(-u_limit));
    ctl_set_pid_int_limit(&fo->pi_comp[1], float2ctrl(u_limit), float2ctrl(-u_limit));

    // Initialize ATO for flux vector tracking.
    // Limits max synchronous speed to +/- 2.0 PU (sufficient for deep field weakening).
    ctl_init_ato_pll(&fo->ato_pll, init->ato_bw_hz, 1.0f, 1.0f, fs_safe, 2.0f, -2.0f);

    // 3. Safety Mechanisms Setup
    fo->flux_min_limit = float2ctrl(0.1f); // 10% of nominal flux is the lowest valid limit

    fo->diverge_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (fo->diverge_limit < 1)
        fo->diverge_limit = 1;

    // 4. Finalize Initialization
    ctl_clear_im_fo(fo);
    ctl_disable_im_fo(fo);
}

/**
 * @brief Advanced initialization function utilizing the IM Consultant models.
 */
void ctl_init_im_fo_consultant(ctl_im_fo_t* fo, const ctl_consultant_im_t* motor, const ctl_consultant_pu_im_t* pu,
                               parameter_gt fs, parameter_gt comp_bw_hz, parameter_gt ato_bw_hz,
                               parameter_gt fault_time_ms)
{
    ctl_im_fo_init_t bare_init;
    parameter_gt Ts = 1.0f / fs;

    bare_init.fs = fs;
    bare_init.ato_bw_hz = ato_bw_hz;
    bare_init.fault_time_ms = fault_time_ms;

    // ========================================================================
    // Physical Parameter PU Derivations (Calculating all 'sf_' constants)
    // ========================================================================
    // Stator resistance (PU)
    bare_init.sf_rs = motor->Rs / pu->Z_s_base;

    // Inductance ratios
    bare_init.sf_lm_over_lr = motor->Lm / motor->Lr;
    bare_init.sf_lr_over_lm = motor->Lr / motor->Lm;

    // Transient Inductance PU
    bare_init.sf_sigma_ls = motor->sigma_Ls / pu->L_s_base;

    // Current Model Constants:
    // tau_r = Lr / Rr. Base frequency is W_base.
    parameter_gt tau_r = motor->tau_r;
    bare_init.sf_cm_k1 = tau_r / (tau_r + Ts);

    // Lm in PU is Lm / L_s_base.
    parameter_gt lm_pu = motor->Lm / pu->L_s_base;
    bare_init.sf_cm_k2 = (lm_pu * Ts) / (tau_r + Ts);

    // Voltage Integration Constant: W_base * Ts
    bare_init.sf_v_int = pu->W_base * Ts;

    // Slip calculation constant: Lm_pu / (tau_r * W_base)
    bare_init.sf_slip_const = lm_pu / (tau_r * pu->W_base);

    // Torque constant: Lm/Lr (PU magnitude)
    bare_init.sf_torque_const = motor->Lm / motor->Lr;

    // ========================================================================
    // Auto-Tuning of Gopinath PI Compensator
    // ========================================================================
    // The PI compensator defines the crossover frequency between the current model
    // and voltage model. Above comp_bw_hz, the voltage model dominates.
    parameter_gt w_comp = CTL_PARAM_CONST_2PI * comp_bw_hz;

    // PI mapping: Kp roughly determines the bandwidth (rad/s)
    // Scale to PU: output is voltage correction PU, input is flux error PU
    // Kp_pu = w_comp * (Flux_base / V_base) = w_comp / W_base
    bare_init.kp_comp_pu = w_comp / pu->W_base;

    // Ki_pu = w_comp^2 / W_base * Ts
    bare_init.ki_comp_pu = (w_comp * w_comp / pu->W_base) * Ts;

    // Allow compensation to reach up to 50% of nominal voltage to handle deep parameter drift
    bare_init.u_comp_limit_pu = 0.5f;

    // Invoke core initialization
    ctl_init_im_fo(fo, &bare_init);
}
