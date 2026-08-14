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

#include <ctl/math_block/gmp_math.h>

#include <ctl/component/motor_control/observer/acim_fo.h>

//=================================================================================================
// Explicit-coefficient initialization

void ctl_init_im_fo(ctl_im_fo_t* fo, const ctl_im_fo_init_t* init)
{
    // 1. Assign Pre-calculated Scale Factors
    fo->sf_cm_k1 = param2ctrl(init->sf_cm_k1);
    fo->sf_cm_k2 = param2ctrl(init->sf_cm_k2);
    fo->sf_rs = param2ctrl(init->sf_rs);
    fo->sf_lm_over_lr = param2ctrl(init->sf_lm_over_lr);
    fo->sf_sigma_ls = param2ctrl(init->sf_sigma_ls);
    fo->sf_lr_over_lm = param2ctrl(init->sf_lr_over_lm);
    fo->sf_v_int = param2ctrl(init->sf_v_int);
    fo->sf_vm_leak = CTL_CTRL_CONST_ZERO;
    fo->sf_slip_const = param2ctrl(init->sf_slip_const);
    fo->sf_torque_const = param2ctrl(init->sf_torque_const);
    fo->sf_mech_w_to_angle = param2ctrl(init->sf_mech_w_to_angle);

    // 2. Sub-module Initialization (PI Compensators for Voltage Model)
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt u_limit = (init->u_comp_limit_pu > 1e-4f) ? init->u_comp_limit_pu : 0.5f;

    ctl_init_pid(&fo->pi_comp[0], init->kp_comp_pu, init->ki_comp_pu, CTL_PARAM_CONST_ZERO,
                 fs_safe);
    ctl_set_pid_limit(&fo->pi_comp[0], real2ctrl(u_limit), real2ctrl(-u_limit));
    ctl_set_pid_int_limit(&fo->pi_comp[0], real2ctrl(u_limit), real2ctrl(-u_limit));

    ctl_init_pid(&fo->pi_comp[1], init->kp_comp_pu, init->ki_comp_pu, CTL_PARAM_CONST_ZERO,
                 fs_safe);
    ctl_set_pid_limit(&fo->pi_comp[1], real2ctrl(u_limit), real2ctrl(-u_limit));
    ctl_set_pid_int_limit(&fo->pi_comp[1], real2ctrl(u_limit), real2ctrl(-u_limit));

    // Initialize ATO for flux vector tracking.
    // Limits max synchronous speed to +/- 2.0 PU (sufficient for deep field weakening).
    ctl_init_ato_pll(&fo->ato_pll, init->ato_bw_hz, 1.0f, init->omega_base, fs_safe, 2.0f, -2.0f);

    // 3. Safety Mechanisms Setup
    fo->flux_min_limit = real2ctrl((init->flux_min_pu > 1e-6f) ? init->flux_min_pu : 0.001f);
    fo->flux_max_limit = real2ctrl((init->flux_max_pu > init->flux_min_pu) ? init->flux_max_pu : 1.5f);

    fo->diverge_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (fo->diverge_limit < 1)
        fo->diverge_limit = 1;

    // 4. Finalize Initialization
    fo->flag_enable_compensation = 1;
    ctl_clear_im_fo(fo);
    ctl_disable_im_fo(fo);
}

//=================================================================================================
// Motor/PU consultant auto-tuning

void ctl_init_im_fo_consultant(ctl_im_fo_t* fo, const ctl_consultant_im_t* motor, const ctl_consultant_pu_im_t* pu,
                               parameter_gt fs, parameter_gt comp_bw_hz, parameter_gt ato_bw_hz,
                               parameter_gt fault_time_ms)
{
    ctl_im_fo_init_t bare_init;
    parameter_gt Ts = 1.0f / fs;

    bare_init.fs = fs;
    bare_init.ato_bw_hz = ato_bw_hz;
    bare_init.fault_time_ms = fault_time_ms;
    bare_init.omega_base = pu->W_base;
    bare_init.sf_mech_w_to_angle = pu->W_base * Ts /
                                   (CTL_PARAM_CONST_2PI * (parameter_gt)motor->pole_pairs);

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

    // With d(psi_pu)/dt = W_base*(v_pu-u_comp_pu), these gains place
    // s^2 + 2*w_comp*s + w_comp^2 (critical damping). ctl_init_pid()
    // performs the single required Ki/fs discretization.
    bare_init.kp_comp_pu = 2.0f * w_comp / pu->W_base;

    bare_init.ki_comp_pu = w_comp * w_comp / pu->W_base;

    // Allow compensation to reach up to 50% of nominal voltage to handle deep parameter drift
    bare_init.u_comp_limit_pu = 0.5f;

    // Rotor flux base is V_base/W_base = L_base*I_base. A threshold equal
    // to 2% of the flux produced by 1 PU magnetizing current avoids imposing
    // a PMSM-like 0.1 PU floor on low-inductance ACIM parameter sets.
    bare_init.flux_min_pu = (lm_pu * 0.02f > 1e-4f) ? lm_pu * 0.02f : 1e-4f;
    bare_init.flux_max_pu = 1.5f;

    // Invoke core initialization
    ctl_init_im_fo(fo, &bare_init);
}

//=================================================================================================
// Runtime observer gain scheduling

void ctl_set_im_fo_compensation_bw(ctl_im_fo_t* fo, parameter_gt comp_bw_hz,
                                   parameter_gt omega_base, parameter_gt fs)
{
    parameter_gt w_base = (omega_base > 1e-6f) ? omega_base : 1.0f;
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt w_comp = CTL_PARAM_CONST_2PI * comp_bw_hz;
    parameter_gt kp = 2.0f * w_comp / w_base;
    parameter_gt ki_per_sample = (w_comp * w_comp / w_base) / fs_safe;
    int axis;

    for (axis = 0; axis < 2; ++axis)
    {
        fo->pi_comp[axis].kp = real2ctrl(kp);
        fo->pi_comp[axis].ki = real2ctrl(ki_per_sample);
        fo->pi_comp[axis].kd = CTL_CTRL_CONST_ZERO;
    }
}

void ctl_set_im_fo_voltage_model_leak(ctl_im_fo_t* fo, parameter_gt cutoff_hz, parameter_gt fs)
{
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt cutoff = (cutoff_hz > 0.0f) ? cutoff_hz : 0.0f;
    fo->sf_vm_leak = real2ctrl(1.0f - param_exp(-CTL_PARAM_CONST_2PI * cutoff / fs_safe));
}
