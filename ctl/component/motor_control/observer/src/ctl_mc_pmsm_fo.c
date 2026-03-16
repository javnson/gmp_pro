/**
 * @file pmsm_fo.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the State-Feedback Flux Observer (FO).
 *
 * @version 1.2
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm_fo.h>


/**
 * @brief Core initialization function using the bare physical parameters.
 */
void ctl_init_pmsm_fo(ctl_pmsm_fo_t* fo, const ctl_pmsm_fo_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * init->V_base) / (init->Ld * init->I_base);
    parameter_gt k2 = (init->Rs * Ts) / init->Ld;
    parameter_gt k3 = (init->Ld - init->Lq) / init->Ld;

    fo->k1 = float2ctrl(k1);
    fo->k2 = float2ctrl(k2);
    fo->k3 = float2ctrl(k3);

    fo->sf_w_to_rad_tick = float2ctrl(init->W_base * Ts);

    // 2. Sub-module Initialization (PID for BEMF & ATO for Tracking)
    parameter_gt e_limit = (init->e_max_limit_pu > 1e-4f) ? init->e_max_limit_pu : 1.5f;

    // Initialize the two PID controllers acting as Extended State Observers (ESO)
    ctl_init_pid(&fo->pi_emf[0], float2ctrl(init->kp_fo_pu), float2ctrl(init->ki_fo_pu), float2ctrl(0.0f), fs_safe);
    ctl_set_pid_limit(&fo->pi_emf[0], float2ctrl(e_limit), float2ctrl(-e_limit));
    ctl_set_pid_int_limit(&fo->pi_emf[0], float2ctrl(e_limit), float2ctrl(-e_limit));

    ctl_init_pid(&fo->pi_emf[1], float2ctrl(init->kp_fo_pu), float2ctrl(init->ki_fo_pu), float2ctrl(0.0f), fs_safe);
    ctl_set_pid_limit(&fo->pi_emf[1], float2ctrl(e_limit), float2ctrl(-e_limit));
    ctl_set_pid_int_limit(&fo->pi_emf[1], float2ctrl(e_limit), float2ctrl(-e_limit));

    // Initialize ATO. Gains should be scheduled externally if normalized error isn't used.
    ctl_init_ato_pll(&fo->ato_pll, init->ato_bw_hz, 1.0f, init->W_base, fs_safe, 1.5f, -1.5f);

    // 3. Protection Mechanisms Setup
    parameter_gt err_lim = (init->current_err_limit_pu > 1e-3f) ? init->current_err_limit_pu : 0.3f;
    fo->current_err_limit = float2ctrl(err_lim);

    fo->diverge_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (fo->diverge_limit < 1)
        fo->diverge_limit = 1;

    // 4. Finalize Initialization
    ctl_clear_pmsm_fo(fo);
    ctl_disable_pmsm_fo(fo);
}

/**
 * @brief Advanced initialization function utilizing the Consultant models.
 */
void ctl_init_pmsm_fo_consultant(ctl_pmsm_fo_t* fo, const ctl_consultant_pmsm_t* motor,
                                 const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt obs_bw_hz,
                                 parameter_gt ato_bw_hz, parameter_gt fault_time_ms)
{
    ctl_pmsm_fo_init_t bare_init;

    // Physical & PU Base mapping
    bare_init.Rs = motor->Rs;
    bare_init.Ld = motor->Ld;
    bare_init.Lq = motor->Lq;
    bare_init.V_base = pu->V_base;
    bare_init.I_base = pu->I_base;
    bare_init.W_base = pu->W_base;

    bare_init.fs = fs;
    bare_init.ato_bw_hz = ato_bw_hz;
    bare_init.fault_time_ms = fault_time_ms;

    // ========================================================================
    // Auto-Tuning of Observer State-Feedback Gains (Luenberger Pole Placement)
    // ========================================================================
    // Target characteristic: s^2 + 2*zeta*wo*s + wo^2 = 0
    parameter_gt wo = CTL_PARAM_CONST_2PI * obs_bw_hz;
    parameter_gt zeta = 1.0f; // Critically damped for stable estimation

    // Kp_phy = 2 * zeta * wo * Ld - Rs
    parameter_gt kp_phy = (2.0f * zeta * wo * motor->Ld) - motor->Rs;
    if (kp_phy < 0.0f)
        kp_phy = 0.0f; // Safety clamp

    // Ki_phy = wo^2 * Ld
    parameter_gt ki_phy = wo * wo * motor->Ld;

    // Map to strict Per-Unit domain
    // Kp_pu maps current_err_pu to voltage_pu: Kp_pu = Kp_phy * (I_base / V_base)
    parameter_gt scale_p = pu->I_base / pu->V_base;
    bare_init.kp_fo_pu = kp_phy * scale_p;

    // Ki_pu maps current_err_pu to voltage_pu * dt: Ki_pu = Ki_phy * (I_base / V_base) * Ts
    bare_init.ki_fo_pu = ki_phy * scale_p * (1.0f / fs);

    // Default robust margins and limits
    bare_init.e_max_limit_pu = 1.5f;
    bare_init.current_err_limit_pu = 0.3f;

    // Invoke core initialization
    ctl_init_pmsm_fo(fo, &bare_init);
}
