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
// DTC

#include <ctl/component/motor_control/current_loop/dtc_svm_core.h>

/**
 * @brief Initializes the DTC-SVM core and rigorously maps all physics to PU.
 */
void ctl_init_dtc_svm(dtc_svm_ctrl_t* mc, const dtc_svm_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Calculate PU Base Parameters
    parameter_gt omega_base_elec = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;
    parameter_gt flux_base = init->v_base / omega_base_elec;

    // 2. Flux Observer Constants Calculation (The core mapping)
    // Equation: dPsi_pu = W_base * (V_pu - I_pu * Rs_pu) * dt
    parameter_gt coef_ts_wbase_phy = Ts * omega_base_elec;
    parameter_gt coef_rs_pu_phy = init->mtr_Rs * (init->i_base / init->v_base);

    // High-Pass Drift Compensation: 1.0 - (wc * Ts)
    parameter_gt coef_drift_phy = 1.0f - (init->flux_drift_wc * Ts);

    mc->coef_ts_wbase = float2ctrl(coef_ts_wbase_phy);
    mc->coef_rs_pu = float2ctrl(coef_rs_pu_phy);
    mc->coef_drift_comp = float2ctrl(coef_drift_phy);

    // 3. Simple PI Gain Tuning
    // Flux loop plant is a pure integrator. Kp = Bandwidth.
    parameter_gt kp_flux = CTL_PARAM_CONST_2PI * init->bw_flux;
    parameter_gt ki_flux = kp_flux * 10.0f; // Minimal integral action to eliminate steady state

    // Torque loop plant depends strongly on leakage inductance.
    // This is a generic estimation, usually tuned empirically in DTC.
    parameter_gt kp_torque = init->bw_torque * 0.1f;
    parameter_gt ki_torque = kp_torque * 50.0f;

    // 4. Initialize Controllers
    mc->v_max_pu = float2ctrl((init->v_phase_limit * 1.4142f) / init->v_base);

    ctl_init_pid(&mc->flux_ctrl, float2ctrl(kp_flux), float2ctrl(ki_flux), 0.0f, fs_safe);
    ctl_set_pid_limit(&mc->flux_ctrl, mc->v_max_pu, -mc->v_max_pu);
    ctl_set_pid_int_limit(&mc->flux_ctrl, mc->v_max_pu, -mc->v_max_pu);

    ctl_init_pid(&mc->torque_ctrl, float2ctrl(kp_torque), float2ctrl(ki_torque), 0.0f, fs_safe);
    ctl_set_pid_limit(&mc->torque_ctrl, mc->v_max_pu, -mc->v_max_pu);
    ctl_set_pid_int_limit(&mc->torque_ctrl, mc->v_max_pu, -mc->v_max_pu);

    // 5. State Initialization
    // Seed the flux observer with the permanent magnet flux to avoid singularity at startup.
    // If IM motor, mtr_Flux will be configured to a small non-zero value.
    parameter_gt initial_flux_pu = init->mtr_Flux / flux_base;
    if (initial_flux_pu < 0.01f)
        initial_flux_pu = 0.01f;

    mc->flux_ab_pu.dat[0] = float2ctrl(initial_flux_pu); // Assume flux aligns with alpha axis at startup
    mc->flux_ab_pu.dat[1] = float2ctrl(0.0f);
    mc->flux_mag_pu = float2ctrl(initial_flux_pu);

    mc->torque_est_pu = float2ctrl(0.0f);
    mc->flux_phasor.dat[0] = float2ctrl(1.0f); // cos(0)
    mc->flux_phasor.dat[1] = float2ctrl(0.0f); // sin(0)

    ctl_vector2_clear(&mc->vxy_ctrl);
    ctl_vector2_clear(&mc->vab_out);

    mc->iab_meas = NULL;
    mc->flag_enable = 0;
}
