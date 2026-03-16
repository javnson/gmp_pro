/**
 * @file pmsm_hfi.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the Pulsating Sinusoidal HFI Observer.
 *
 * @version 1.0
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */


#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm_hfi.h>

void ctl_init_pmsm_hfi(ctl_pmsm_hfi_t* hfi, const ctl_pmsm_hfi_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;

    // 1. Injection Parameters
    hfi->v_inj_amp = float2ctrl(init->v_inj_pu);

    // Carrier angle steps per ISR tick (PU): (f_inj / f_ctrl)
    hfi->sf_carrier_step = float2ctrl(init->f_inj_hz / fs_safe);
    hfi->sf_delay_comp_pu = float2ctrl(init->delay_comp_rad / CTL_PARAM_CONST_2PI);
    hfi->sf_err_gain = float2ctrl(init->err_gain_sf);

    // 2. Sub-module Initialization
    // The IQ LPF should be low enough to filter out the injection frequency, leaving only fundamental torque current.
    parameter_gt f_iq_lpf = (init->f_lpf_iq_hz > 1.0f) ? init->f_lpf_iq_hz : (init->f_inj_hz / 10.0f);
    ctl_init_filter_iir1_lpf(&hfi->lpf_iq, fs_safe, f_iq_lpf);

    // The Demodulation LPF extracts the DC envelope of the error. Must be < 2 * f_inj.
    parameter_gt f_demod_lpf = (init->f_lpf_demod_hz > 1.0f) ? init->f_lpf_demod_hz : (init->f_inj_hz / 5.0f);
    ctl_init_filter_iir1_lpf(&hfi->lpf_demod, fs_safe, f_demod_lpf);

    // Initialize ATO (Limits are usually small since HFI operates near zero speed)
    ctl_init_ato_pll(&hfi->ato_pll, init->ato_bw_hz, 1.0f, 1.0f, fs_safe, 0.2f, -0.2f); // Restrict speed to 20% PU max

    // 3. Finalize Initialization
    ctl_clear_pmsm_hfi(hfi);
    ctl_disable_pmsm_hfi(hfi);
}

void ctl_init_pmsm_hfi_consultant(ctl_pmsm_hfi_t* hfi, const ctl_consultant_pmsm_t* motor,
                                  const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt f_inj_hz,
                                  parameter_gt v_inj_v, parameter_gt ato_bw_hz)
{
    ctl_pmsm_hfi_init_t bare_init;

    bare_init.fs = fs;
    bare_init.f_inj_hz = f_inj_hz;
    bare_init.v_inj_pu = v_inj_v / pu->V_base;
    bare_init.ato_bw_hz = ato_bw_hz;

    // Auto-configure LPFs based on injection frequency
    bare_init.f_lpf_iq_hz = f_inj_hz / 10.0f; // Typical empirical value
    bare_init.f_lpf_demod_hz = f_inj_hz / 5.0f;

    // ========================================================================
    // Auto-Tuning of Error Gain & Phase Delay
    // ========================================================================
    // Theoretical HF q-axis current amplitude caused by an angle error of 45 deg (max error):
    // I_q_hf_max = V_inj * (Lq - Ld) / (2 * W_h * Ld * Lq)
    parameter_gt w_h = CTL_PARAM_CONST_2PI * f_inj_hz;
    parameter_gt L_diff = motor->Lq - motor->Ld;

    if (L_diff < 1e-6f)
        L_diff = 1e-6f; // Prevent division by zero if erroneously used on SPM

    parameter_gt I_q_hf_max_phy = (v_inj_v * L_diff) / (2.0f * w_h * motor->Ld * motor->Lq);

    // Convert to PU
    parameter_gt I_q_hf_max_pu = I_q_hf_max_phy / pu->I_base;

    // We want the demodulated error to have a peak magnitude of roughly 1.0.
    // The demodulation process (multiplying by sin and filtering) halves the amplitude.
    parameter_gt expected_dc_peak = I_q_hf_max_pu * 0.5f;
    bare_init.err_gain_sf = 1.0f / expected_dc_peak;

    // Digital Delay Compensation (Empirical / System Dependent)
    // Typically, PWM update + ADC sampling + digital filtering introduces roughly 1.5 ~ 2.0
    // sample periods of delay at the carrier frequency.
    parameter_gt T_sample = 1.0f / fs;
    bare_init.delay_comp_rad = w_h * (1.5f * T_sample); // Auto-estimated phase lag

    // Invoke core initialization
    ctl_init_pmsm_hfi(hfi, &bare_init);
}
