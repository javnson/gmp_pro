/**
 * @file ctl_dp_single_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for single-phase digital power controller modules.
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for single-phase digital power components,
 * including the Single-Phase PLL, H-Bridge Modulation, and PFC controllers.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Single Phase PLL
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/spll.h>

void ctl_init_single_phase_pll(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt Ti, parameter_gt fc,
                               parameter_gt fg, parameter_gt fs)
{
    // Clear the SPLL structure to ensure a clean state.
    ctl_clear_single_phase_pll(spll);

    // Initialize a discrete SOGI for generating alpha-beta orthogonal signals.
    ctl_init_discrete_sogi(&spll->sogi, 0.5, fg, fs);

    // Initialize a low-pass filter for the q-axis component (Uq) of the SOGI output.
    ctl_init_lp_filter(&spll->filter_uq, fs, fc);

    // Initialize the PI controller for the phase-locking loop.
    ctl_init_pid_Tmode(&spll->spll_ctrl, gain, Ti, 0, fs);

    // Pre-calculate the normalized grid frequency as a feed-forward term for the VCO.
    spll->freq_sf = float2ctrl(fg / fs);
}

void ctl_init_single_phase_pll_T(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt ki, parameter_gt fc,
                               parameter_gt fg, parameter_gt fs)
{
    // Clear the SPLL structure to ensure a clean state.
    ctl_clear_single_phase_pll(spll);

    // Initialize a discrete SOGI for generating alpha-beta orthogonal signals.
    ctl_init_discrete_sogi(&spll->sogi, 0.5, fg, fs);

    // Initialize a low-pass filter for the q-axis component (Uq) of the SOGI output.
    ctl_init_lp_filter(&spll->filter_uq, fs, fc);

    // Initialize the PI controller for the phase-locking loop.
    ctl_init_pid(&spll->spll_ctrl, gain, ki, 0, fs);

    // Pre-calculate the normalized grid frequency as a feed-forward term for the VCO.
    spll->freq_sf = float2ctrl(fg / fs);
}

void ctl_init_single_phase_dc_pll(ctl_single_phase_dc_pll* spll, parameter_gt loop_kp, parameter_gt loop_ki,
                                  parameter_gt k_sogi, parameter_gt k_dc, parameter_gt fc_uq, parameter_gt fg,
                                  parameter_gt fs)
{
    // Clear states
    ctl_clear_single_phase_dc_pll(spll);

    // 1. Init SOGI-DC
    // Note: k_damp is usually 1.414. k_dc is usually 0.5 to 1.0.
    ctl_init_discrete_sogi_dc(&spll->sogi_dc, k_sogi, k_dc, fg, fs);

    // 2. Init Loop Filter (PI)
    ctl_init_pid(&spll->spll_ctrl, loop_kp, loop_ki, 0, fs);

    // 3. Init Q-axis LPF
    ctl_init_lp_filter(&spll->filter_uq, fs, fc_uq);

    // 4. Init Scaling Factor
    // Step = Fg / Fs
    spll->freq_sf = float2ctrl(fg / fs);
}


//////////////////////////////////////////////////////////////////////////
// Single Phase Modulation
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/sp_modulation.h>

void ctl_init_single_phase_H_modulation(single_phase_H_modulation_t* bridge, pwm_gt pwm_full_scale, pwm_gt pwm_deadband,
                                        ctrl_gt current_deadband)
{
    bridge->pwm_full_scale = pwm_full_scale;
    // The deadband value is typically applied symmetrically, so we store half of it.
    bridge->pwm_deadband = pwm_deadband / 2;
    bridge->current_deadband = current_deadband;

    ctl_clear_single_phase_H_modulation(bridge);
}

//////////////////////////////////////////////////////////////////////////
// Single Phase PFC Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/single_phase/spfc.h>

void ctl_init_spfc_ctrl(spfc_t* pfc, parameter_gt voltage_kp, parameter_gt voltage_Ti, parameter_gt voltage_Td,
                        parameter_gt current_kp, parameter_gt current_Ti, parameter_gt current_Td, parameter_gt fs)
{
    // Initialize the series-form PID for the outer voltage control loop.
    ctl_init_pid_Tmode(&pfc->voltage_ctrl, voltage_kp, voltage_Ti, voltage_Td, fs);
    // Initialize the series-form PID for the inner current control loop.
    ctl_init_pid_Tmode(&pfc->current_ctrl, current_kp, current_Ti, current_Td, fs);
}

