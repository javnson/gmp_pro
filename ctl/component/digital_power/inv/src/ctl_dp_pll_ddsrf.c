/**
 * @file ctl_dp_three_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for three-phase digital power controller modules.
 * @version 1.05
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for three-phase digital power components,
 * including the Three-Phase PLL and bridge modulation modules.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Three Phase PLL

#include <ctl/component/digital_power/inv/pll_ddsrf.h>

void ctl_init_ddsrf_pll(ddsrf_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_ki,
                        parameter_gt f_ctrl, parameter_gt decoupling_fc)
{
    // Clear State
    ctl_clear_ddsrf_pll(pll);

    // Init Frequency Scaling Factor
    pll->freq_sf = float2ctrl(f_base / f_ctrl);

    // Init PID
    ctl_init_pid(&pll->pid_pll, pid_kp, pid_ki, 0, f_ctrl);

    // Init Decoupling LPFs
    ctl_init_filter_iir1_lpf(&pll->lpf_pos_d, f_ctrl, decoupling_fc);
    ctl_init_filter_iir1_lpf(&pll->lpf_pos_q, f_ctrl, decoupling_fc);
    ctl_init_filter_iir1_lpf(&pll->lpf_neg_d, f_ctrl, decoupling_fc);
    ctl_init_filter_iir1_lpf(&pll->lpf_neg_q, f_ctrl, decoupling_fc);
}

/**
 * @brief Auto-tune and initialize the DDSRF-PLL.
 * @details Bandwidth usually 20-30Hz. Decoupling FC usually f_base/sqrt(2).
 */
void ctl_init_ddsrf_pll_auto_tune(ddsrf_pll_t* pll, parameter_gt f_base, parameter_gt f_ctrl, parameter_gt voltage_mag,
                                  parameter_gt bandwidth_hz)
{
    // 1. Calculate Loop Gains (Same as SRF-PLL)
    parameter_gt omega_n = CTL_PARAM_CONST_2PI * bandwidth_hz;
    parameter_gt loop_gain_constant = CTL_PARAM_CONST_2PI * voltage_mag * f_base;

    if (loop_gain_constant < 0.0001f)
        loop_gain_constant = 0.0001f;

    parameter_gt damping = 0.707f;
    parameter_gt kp = (2.0f * damping * omega_n) / loop_gain_constant;
    parameter_gt ki = (omega_n * omega_n) / loop_gain_constant;

    // 2. Set Decoupling Bandwidth
    // Optimal is usually freq_grid / sqrt(2)
    parameter_gt decoupling_fc = f_base / 1.414f;

    // 3. Initialize
    ctl_init_ddsrf_pll(pll, f_base, kp, ki, f_ctrl, decoupling_fc);
}
