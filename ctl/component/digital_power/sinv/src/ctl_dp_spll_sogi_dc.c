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
#include <ctl/component/digital_power/sinv/spll_sogi_dc.h>

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
