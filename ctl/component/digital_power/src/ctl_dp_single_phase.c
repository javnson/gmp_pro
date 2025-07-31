/**
 * @file ctl_common_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2025-06-21
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Single Phase PLL
#include <ctl/component/digital_power/single_phase/spll.h>

void ctl_init_single_phase_pll(
    // handle of Single phase PLL object
    ctl_single_phase_pll *spll,
    // gain of SPLL module
    parameter_gt gain,
    // integrate time
    parameter_gt Ti,
    // filter cut frequency
    parameter_gt fc,
    // Target frequency, grid frequency, Hz
    parameter_gt fg,
    // isr frequency, Hz
    parameter_gt fs)
{
    // clear single phase PLL objects
    ctl_clear_single_phase_pll(spll);

    // init a discrete SOGI object
    ctl_init_discrete_sogi(&spll->sogi, 0.5, fg, fs);

    // init filter object
    ctl_init_lp_filter(&spll->filter_uq, fs, fc);

    // init PID controller
    ctl_init_pid(&spll->spll_ctrl, gain, Ti, 0, fs);

    // frequency parameters
    spll->frequency_sf = float2ctrl(fg / fs);

    //// gain of SPLL
    // spll->pll_gain = float2ctrl(gain);
}

//////////////////////////////////////////////////////////////////////////
// Single Phase PLL
#include <ctl/component/digital_power/single_phase/sp_modulation.h>

void ctl_init_single_phase_H_modulation(
    // handle of modulation object
    single_phase_H_modulation_t *bridge,
    // PWM depth
    pwm_gt pwm_full_scale,
    // PWM dead band real value
    pwm_gt pwm_deadband,
    // current dead band
    ctrl_gt current_deadband)
{
    bridge->pwm_full_scale = pwm_full_scale;
    bridge->pwm_deadband = pwm_deadband / 2;
    bridge->current_deadband = current_deadband;

    ctl_clear_single_phase_H_modulation(bridge);
}

//////////////////////////////////////////////////////////////////////////
// Single Phase PLL
#include <ctl/component/digital_power/single_phase/spfc.h>

void ctl_init_spfc_ctrl(
    // handle of PFC controller
    spfc_t *pfc,
    // voltage controller parameters
    parameter_gt voltage_kp, parameter_gt voltage_Ti, parameter_gt voltage_Td,
    // current controller parameters
    parameter_gt current_kp, parameter_gt current_Ti, parameter_gt current_Td,
    // controller frequency
    parameter_gt fs)
{
    ctl_init_pid_ser(&pfc->voltage_ctrl, voltage_kp, voltage_Ti, voltage_Td, fs);
    ctl_init_pid_ser(&pfc->current_ctrl, current_kp, current_Ti, current_Td, fs);
}
