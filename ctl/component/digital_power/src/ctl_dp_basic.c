/**
 * @file ctl_dp_basic.c
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implementation file for basic digital power controller modules.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file contains the function definitions for initializing and
 * attaching interfaces for various digital power controllers, including Buck,
 * Boost, and protection modules.
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Buck Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/buck.h>

void ctl_init_buck_ctrl(buck_ctrl_t* buck, parameter_gt v_kp, parameter_gt v_Ti, parameter_gt v_Td, parameter_gt i_kp,
                        parameter_gt i_Ti, parameter_gt i_Td, parameter_gt uin_min, parameter_gt uin_max,
                        parameter_gt fc, parameter_gt fs)
{
    // Ensure controller is disabled on startup
    ctl_disable_buck_ctrl(buck);

    // Initialize low-pass filters for all sensor inputs
    ctl_init_lp_filter(&buck->lpf_il, fs, fc);
    ctl_init_lp_filter(&buck->lpf_ui, fs, fc);
    ctl_init_lp_filter(&buck->lpf_uo, fs, fc);

    // Initialize saturation block for input voltage to prevent division by zero
    ctl_init_saturation(&buck->modulation_saturation, uin_min, uin_max);

    // Initialize PID controllers
    ctl_init_pid_Tmode(&buck->current_pid, i_kp, i_Ti, i_Td, fs);
    ctl_init_pid_Tmode(&buck->voltage_pid, v_kp, v_Ti, v_Td, fs);

    // Clear all internal states
    ctl_clear_buck_ctrl(buck);
}

void ctl_attach_buck_ctrl_input(buck_ctrl_t* buck, adc_ift* uo, adc_ift* il, adc_ift* uin)
{
    buck->adc_il = il;
    buck->adc_ui = uin;
    buck->adc_uo = uo;
}

//////////////////////////////////////////////////////////////////////////
// Boost Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/boost.h>

void ctl_init_boost_ctrl(boost_ctrl_t* boost, parameter_gt v_kp, parameter_gt v_Ti, parameter_gt v_Td,
                         parameter_gt i_kp, parameter_gt i_Ti, parameter_gt i_Td, parameter_gt vo_min,
                         parameter_gt vo_max, parameter_gt fc, parameter_gt fs)
{
    // Ensure controller is disabled on startup
    ctl_disable_boost_ctrl(boost);

    // Initialize PID controllers with correct parameters
    ctl_init_pid_Tmode(&boost->current_pid, i_kp, i_Ti, i_Td, fs);
    ctl_init_pid_Tmode(&boost->voltage_pid, v_kp, v_Ti, v_Td, fs);

    // Initialize low-pass filters for all sensor inputs
    ctl_init_lp_filter(&boost->lpf_il, fs, fc);
    ctl_init_lp_filter(&boost->lpf_ui, fs, fc);
    ctl_init_lp_filter(&boost->lpf_uo, fs, fc);

    // Initialize saturation block for output voltage command
    ctl_init_saturation(&boost->modulation_saturation, vo_min, vo_max);

    // Clear all internal states
    ctl_clear_boost_ctrl(boost);

    // clear open state
    boost->flag_enable_system = 0;
}

void ctl_attach_boost_ctrl_input(boost_ctrl_t* boost, adc_ift* uc_port, adc_ift* il_port, adc_ift* uin_port)
{
    boost->adc_uo = uc_port;
    boost->adc_il = il_port;
    boost->adc_ui = uin_port;
}

//////////////////////////////////////////////////////////////////////////
// Protection Strategy
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/protectoion_strategy.h>

void ctl_attach_vip_protection(std_vip_protection_t* obj, adc_ift* uo, adc_ift* io)
{
    obj->adc_io = io;
    obj->adc_uo = uo;
}

void ctl_init_vip_protection(std_vip_protection_t* obj, parameter_gt power_f_cut, parameter_gt voltage_f_cut,
                             parameter_gt current_f_cut, parameter_gt v_max, parameter_gt v_base, parameter_gt i_max,
                             parameter_gt i_base, parameter_gt p_max, parameter_gt fs)
{
    // Set protection thresholds in per-unit values
    obj->voltage_max = float2ctrl(v_max / v_base);
    obj->current_max = float2ctrl(i_max / i_base);
    obj->power_max = float2ctrl(p_max / v_base / i_base);

    // Initialize low-pass filters for measurements
    ctl_init_lp_filter(&obj->power_filter, fs, power_f_cut);
    ctl_init_lp_filter(&obj->voltage_filter, fs, voltage_f_cut);
    ctl_init_lp_filter(&obj->current_filter, fs, current_f_cut);
}

void ctl_init_foldback_protection(std_foldback_protection_t* obj, parameter_gt voltage_f_cut,
                                  parameter_gt current_f_cut, parameter_gt v_rated, parameter_gt i_max,
                                  parameter_gt i_short, parameter_gt fs)
{
    // Initialize all members to a known state
    memset(obj, 0, sizeof(std_foldback_protection_t));

    // Store protection parameters
    obj->voltage_rated = v_rated;
    obj->current_max = i_max;
    obj->current_short_circuit = i_short;

    // The initial output should be the maximum allowed current
    obj->current_limit_output = i_max;

    // Pre-calculate the slope of the foldback line for efficiency.
    // Slope = (delta_I) / (delta_V) = (i_max - i_short) / (v_rated - 0)
    if (v_rated > 0)
    {
        obj->slope = ctl_div(obj->current_max - obj->current_short_circuit, obj->voltage_rated);
    }

    // Initialize the low-pass filters for input signal conditioning
    ctl_init_lp_filter(&obj->voltage_filter, voltage_f_cut, fs);
    ctl_init_lp_filter(&obj->current_filter, current_f_cut, fs);
}

void ctl_attach_foldback_protection(std_foldback_protection_t* obj, adc_ift* uo, adc_ift* io)
{
    obj->adc_uo = uo;
    obj->adc_io = io;
}

//////////////////////////////////////////////////////////////////////////
// Protection Strategy
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/virtual_imp.h>

/**
 * @brief Helper to configure Biquad as a Band-Limited Differentiator.
 * TF: H(s) = K * s / (s + wc)
 * where K = L * wc
 */
static void _ctl_init_biquad_differentiator(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt L, parameter_gt fc)
{
    parameter_gt wc = CTL_PARAM_CONST_2PI * fc;
    parameter_gt K = L * wc; // Gain factor

    // Tustin Transform: s = 2*fs * (1-z^-1)/(1+z^-1)
    // H(z) = K * [2fs(1-z^-1)/(1+z^-1)] / [2fs(1-z^-1)/(1+z^-1) + wc]
    // Simplifying...

    parameter_gt k_tustin = 2.0f * fs;
    parameter_gt D0 = k_tustin + wc;

    // Coefficients for standard Direct Form I Biquad
    // y[n] = b0*x[n] + b1*x[n-1] + ... - a1*y[n-1] ...

    obj->b[0] = (K * k_tustin) / D0;
    obj->b[1] = -(K * k_tustin) / D0; // Note the negative sign
    obj->b[2] = 0.0f;

    // Denominator a1, a2
    // Denom(z) = (k + wc) + (wc - k)z^-1
    // a1 = (wc - k) / (k + wc)
    obj->a[0] = (wc - k_tustin) / D0;
    obj->a[1] = 0.0f;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_vir_imp(vir_imp_t* imp, const parameter_gt R_vir, const parameter_gt L_vir, parameter_gt fs);
{
    gmp_base_assert(imp);

    // 1. Resistive Gain
    imp->gain_R = > R_vir;

    // 2. Inductive Differentiator
    if (fabsf(L_vir) > 1e-12f) // If L is non-zero
    {
        _ctl_init_biquad_differentiator(&imp->diff_filter, fs, L_vir, fs / 10);
    }
    else
    {
        // Zero inductance -> Pass 0
        // Set all coeffs to 0
        ctl_clear_biquad_filter(&imp->diff_filter);
        imp->diff_filter.b[0] = 0;
        imp->diff_filter.b[1] = 0;
        imp->diff_filter.b[2] = 0;
        imp->diff_filter.a[0] = 0;
        imp->diff_filter.a[1] = 0;
    }

    imp->out = 0.0f;
}
