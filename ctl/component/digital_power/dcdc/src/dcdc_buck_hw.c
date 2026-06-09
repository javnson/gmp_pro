/**
 * @file dcdc_buck_hw.c
 * @brief Analytical pole-placement tuning calculator incorporating resistive loading and Nyquist bypass bounds.
 */

#include <gmp_core.h>

#include <ctl/component/digital_power/dcdc/buck.h>

/**
 * @brief Clamps cutoff frequency below Nyquist limit, or forces pure bypass if limits exceeded.
 */
static parameter_gt ctl_buck_calc_safe_fc(parameter_gt fs, parameter_gt target_fc)
{
    parameter_gt nyquist = fs * 0.5f;

    if (target_fc <= 0.0f)
    {
        return nyquist * 0.9f;
    }
    /* If the requested filter bandwidth is too high, set to fs to invoke pure pass-through mode safely */
    if (target_fc >= nyquist)
    {
        return fs;
    }
    return target_fc;
}

void ctl_dcdc_blueprint_buck_cascade(ctl_dcdc_core_init_t* init_config, const ctl_buck_hardware_t* hw)
{
    gmp_base_assert(init_config != NULL);
    gmp_base_assert(hw != NULL);
    gmp_base_assert(hw->fs > 0.0f);
    gmp_base_assert(hw->v_base > 0.0f && hw->i_base > 0.0f);

    /* 1. Directly pipe system execution frequency */
    init_config->fs = hw->fs;

    /* 2. Configure Signal Filters with Nyquist Protection & Bypass Rules */
    init_config->fc_v_in = ctl_buck_calc_safe_fc(hw->fs, hw->fc_voltage_loop * 20.0f);
    init_config->fc_v_out = ctl_buck_calc_safe_fc(hw->fs, hw->fc_voltage_loop * 15.0f);
    init_config->fc_i_L = ctl_buck_calc_safe_fc(hw->fs, hw->fc_current_loop * 10.0f);
    init_config->fc_i_load = ctl_buck_calc_safe_fc(hw->fs, hw->fc_current_loop * 5.0f);

    /* 3. Extract Profile Slew-Rates from Configuration Boundaries */
    init_config->slope_v_pu_s = hw->slope_v_pu_s;
    init_config->slope_i_pu_s = hw->slope_i_pu_s;

    /* ===================================================================== */
    /* 4. Inner Current Loop Analytical Tuning                               */
    /* ===================================================================== */
    /* Continuous plant cancelation: Kp = 2 * pi * fc * L / Vin_nominal */
    parameter_gt raw_i_kp = (CTL_PARAM_CONST_2PI * hw->fc_current_loop * hw->L_henry) / hw->v_in_nominal;

    /* Transform to digital Per-Unit gains */
    init_config->i_kp = raw_i_kp * (hw->i_base / hw->v_base);

    /* Series zero-pole compensation: Ki = Kp / Ti , where Ti = L / R_esr */
    init_config->i_ki = init_config->i_kp * (hw->R_esr_ohm / hw->L_henry);
    init_config->i_kd = 0.0f;

    /* Enforce explicit user current constraints */
    init_config->i_out_max = hw->i_out_max;
    init_config->i_out_min = hw->i_out_min;

    /* ===================================================================== */
    /* 5. Outer Voltage Loop Analytical Tuning (Resistive Loaded Model)      */
    /* ===================================================================== */
    /* Plant: G_vp(s) = R_load / (R_load * C * s + 1) */
    /* Continuous Voltage Loop Kp: Kp = 2 * pi * fc * C */
    parameter_gt raw_v_kp = CTL_PARAM_CONST_2PI * hw->fc_voltage_loop * hw->C_farad;

    /* Transform to digital Per-Unit gains */
    init_config->v_kp = raw_v_kp * (hw->v_base / hw->i_base);

    /* Series compensation matching load dynamics: Ki = Kp / Tv, where Tv = R_load * C */
    if (hw->R_load_nominal > 0.0f)
    {
        init_config->v_ki = init_config->v_kp / (hw->R_load_nominal * hw->C_farad);
    }
    else
    {
        /* Fallback to legacy capacitive pure integrator tracking if load is unspecified */
        init_config->v_ki = init_config->v_kp * (hw->fc_voltage_loop * 0.5f);
    }
    init_config->v_kd = 0.0f;

    /* Outer loop output directly limits the inner loop command headroom */
    init_config->v_out_max = hw->i_out_max;
    init_config->v_out_min = hw->i_out_min;
}
