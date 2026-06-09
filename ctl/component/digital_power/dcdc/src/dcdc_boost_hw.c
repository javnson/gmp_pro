/**
 * @file dcdc_boost_hw.c
 * @brief Analytical pole-placement tuning calculator incorporating resistive loading and Nyquist bypass bounds.
 */

#include <gmp_core.h>

#include <ctl/component/digital_power/dcdc/boost.h>

/**
 * @brief Clamps cutoff frequency below Nyquist limit, or forces pure bypass if limits exceeded.
 */
static parameter_gt ctl_boost_calc_safe_fc(parameter_gt fs, parameter_gt target_fc)
{
    parameter_gt nyquist = fs * 0.5f;

    if (target_fc <= 0.0f)
    {
        return nyquist * 0.9f;
    }
    if (target_fc >= nyquist)
    {
        return fs;
    }
    return target_fc;
}

parameter_gt ctl_boost_calc_rhp_zero(const ctl_boost_hardware_t* hw)
{
    gmp_base_assert(hw != NULL);
    gmp_base_assert(hw->v_out_nominal > 0.0f);
    gmp_base_assert(hw->L_henry > 0.0f);
    gmp_base_assert(hw->R_load_min > 0.0f);

    /* Compute maximum steady-state duty cycle D under worst-case minimum input voltage */
    /* V_in_min = (1 - D) * V_out_nominal -> (1 - D) = V_in_min / V_out_nominal */
    parameter_gt complex_d_prime = hw->v_in_min / hw->v_out_nominal;
    if (complex_d_prime > 1.0f)
    {
        complex_d_prime = 1.0f; /* Guard against fractional buck actions */
    }

    /* f_rhpz = (R_load * (1 - D)^2) / (2 * pi * L) */
    parameter_gt f_rhpz = (hw->R_load_min * complex_d_prime * complex_d_prime) / (CTL_PARAM_CONST_2PI * hw->L_henry);
    return f_rhpz;
}

void ctl_dcdc_blueprint_boost_cascade(ctl_dcdc_core_init_t* init_config, const ctl_boost_hardware_t* hw)
{
    gmp_base_assert(init_config != NULL);
    gmp_base_assert(hw != NULL);
    gmp_base_assert(hw->fs > 0.0f);
    gmp_base_assert(hw->v_base > 0.0f && hw->i_base > 0.0f);

    /* 1. Directly pipe system execution frequency */
    init_config->fs = hw->fs;

    /* 2. Analyze Right Half Plane Zero boundary constraints */
    parameter_gt worst_case_f_rhpz = ctl_boost_calc_rhp_zero(hw);

    /* Enforce safe bandwidth limit rule of thumb: fc_v <= f_rhpz / 5 */
    parameter_gt max_safe_voltage_bandwidth = worst_case_f_rhpz * 0.2f;
    parameter_gt actual_fc_v = hw->fc_voltage_loop;

    /* Automatic active safety rollback clamp */
    if (actual_fc_v > max_safe_voltage_bandwidth)
    {
        actual_fc_v = max_safe_voltage_bandwidth;
    }

    /* 3. Configure Signal Filters based on the validated loop dynamics */
    init_config->fc_v_in = ctl_boost_calc_safe_fc(hw->fs, actual_fc_v * 20.0f);
    init_config->fc_v_out = ctl_boost_calc_safe_fc(hw->fs, actual_fc_v * 15.0f);
    init_config->fc_i_L = ctl_boost_calc_safe_fc(hw->fs, hw->fc_current_loop * 10.0f);
    init_config->fc_i_load = ctl_boost_calc_safe_fc(hw->fs, hw->fc_current_loop * 5.0f);

    /* 4. Extract Profile Slew-Rates from Configuration Boundaries */
    init_config->slope_v_pu_s = hw->slope_v_pu_s;
    init_config->slope_i_pu_s = hw->slope_i_pu_s;

    /* ===================================================================== */
    /* 5. Inner Current Loop Analytical Tuning                               */
    /* ===================================================================== */
    /* Boost Current Plant Gain depends on the output voltage:               */
    /* G_id(s) = V_out / (L*s + R_esr)                                       */
    parameter_gt raw_i_kp = (CTL_PARAM_CONST_2PI * hw->fc_current_loop * hw->L_henry) / hw->v_out_nominal;

    /* Transform to digital Per-Unit gains */
    init_config->i_kp = raw_i_kp * (hw->i_base / hw->v_base);

    /* Series zero-pole compensation: Ki = Kp / Ti , where Ti = L / R_esr */
    init_config->i_ki = init_config->i_kp * (hw->R_esr_ohm / hw->L_henry);
    init_config->i_kd = 0.0f;

    /* Enforce explicit user current constraints */
    init_config->i_out_max = hw->i_out_max;
    init_config->i_out_min = hw->i_out_min;

    /* ===================================================================== */
    /* 6. Outer Voltage Loop Analytical Tuning (RHPZ Restricted Model)       */
    /* ===================================================================== */
    /* Target continuous voltage loop Kp: Kp = 2 * pi * fc * C * (V_out/V_in)*/
    /* Accommodates topology conversion boost ratio gain scaling             */
    parameter_gt boost_gain_ratio = hw->v_out_nominal / hw->v_in_min;
    parameter_gt raw_v_kp = CTL_PARAM_CONST_2PI * actual_fc_v * hw->C_farad * boost_gain_ratio;

    /* Transform to digital Per-Unit gains */
    init_config->v_kp = raw_v_kp * (hw->v_base / hw->i_base);

    /* Series compensation matching resistive load pole: Tv = R_load * C */
    if (hw->R_load_min > 0.0f)
    {
        init_config->v_ki = init_config->v_kp / (hw->R_load_min * hw->C_farad);
    }
    else
    {
        /* Fallback conservative integration constant if load is undetermined */
        init_config->v_ki = init_config->v_kp * (actual_fc_v * 0.5f);
    }
    init_config->v_kd = 0.0f;

    /* Outer loop output bounds match inner current amplifier boundaries */
    init_config->v_out_max = hw->i_out_max;
    init_config->v_out_min = hw->i_out_min;
}
