/**
 * @file ctl_dcdc_blueprint_4sbb.c
 * @brief Unified loop tuning calculator for 4-Switch Buck-Boost topologies with linear modulation.
 */

#include <gmp_core.h>

#include <ctl/component/digital_power/dcdc/fsbb.h>

/**
 * @brief Clamps cutoff frequency below Nyquist limit, or forces pure bypass if limits exceeded.
 */
static parameter_gt ctl_fsbb_calc_safe_fc(parameter_gt fs, parameter_gt target_fc)
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

parameter_gt ctl_fsbb_calc_worst_rhp_zero(const ctl_4switch_buckboost_hardware_t* hw)
{
    gmp_base_assert(hw != NULL);
    gmp_base_assert(hw->v_in_min > 0.0f);
    gmp_base_assert(hw->v_out_max > 0.0f);
    gmp_base_assert(hw->L_henry > 0.0f);
    gmp_base_assert(hw->R_load_min > 0.0f);

    /* Worst-case RHPZ happens at minimum input voltage and maximum output voltage (deepest Boost mode) */
    parameter_gt max_boost_d_prime = hw->v_in_min / hw->v_out_max;
    if (max_boost_d_prime > 1.0f)
    {
        max_boost_d_prime = 1.0f;
    }

    /* f_rhpz_min = (R_load_min * (D_prime_min)^2) / (2 * pi * L) */
    parameter_gt worst_f_rhpz =
        (hw->R_load_min * max_boost_d_prime * max_boost_d_prime) / (CTL_PARAM_CONST_2PI * hw->L_henry);
    return worst_f_rhpz;
}

void ctl_dcdc_blueprint_fsbb_cascade(ctl_dcdc_core_init_t* init_config, const ctl_4switch_buckboost_hardware_t* hw)
{
    gmp_base_assert(init_config != NULL);
    gmp_base_assert(hw != NULL);
    gmp_base_assert(hw->fs > 0.0f);
    gmp_base_assert(hw->v_base > 0.0f && hw->i_base > 0.0f);

    /* 1. Map system execution frequency */
    init_config->fs = hw->fs;

    /* 2. Analyze right half plane zero boundary under deepest boost condition */
    parameter_gt limit_f_rhpz = ctl_fsbb_calc_worst_rhp_zero(hw);

    /* Enforce 1/5 RHPZ safety margin for the single voltage controller parameter set */
    parameter_gt max_safe_v_bandwidth = limit_f_rhpz * 0.2f;
    parameter_gt actual_fc_v = hw->fc_voltage_loop;

    if (actual_fc_v > max_safe_v_bandwidth)
    {
        actual_fc_v = max_safe_v_bandwidth; /* Active protection rollback */
    }

    /* 3. Coordinate Signal Filters based on the validated single loop bandwidth */
    init_config->fc_v_in = ctl_fsbb_calc_safe_fc(hw->fs, actual_fc_v * 20.0f);
    init_config->fc_v_out = ctl_fsbb_calc_safe_fc(hw->fs, actual_fc_v * 15.0f);
    init_config->fc_i_L = ctl_fsbb_calc_safe_fc(hw->fs, hw->fc_current_loop * 10.0f);
    init_config->fc_i_load = ctl_fsbb_calc_safe_fc(hw->fs, hw->fc_current_loop * 5.0f);

    /* 4. Assign Global Dynamic Profile Limits */
    init_config->slope_v_pu_s = hw->slope_v_pu_s;
    init_config->slope_i_pu_s = hw->slope_i_pu_s;

    /* ===================================================================== */
    /* 5. Inner Current Loop Universal Tuning                                */
    /* ===================================================================== */
    /* Because the modular linearizes the physical plant gain across modes,  */
    /* the normalized inner current plant gain can be treated as unity.       */
    parameter_gt raw_i_kp = CTL_PARAM_CONST_2PI * hw->fc_current_loop * hw->L_henry;

    /* Transform to digital Per-Unit gains */
    init_config->i_kp = raw_i_kp * (hw->i_base / hw->v_base);

    /* Series zero-pole compensation: Ki = Kp / Ti , where Ti = L / R_esr */
    init_config->i_ki = init_config->i_kp * (hw->R_esr_ohm / hw->L_henry);
    init_config->i_kd = 0.0f;

    /* Enforce hardware protection bounds */
    init_config->i_out_max = hw->i_out_max;
    init_config->i_out_min = hw->i_out_min;

    /* ===================================================================== */
    /* 6. Outer Voltage Loop Universal Tuning (Worst-Case RHPZ Bound)        */
    /* ===================================================================== */
    /* Base plant capacitive charging gain model */
    parameter_gt raw_v_kp = CTL_PARAM_CONST_2PI * actual_fc_v * hw->C_farad;

    /* Adjust gain slightly based on average transfer ratio to ensure robustness in Boost region */
    parameter_gt max_boost_ratio = hw->v_out_max / hw->v_in_min;
    raw_v_kp = raw_v_kp * max_boost_ratio;

    /* Transform to digital Per-Unit gains */
    init_config->v_kp = raw_v_kp * (hw->v_base / hw->i_base);

    /* Series compensation utilizing minimal resistive load boundary: Tv = R_load_min * C */
    if (hw->R_load_min > 0.0f)
    {
        init_config->v_ki = init_config->v_kp / (hw->R_load_min * hw->C_farad);
    }
    else
    {
        init_config->v_ki = init_config->v_kp * (actual_fc_v * 0.5f);
    }
    init_config->v_kd = 0.0f;

    /* Sync voltage loop headroom to current constraints */
    init_config->v_out_max = hw->i_out_max;
    init_config->v_out_min = hw->i_out_min;
}
