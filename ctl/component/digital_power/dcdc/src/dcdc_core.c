#include <gmp_core.h>

#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>

/**
 * @brief Helper function to compute safe cutoff frequencies with protection limits.
 */
static parameter_gt ctl_dcdc_get_safe_fc(parameter_gt fs, parameter_gt fc)
{
    parameter_gt nyquist = fs * 0.5f;

    if (fc <= 0.0f)
    {
        return nyquist * 0.9f; /* Fallback to safe high bandwidth */
    }
    if (fc >= nyquist)
    {
        return nyquist * 0.95f; /* Clamp below Nyquist limit */
    }
    return fc;
}

/**
 * @brief Initializes the universal DCDC core based on offline user configuration.
 * @param[out] core Pointer to the DCDC core instance.
 * @param[in] init_config Pointer to the parameter configuration package.
 */
void ctl_init_dcdc_core(ctl_dcdc_core_t* core, const ctl_dcdc_core_init_t* init_config)
{
    gmp_base_assert(core != NULL);
    gmp_base_assert(init_config != NULL);
    gmp_base_assert(init_config->fs > 0.0f);

    /* 1. Global State Architecture Setup */
    core->mode = CTL_DCDC_MODE_OPENLOOP;
    core->v_target = float2ctrl(0.0f);
    core->i_target = float2ctrl(0.0f);
    core->out_max = float2ctrl(1.0f);
    core->out_min = float2ctrl(0.0f);

    /* 2. Adaptive Filtering Configuration with Safety Bounds Check */
    parameter_gt safe_fc_v_in = ctl_dcdc_get_safe_fc(init_config->fs, init_config->fc_v_in);
    parameter_gt safe_fc_v_out = ctl_dcdc_get_safe_fc(init_config->fs, init_config->fc_v_out);
    parameter_gt safe_fc_i_L = ctl_dcdc_get_safe_fc(init_config->fs, init_config->fc_i_L);
    parameter_gt safe_fc_i_load = ctl_dcdc_get_safe_fc(init_config->fs, init_config->fc_i_load);

    /* Standard LPF (Bilinear) for smooth DC voltages */
    ctl_init_filter_iir1_lpf(&core->filter_v_in, init_config->fs, safe_fc_v_in);
    ctl_init_filter_iir1_lpf(&core->filter_v_out, init_config->fs, safe_fc_v_out);

    /* Optimized 1st-Order Lag Filter for ripple/hysteresis current channels (No Nyquist Traps) */
    ctl_init_filter_iir1_lag(&core->filter_i_L, init_config->fs, safe_fc_i_L);
    ctl_init_filter_iir1_lag(&core->filter_i_load, init_config->fs, safe_fc_i_load);

    /* 3. Slew-Rate Limiter Initialization (PU/s) */
    ctl_init_slope_limiter(&core->ramp_v, init_config->slope_v_pu_s, -init_config->slope_v_pu_s, init_config->fs);
    ctl_init_slope_limiter(&core->ramp_i, init_config->slope_i_pu_s, -init_config->slope_i_pu_s, init_config->fs);

    /* 4. Series-Form Linear Compensators Configuration */
    /* Strict clamping for outer loop integrator to prevent overshoots */
    ctl_init_pid(&core->voltage_pid, init_config->v_kp, init_config->v_ki, init_config->v_kd, init_config->fs);
    ctl_set_pid_limit(&core->voltage_pid, init_config->v_out_max, init_config->v_out_min);
    ctl_set_pid_int_limit(&core->voltage_pid, init_config->v_out_max, init_config->v_out_min);

    /* Supports asymmetric current bounds (e.g., heavy motoring vs light regeneration boundaries) */
    ctl_init_pid(&core->current_pid, init_config->i_kp, init_config->i_ki, init_config->i_kd, init_config->fs);
    ctl_set_pid_limit(&core->current_pid, init_config->i_out_max, init_config->i_out_min);
    ctl_set_pid_int_limit(&core->current_pid, init_config->i_out_max, init_config->i_out_min);

    /* 5. Zero-crossing Current Direction Non-linear Bound */
    ctl_init_hysteresis_controller(&core->i_dir_hcc, 1, float2ctrl(0.01f)); /* 1% PU band */
    ctl_set_hysteresis_target(&core->i_dir_hcc, float2ctrl(0.0f));

    /* 6. Enforce cold state clean slate initialization */
    ctl_clear_dcdc_core(core);
}

