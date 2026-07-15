/** @file dcdc_clllc_hw.c @brief Conservative CLLLC/DAB PI auto tuning. */
#include <gmp_core.h>
#include <ctl/component/digital_power/dcdc/clllc.h>

static parameter_gt clllc_limit_fc(parameter_gt requested, parameter_gt limit)
{
    if (requested <= 0.0f || requested > limit)
        return limit;
    return requested;
}

void ctl_dcdc_blueprint_clllc_parallel(ctl_dcdc_core_init_t* init_config,
                                       const ctl_clllc_hardware_t* hw)
{
    parameter_gt lr_eq;
    parameter_gt fc_i;
    parameter_gt fc_v;
    parameter_gt req_reflected;

    gmp_base_assert(init_config != NULL && hw != NULL);
    gmp_base_assert(hw->fs > 0.0f && hw->f_res_hz > 0.0f);
    gmp_base_assert(hw->lr_primary_h > 0.0f && hw->c_out_f > 0.0f);
    gmp_base_assert(hw->v_base > 0.0f && hw->i_base > 0.0f);

    lr_eq = hw->lr_primary_h + hw->lr_secondary_h /
            (hw->transformer_ratio * hw->transformer_ratio);
    req_reflected = hw->r_load_min_ohm /
                    (hw->transformer_ratio * hw->transformer_ratio);

    /* The current loop must stay well below both resonance and the sampled
       Nyquist limit.  The voltage loop is kept at least one decade slower. */
    fc_i = clllc_limit_fc(hw->fc_current_loop,
                         (hw->f_res_hz * 0.1f < hw->fs * 0.1f) ?
                         hw->f_res_hz * 0.1f : hw->fs * 0.1f);
    fc_v = clllc_limit_fc(hw->fc_voltage_loop, fc_i * 0.1f);

    init_config->fs = hw->fs;
    init_config->fc_v_in = clllc_limit_fc(fc_i * 5.0f, hw->fs * 0.45f);
    init_config->fc_v_out = clllc_limit_fc(fc_v * 10.0f, hw->fs * 0.45f);
    init_config->fc_i_L = clllc_limit_fc(fc_i * 5.0f, hw->fs * 0.45f);
    init_config->fc_i_load = init_config->fc_i_L;
    init_config->slope_v_pu_s = hw->slope_v_pu_s;
    init_config->slope_i_pu_s = hw->slope_i_pu_s;

    /* Around resonance, the FHA small-signal tank plant is dominated by the
       total leakage inductance.  ESR plus reflected load places the PI zero. */
    init_config->i_kp = CTL_PARAM_CONST_2PI * fc_i * lr_eq *
                        (hw->i_base / hw->v_base);
    init_config->i_ki = (hw->tank_esr_ohm + req_reflected) / lr_eq;
    init_config->i_kd = 0.0f;
    init_config->i_out_max = hw->modulation_max;
    init_config->i_out_min = hw->modulation_min;

    /* Output capacitor plant: dv/dt = i/C. */
    init_config->v_kp = CTL_PARAM_CONST_2PI * fc_v * hw->c_out_f *
                        (hw->v_base / hw->i_base);
    init_config->v_ki = 1.0f / (hw->r_load_min_ohm * hw->c_out_f);
    init_config->v_kd = 0.0f;
    init_config->v_out_max = hw->i_limit_max;
    init_config->v_out_min = hw->i_limit_min;
}
