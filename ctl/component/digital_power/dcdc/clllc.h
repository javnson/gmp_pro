/**
 * @file clllc.h
 * @brief Bidirectional CLLLC/DAB plant blueprint and four-leg modulator.
 */
#ifndef CTL_DCDC_CLLLC_H
#define CTL_DCDC_CLLLC_H

#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/interface/adv_pwm_channel.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _tag_ctl_clllc_hardware
{
    parameter_gt fs;              /**< Nominal control execution rate (Hz). */
    parameter_gt f_res_hz;        /**< Series resonance (Hz). */
    parameter_gt f_sw_min_hz;
    parameter_gt f_sw_max_hz;
    parameter_gt lm_h;
    parameter_gt lr_primary_h;
    parameter_gt lr_secondary_h;
    parameter_gt cr_primary_f;
    parameter_gt cr_secondary_f;
    parameter_gt transformer_ratio; /**< Ns/Np. */
    parameter_gt c_out_f;
    parameter_gt r_load_min_ohm;
    parameter_gt tank_esr_ohm;
    parameter_gt v_base;
    parameter_gt i_base;
    parameter_gt slope_v_pu_s;
    parameter_gt slope_i_pu_s;
    parameter_gt fc_current_loop;
    parameter_gt fc_voltage_loop;
    ctrl_gt i_limit_max;
    ctrl_gt i_limit_min;
    ctrl_gt modulation_max;
    ctrl_gt modulation_min;
} ctl_clllc_hardware_t;

/** Populate DCDC Core PI/filter settings from the physical resonant tank. */
void ctl_dcdc_blueprint_clllc_parallel(ctl_dcdc_core_init_t* init_config,
                                       const ctl_clllc_hardware_t* hw);

typedef struct _tag_clllc_modulator
{
    adv_pwm_channel_t leg[4]; /**< Primary A/B, then secondary A/B logical legs. */
    parameter_gt timer_clock_hz;
    parameter_gt f_nominal_hz;
    parameter_gt f_min_hz;
    parameter_gt f_max_hz;
    ctrl_gt deadband_pu;
    ctrl_gt max_phase_shift_pu;
} clllc_modulator_t;

GMP_STATIC_INLINE void ctl_init_clllc_modulator(clllc_modulator_t* mod,
                                                 parameter_gt timer_clock_hz,
                                                 parameter_gt f_nominal_hz,
                                                 parameter_gt f_min_hz,
                                                 parameter_gt f_max_hz,
                                                 parameter_gt deadband_s,
                                                 ctrl_gt max_phase_shift_pu)
{
    int i;
    pwm_gt nominal_period = (pwm_gt)(timer_clock_hz / f_nominal_hz);
    for (i = 0; i < 4; ++i)
        ctl_init_adv_pwm_channel(&mod->leg[i], nominal_period);
    mod->timer_clock_hz = timer_clock_hz;
    mod->f_nominal_hz = f_nominal_hz;
    mod->f_min_hz = f_min_hz;
    mod->f_max_hz = f_max_hz;
    mod->deadband_pu = float2ctrl(deadband_s * f_nominal_hz);
    mod->max_phase_shift_pu = max_phase_shift_pu;
}

/**
 * @brief Convert a signed formal command into joint frequency/phase modulation.
 * @details Command magnitude moves the switching frequency from fmax toward
 * fmin, while command sign and magnitude set the secondary-bridge phase shift.
 * Reversing the sign therefore reverses the requested power-flow direction
 * without abandoning frequency control.  The physical sign is selected by the
 * SDPE logical-leg ordering so a wiring change does not require an algorithm edit.
 */
GMP_STATIC_INLINE void ctl_step_clllc_modulator(clllc_modulator_t* mod, ctrl_gt formal_cmd)
{
    ctrl_gt mag = ctl_abs(formal_cmd);
    ctrl_gt period_pu;
    ctrl_gt secondary_phase;
    parameter_gt f_cmd;
    adv_pwm_ift cmd;

    mag = ctl_sat(mag, float2ctrl(1.0f), float2ctrl(0.0f));
    f_cmd = mod->f_max_hz - ctrl2float(mag) * (mod->f_max_hz - mod->f_min_hz);
    secondary_phase = ctl_mul(formal_cmd, mod->max_phase_shift_pu);
    period_pu = float2ctrl(mod->f_nominal_hz / f_cmd);

    cmd.period = period_pu;
    cmd.duty = float2ctrl(0.5f);
    /* dead time in seconds remains constant while frequency changes. */
    cmd.deadband = ctl_div(mod->deadband_pu, period_pu);

    cmd.phase = float2ctrl(0.0f);
    ctl_step_adv_pwm_channel(&mod->leg[0], &cmd);
    cmd.phase = float2ctrl(0.5f);
    ctl_step_adv_pwm_channel(&mod->leg[1], &cmd);
    cmd.phase = secondary_phase;
    ctl_step_adv_pwm_channel(&mod->leg[2], &cmd);
    cmd.phase = secondary_phase + float2ctrl(0.5f);
    ctl_step_adv_pwm_channel(&mod->leg[3], &cmd);

}

#ifdef __cplusplus
}
#endif
#endif
