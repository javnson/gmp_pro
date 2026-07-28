#include <gmp_core.h>

#include <ctl/component/digital_power/inv/inv_voltage_ctrl.h>

void ctl_auto_tuning_voltage_inv(inv_voltage_ctrl_init_t* voltage_init, const gfl_inv_ctrl_init_t* gfl_init)
{
    gmp_base_assert(voltage_init);
    gmp_base_assert(gfl_init);
    gmp_base_assert(gfl_init->fs > 0.0f);
    gmp_base_assert(gfl_init->grid_filter_C > 0.0f);

    voltage_init->fs = gfl_init->fs;
    voltage_init->freq_base = gfl_init->freq_base;
    voltage_init->v_base = gfl_init->v_base;
    voltage_init->i_base = gfl_init->i_base;
    voltage_init->filter_C = gfl_init->grid_filter_C;

    /* Keep the outer loop comfortably below the existing current loop. */
    voltage_init->voltage_loop_bw = gfl_init->current_loop_bw / 5.0f;
    voltage_init->voltage_loop_zero = voltage_init->voltage_loop_bw / 5.0f;
    voltage_init->current_limit_max = 1.0f;
    voltage_init->current_limit_min = -1.0f;
}

void ctl_update_voltage_inv_coeff(inv_voltage_ctrl_t* voltage, const inv_voltage_ctrl_init_t* init)
{
    parameter_gt kp;
    parameter_gt ki;

    gmp_base_assert(voltage);
    gmp_base_assert(init);
    gmp_base_assert(init->fs > 0.0f);
    gmp_base_assert(init->v_base > 0.0f);
    gmp_base_assert(init->i_base > 0.0f);
    gmp_base_assert(init->filter_C > 0.0f);
    gmp_base_assert(init->current_limit_max >= init->current_limit_min);

    /*
     * Capacitor plant: G(s) = 1/(C*s).
     * Convert the physical A/V gain to per-unit current / per-unit voltage.
     */
    kp = CTL_PARAM_CONST_2PI * init->voltage_loop_bw * init->filter_C * init->v_base / init->i_base;
    ki = kp * CTL_PARAM_CONST_2PI * init->voltage_loop_zero;

    ctl_init_pid(&voltage->pid_vdq[phase_d], kp, ki, 0.0f, init->fs);
    ctl_init_pid(&voltage->pid_vdq[phase_q], kp, ki, 0.0f, init->fs);
    ctl_set_pid_limit(&voltage->pid_vdq[phase_d], float2ctrl(init->current_limit_max),
                      float2ctrl(init->current_limit_min));
    ctl_set_pid_limit(&voltage->pid_vdq[phase_q], float2ctrl(init->current_limit_max),
                      float2ctrl(init->current_limit_min));

    voltage->coef_ff_decouple =
        float2ctrl(CTL_PARAM_CONST_2PI * init->freq_base * init->filter_C * init->v_base / init->i_base);
    voltage->current_limit_max = float2ctrl(init->current_limit_max);
    voltage->current_limit_min = float2ctrl(init->current_limit_min);
}

void ctl_init_voltage_inv(inv_voltage_ctrl_t* voltage, const inv_voltage_ctrl_init_t* init)
{
    gmp_base_assert(voltage);

    voltage->vab = NULL;
    voltage->phasor = NULL;
    voltage->idq_sink = NULL;

    ctl_update_voltage_inv_coeff(voltage, init);
    ctl_clear_voltage_inv(voltage);
    ctl_vector2_clear(&voltage->vdq_set);

    voltage->flag_enable = 0;
    voltage->flag_enable_decouple = 1;
}
