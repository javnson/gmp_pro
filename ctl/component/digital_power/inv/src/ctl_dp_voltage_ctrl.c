#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/inv/inv_voltage_ctrl.h>

void ctl_auto_tuning_voltage_inv(inv_voltage_ctrl_init_t* voltage_init, const gfl_inv_ctrl_init_t* gfl_init)
{
    gmp_ctl_assert(voltage_init);
    gmp_ctl_assert(gfl_init);
    gmp_ctl_assert(gfl_init->fs > 0.0f);
    gmp_ctl_assert(gfl_init->grid_filter_C > 0.0f);

    voltage_init->fs = gfl_init->fs;
    voltage_init->freq_base = gfl_init->freq_base;
    voltage_init->v_base = gfl_init->v_base;
    voltage_init->i_base = gfl_init->i_base;
    voltage_init->filter_C = gfl_init->grid_filter_C;

    /* Keep the outer loop comfortably below the existing current loop. */
    voltage_init->voltage_loop_bw = gfl_init->current_loop_bw / 5.0f;
    voltage_init->voltage_loop_zero = voltage_init->voltage_loop_bw / 5.0f;
    voltage_init->current_circle_limit = 1.0f;
    voltage_init->current_square_limit = 1.0f;
    voltage_init->flag_enable_circle_limit = 1;
    voltage_init->flag_enable_square_limit = 0;
}

void ctl_update_voltage_inv_coeff(inv_voltage_ctrl_t* voltage, const inv_voltage_ctrl_init_t* init)
{
    parameter_gt kp;
    parameter_gt ki;
    parameter_gt integral_limit;

    gmp_ctl_assert(voltage);
    gmp_ctl_assert(init);
    gmp_ctl_assert(init->fs > 0.0f);
    gmp_ctl_assert(init->v_base > 0.0f);
    gmp_ctl_assert(init->i_base > 0.0f);
    gmp_ctl_assert(init->filter_C > 0.0f);
    gmp_ctl_assert(init->current_circle_limit > 0.0f);
    gmp_ctl_assert(init->current_square_limit > 0.0f);

    /*
     * Capacitor plant: G(s) = 1/(C*s).
     * Convert the physical A/V gain to per-unit current / per-unit voltage.
     */
    kp = CTL_PARAM_CONST_2PI * init->voltage_loop_bw * init->filter_C * init->v_base / init->i_base;
    ki = kp * CTL_PARAM_CONST_2PI * init->voltage_loop_zero;
    integral_limit = init->current_circle_limit > init->current_square_limit
                         ? init->current_circle_limit
                         : init->current_square_limit;

    ctl_init_pid(&voltage->pid_vdq[phase_d], kp, ki, 0.0f, init->fs);
    ctl_init_pid(&voltage->pid_vdq[phase_q], kp, ki, 0.0f, init->fs);
    ctl_set_pid_int_limit(&voltage->pid_vdq[phase_d], real2ctrl(integral_limit),
                          real2ctrl(-integral_limit));
    ctl_set_pid_int_limit(&voltage->pid_vdq[phase_q], real2ctrl(integral_limit),
                          real2ctrl(-integral_limit));

    voltage->coef_ff_decouple =
        real2ctrl(CTL_PARAM_CONST_2PI * init->freq_base * init->filter_C * init->v_base / init->i_base);
    voltage->current_circle_limit = param2ctrl(init->current_circle_limit);
    voltage->current_square_limit = param2ctrl(init->current_square_limit);
    voltage->flag_enable_circle_limit = init->flag_enable_circle_limit;
    voltage->flag_enable_square_limit = init->flag_enable_square_limit;
}

void ctl_init_voltage_inv(inv_voltage_ctrl_t* voltage, const inv_voltage_ctrl_init_t* init)
{
    gmp_ctl_assert(voltage);

    voltage->vab = NULL;
    voltage->phasor = NULL;
    voltage->idq_sink = NULL;

    ctl_update_voltage_inv_coeff(voltage, init);
    ctl_clear_voltage_inv(voltage);
    ctl_vector2_clear(&voltage->vdq_set);

    voltage->flag_enable = 0;
    voltage->flag_enable_decouple = 1;
}
