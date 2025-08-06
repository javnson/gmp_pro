

#include <gmp_core.h>

#include <ctl/suite/mcs_acm/acm_sensored_ctrl.h>

// init acm_sensored_bare_controller_t struct
void ctl_init_acm_sensored_bare_controller(
    // ACM Controller handle
    acm_sensored_bare_controller_t *ctrl,
    // ACM initialize structure
    acm_sensored_bare_controller_init_t *init)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    // controller implement
    ctl_init_discrete_pid(
        // d axis current controller
        &ctrl->current_ctrl[phase_d],
        // parameters for current controller
        init->current_pid_gain, init->current_Ti, init->current_Td,
        // controller frequency
        init->fs);
    ctl_set_discrete_pid_limit(&ctrl->current_ctrl[phase_d], init->voltage_limit_max, init->voltage_limit_min);

    ctl_init_discrete_pid(
        // d axis current controller
        &ctrl->current_ctrl[phase_q],
        // parameters for current controller
        init->current_pid_gain, init->current_Ti, init->current_Td,
        // controller frequency
        init->fs);
    ctl_set_discrete_pid_limit(&ctrl->current_ctrl[phase_q], init->voltage_limit_max, init->voltage_limit_min);

    ctl_init_discrete_track_pid(
        // speed controller
        &ctrl->spd_ctrl,
        // parameters for speed controller
        init->spd_pid_gain, init->spd_Ti, init->spd_Td,
        // saturation
        init->current_limit_max, init->current_limit_min,
        // acceleration
        init->acc_limit_max, init->acc_limit_min,
        // speed controller divider
        init->spd_ctrl_div,
        // controller frequency
        init->fs);

#else // using continuous controller

    ctl_init_pid(
        // d axis current controller
        &ctrl->current_ctrl[phase_d],
        // parameters for current controller
        init->current_pid_gain, init->current_Ti, init->current_Td,
        // controller frequency
        init->fs);
    ctl_set_pid_limit(&ctrl->current_ctrl[phase_d], init->voltage_limit_max, init->voltage_limit_min);

    ctl_init_pid(
        // d axis current controller
        &ctrl->current_ctrl[phase_q],
        // parameters for current controller
        init->current_pid_gain, init->current_Ti, init->current_Td,
        // controller frequency
        init->fs);
    ctl_set_pid_limit(&ctrl->current_ctrl[phase_q], init->voltage_limit_max, init->voltage_limit_min);

    ctl_init_track_pid(
        // speed controller
        &ctrl->spd_ctrl,
        // parameters for speed controller
        init->spd_pid_gain, init->spd_Ti, init->spd_Td,
        // saturation
        init->current_limit_max, init->current_limit_min,
        // acceleration
        init->acc_limit_max, init->acc_limit_min,
        // speed controller divider
        init->spd_ctrl_div,
        // controller frequency
        init->fs);

#endif // PMSM_CTRL_USING_DISCRETE_CTRL

    // flux estimate
    ctl_init_im_spd_calc(
        // IM speed calculate object
        &ctrl->flux_calc,
        // rotor parameters, unit Ohm, H
        init->Rr, init->Lr,
        // ACM Rotor, per-unit Speed, unit rpm
        init->base_spd,
        // ACM pole pairs
        init->pole_pairs,
        // base electrical frequency(Hz), ISR frequency (Hz)
        init->base_freq, init->fs);

    // Create position encoder and speed encoder

    // controller intermediate variable
    ctl_vector3_clear(&ctrl->iab0);
    ctl_vector3_clear(&ctrl->uab0);
    ctl_vector3_clear(&ctrl->idq0);
    ctl_vector3_clear(&ctrl->udq0);

    // controller feed forward parameters
    ctl_vector2_clear(&ctrl->idq_ff);
    ctl_vector2_clear(&ctrl->vdq_ff);

    // controller set parameters
    ctl_vector3_clear(&ctrl->vab0_set);
    ctl_vector3_clear(&ctrl->vdq_set);
    ctl_vector2_clear(&ctrl->idq_set);
    ctrl->speed_set = 0;

    // scale factor for per-unit to RG frequency
    ctrl->speed_pu_rg_sf = float2ctrl(init->base_spd * init->pole_pairs / 60.0 / init->fs);

    // connect flux Position encoder with angle generator
    ctl_attach_mtr_position(&ctrl->mtr_interface, &ctrl->rg.enc);

    // rotor speed controller
    ctl_init_spd_calculator(
        // speed calculator objects
        &ctrl->spd_enc,
        // link to a position encoder
        ctrl->rotor_pos,
        // control law frequency, unit Hz
        init->fs,
        // division of control law frequency, unit ticks
        init->spd_ctrl_div,
        // Speed per unit base value, unit rpm
        init->base_spd,
        // pole pairs, if you pass a elec-angle,
        1,
        // just set this value to 1.
        // generally, speed_filter_fc approx to speed_calc freq divided by 5
        150);

    // connect speed with encoder
    ctl_attach_mtr_velocity(&ctrl->mtr_interface, &ctrl->spd_enc.encif);

    // ramp generator
    ctl_init_const_slope_f_controller(
        // controller object
        &ctrl->rg,
        // target frequency, Hz
        0,
        // frequency slope, Hz/s
        init->target_freq_slope,
        // ISR frequency
        init->fs);

    // flag stack
    ctl_enable_acm_sensored_ctrl_flux_est(ctrl);
    ctl_disable_acm_sensored_ctrl(ctrl);
    ctl_acm_sensored_ctrl_valphabeta_mode(ctrl);
}

// attach to output port
void ctl_attach_acm_sensored_bare_output(
    // ACM Controller handle
    acm_sensored_bare_controller_t *ctrl,
    // PWM handle
    tri_pwm_ift *pwm_out)
{
    ctrl->pwm_out = pwm_out;
}

// attach to rotor speed encoder port
void ctl_attach_acm_sensored_bare_rotor_postion(
    // ACM Controller handle
    acm_sensored_bare_controller_t *ctrl,
    // rotor position
    rotation_ift *rotor_enc)
{
    ctrl->rotor_pos = rotor_enc;
}
