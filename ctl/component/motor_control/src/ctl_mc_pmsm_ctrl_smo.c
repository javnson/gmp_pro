

#include <gmp_core.h>

#include <ctl/suite/mcs_pmsm/pmsm_ctrl.h>

// #include "peripheral.h"

// init pmsm_bare_controller struct
void ctl_init_pmsm_smo_bare_controller(pmsm_smo_bare_controller_t *ctrl, pmsm_smo_bare_controller_init_t *init)
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

    ctl_smo_init_t smo_init;
    smo_init.Rs = init->Rs;
    smo_init.Ld = init->Ld;
    smo_init.Lq = init->Lq;
    smo_init.pole_pairs = init->pole_pairs;

    smo_init.speed_base_rpm = init->speed_base_rpm;

    smo_init.f_ctrl = init->fs;

    smo_init.u_base = init->u_base;
    smo_init.i_base = init->i_base;

    smo_init.fc_e = init->smo_fc_e;
    smo_init.fc_omega = init->smo_fc_omega;
    smo_init.pid_kp = init->smo_kp;
    smo_init.pid_Ti = init->smo_Ti;
    smo_init.pid_Td = init->smo_Td;
    smo_init.k_slide = init->smo_k_slide;

    smo_init.spd_max_limit = float2ctrl(1.0);
    smo_init.spd_min_limit = float2ctrl(-1.0);

    // init SMO controller
    ctl_init_pmsm_smo(&ctrl->smo, &smo_init);

    ctrl->ramp_freq_spd_set_sf = float2ctrl(init->fs / init->pole_pairs / init->speed_base_rpm * 60);

    ctrl->flag_enable_smo = 0;
    ctrl->flag_switch_cplt = 0;

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

    // attach a speed encoder object with motor controller
    ctl_attach_mtr_velocity(&ctrl->mtr_interface, &ctrl->smo.spdif);

    // init slope f ramp generator
    ctl_init_const_slope_f_controller(&ctrl->ramp_gen, init->ramp_target_freq, init->ramp_target_freq_slope, init->fs);
    ctl_attach_mtr_position(&ctrl->mtr_interface, &ctrl->ramp_gen.enc);

    // flag stack
    ctl_disable_pmsm_smo_ctrl(ctrl);
    ctl_pmsm_smo_ctrl_valphabeta_mode(ctrl);
}

void ctl_attach_pmsm_smo_bare_output(pmsm_smo_bare_controller_t *ctrl, tri_pwm_ift *pwm_out)
{
    ctrl->pwm_out = pwm_out;
}
