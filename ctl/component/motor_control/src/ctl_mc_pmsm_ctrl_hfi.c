

#include <gmp_core.h>

#include <ctl/suite/mcs_pmsm/pmsm_ctrl.h>

// #include "peripheral.h"

// init pmsm_bare_controller struct
void ctl_init_pmsm_bare_controller(pmsm_bare_controller_t *ctrl, pmsm_bare_controller_init_t *init)
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
    ctrl->pos_set = 0;
    ctrl->revolution_set = 0;

    // flag stack
    ctl_disable_pmsm_ctrl(ctrl);
    ctl_pmsm_ctrl_valphabeta_mode(ctrl);
}

void ctl_attach_pmsm_bare_output(pmsm_bare_controller_t *ctrl, tri_pwm_ift *pwm)
{
    ctrl->pwm_out = pwm;
}
