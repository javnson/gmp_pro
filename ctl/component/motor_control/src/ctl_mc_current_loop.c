
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Motor controller basic structure

#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>

void ctl_setup_motor_current_ctrl(
    // motor current controller
    ctl_motor_current_ctrl_t* obj,
    // PID parameter for motor current controller
    ctrl_gt kp, ctrl_gt Ti, ctrl_gt Td,
    // PID saturation parameter for motor current controller
    ctrl_gt out_min, ctrl_gt out_max,
    // controller rate, Hz
    parameter_gt fs)
{
    obj->udc = float2ctrl(1.0);
    ctl_vector3_clear(&obj->iabc);
    ctl_vector2_clear(&obj->idq_ref);
    ctl_vector2_clear(&obj->vdq_ff);
    obj->theta = 0;

    // ctl_vector3_clear(&obj->Tabc);
    ctl_vector3_clear(&obj->idq0);
    ctl_vector3_clear(&obj->iab0);
    ctl_vector3_clear(&obj->vdq0);
    ctl_vector3_clear(&obj->vab0);

    ctl_init_pid_ser(&obj->idq_ctrl[phase_d], kp, Ti, Td, fs);
    ctl_set_pid_limit(&obj->idq_ctrl[phase_d], out_min, out_max);
    ctl_init_pid_ser(&obj->idq_ctrl[phase_q], kp, Ti, Td, fs);
    ctl_set_pid_limit(&obj->idq_ctrl[phase_q], out_min, out_max);

    obj->flag_enable_current_controller = 1;
}

//////////////////////////////////////////////////////////////////////////
// PMSM DPCC init

#include <ctl/component/motor_control/current_loop/PMSM_DPCC.h>

void ctl_init_PMSM_DPCC(PMSM_DPCC_t* ctrl, PMSM_DPCC_init_t* init)
{
    //current control coefficient cal
    ctrl->coeff_d_current = float2ctrl(init->Ubase / init->fctrl / init->Ld / init->Ibase);
    ctrl->coeff_q_current = float2ctrl(init->Ubase / init->fctrl / init->Lq / init->Ibase);
    //voltage control coefficient cal
    ctrl->coeff_d_voltage = float2ctrl(init->Ibase * init->fctrl * init->Ld / init->Ubase);
    ctrl->coeff_q_voltage = float2ctrl(init->Ibase * init->fctrl * init->Lq / init->Ubase);

    //motor parameter unify
    ctrl->Rs_pu = float2ctrl(init->Rs * init->Ibase / init->Ubase);
    ctrl->Ld_pu = float2ctrl(init->Ld * 2 * PI * init->fbase * init->Ibase / init->Ubase);
    ctrl->Lq_pu = float2ctrl(init->Lq * 2 * PI * init->fbase * init->Ibase / init->Ubase);
    ctrl->Psi_f_pu = float2ctrl(2 * PI * init->Psi_f * init->fbase / init->Ubase);
}
