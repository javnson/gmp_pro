
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Motor controller basic structure

#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>

void ctl_setup_current_controller(ctl_current_controller_t* cc, ctrl_gt kp, ctrl_gt Ti, ctrl_gt Td, ctrl_gt out_max,
                                  ctrl_gt out_min, parameter_gt fs)
{
    // clear the controller
    ctl_clear_current_controller(cc);

    // Setup the d-axis current controller
    ctl_init_pid_ser(&cc->idq_ctrl[0], kp, Ti, Td, fs);
    ctl_set_pid_limit(&cc->idq_ctrl[0], out_max, out_min);

    // Setup the q-axis current controller
    ctl_init_pid_ser(&cc->idq_ctrl[1], kp, Ti, Td, fs);
    ctl_set_pid_limit(&cc->idq_ctrl[1], out_max, out_min);

    cc->flag_enable_controller = 0;
}

//////////////////////////////////////////////////////////////////////////
// PMSM DPCC init

#include <ctl/component/motor_control/current_loop/PMSM_DPCC.h>

void ctl_init_PMSM_DPCC(ctl_dpcc_controller_t* dpcc, ctl_dpcc_init_t* init)
{
    ////current control coefficient cal
    //ctrl->coeff_d_current = float2ctrl(init->Ubase / init->fctrl / init->Ld / init->Ibase);
    //ctrl->coeff_q_current = float2ctrl(init->Ubase / init->fctrl / init->Lq / init->Ibase);
    ////voltage control coefficient cal
    //ctrl->coeff_d_voltage = float2ctrl(init->Ibase * init->fctrl * init->Ld / init->Ubase);
    //ctrl->coeff_q_voltage = float2ctrl(init->Ibase * init->fctrl * init->Lq / init->Ubase);

    ////motor parameter unify
    //ctrl->Rs_pu = float2ctrl(init->Rs * init->Ibase / init->Ubase);
    //ctrl->Ld_pu = float2ctrl(init->Ld * CTL_PARAM_CONST_2PI * init->fbase * init->Ibase / init->Ubase);
    //ctrl->Lq_pu = float2ctrl(init->Lq * CTL_PARAM_CONST_2PI * init->fbase * init->Ibase / init->Ubase);
    //ctrl->Psi_f_pu = float2ctrl(CTL_PARAM_CONST_2PI * init->Psi_f * init->fbase / init->Ubase);

    parameter_gt Ts = 1.0f / init->f_ctrl;

    // Store parameters
    dpcc->rs = (ctrl_gt)init->Rs;
    dpcc->ld = (ctrl_gt)init->Ld;
    dpcc->lq = (ctrl_gt)init->Lq;
    dpcc->psi_f = (ctrl_gt)init->psi_f;

    // Pre-calculate coefficients for the step function
    dpcc->ts_over_ld = (ctrl_gt)(Ts / init->Ld);
    dpcc->ts_over_lq = (ctrl_gt)(Ts / init->Lq);
    dpcc->ld_over_ts = (ctrl_gt)(init->Ld / Ts);
    dpcc->lq_over_ts = (ctrl_gt)(init->Lq / Ts);

    ctl_clear_dpcc(dpcc);
}
