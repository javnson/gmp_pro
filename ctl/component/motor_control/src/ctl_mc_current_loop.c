
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

void ctl_init_dpcc(ctl_dpcc_controller_t* dpcc, const ctl_dpcc_init_t* init)
{
    parameter_gt Ts = 1.0f / init->f_ctrl;

    // Store base parameters
    dpcc->rs = float2ctrl(init->Rs);
    dpcc->ld = float2ctrl(init->Ld);
    dpcc->lq = float2ctrl(init->Lq);
    dpcc->psi_f = float2ctrl(init->psi_f);

    // Pre-calculate coefficients to avoid divisions in the control loop
    dpcc->ts_over_ld = float2ctrl(Ts / init->Ld);
    dpcc->ts_over_lq = float2ctrl(Ts / init->Lq);
    dpcc->ld_over_ts = float2ctrl(init->Ld / Ts);
    dpcc->lq_over_ts = float2ctrl(init->Lq / Ts);

    // Initialize all state variables to zero
    ctl_clear_dpcc(dpcc);
}
