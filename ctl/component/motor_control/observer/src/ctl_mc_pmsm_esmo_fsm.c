#include <ctl/component/motor_control/observer/pmsm_esmo_fsm.h>

void ctl_init_esmo_fsm(ctl_pmsm_esmo_fsm_t* fsm, ctl_pmsm_esmo_t* esmo_ptr, ctl_slope_f_pu_controller* if_slope_ptr,
    parameter_gt fs, parameter_gt t_trans_sec, parameter_gt up_th_pu, parameter_gt down_th_pu,
    parameter_gt if_is_start)
{
    fsm->esmo_ptr = esmo_ptr;
    fsm->if_slope_ptr = if_slope_ptr;

    fsm->speed_up_th_pu = float2ctrl(up_th_pu);
    fsm->speed_down_th_pu = float2ctrl(down_th_pu);

    fsm->if_is_startup = float2ctrl(if_is_start);

    parameter_gt alpha_step = 1.0f / (t_trans_sec * fs);
    ctl_init_slope_limiter(&fsm->alpha_ramp, alpha_step, -alpha_step, fs);
    ctl_set_slope_limiter_current(&fsm->alpha_ramp, float2ctrl(0.0f));

    ctl_clear_esmo_fsm(fsm);
}