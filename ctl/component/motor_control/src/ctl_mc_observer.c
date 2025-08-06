
#include <gmp_core.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm.smo.h>

void ctl_init_pmsm_smo(
    // SMO handle
    pmsm_smo_t* smo,
    // SMO Initialize object
    ctl_smo_init_t* init)
{
    smo->e_alpha_est = 0;
    smo->e_beta_est = 0;

    smo->z_alpha = 0;
    smo->z_beta = 0;

    smo->i_alpha = 0;
    smo->i_beta = 0;
    smo->u_alpha = 0;
    smo->u_beta = 0;

    smo->i_alpha_est = 0;
    smo->i_beta_est = 0;

    smo->theta_est = 0;

    ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);

    // smo->k1 = float2ctrl(1.0f / (init->Ld * init->f_ctrl));
    smo->k1 = float2ctrl(1.0f / (init->Ld * init->f_ctrl) * init->u_base / init->i_base);
    smo->k2 = float2ctrl(init->Rs / (init->Ld * init->f_ctrl));
    // smo->k3 = float2ctrl((init->Ld - init->Lq) / (init->Ld * init->f_ ctrl));
    smo->k3 = float2ctrl((init->Ld - init->Lq) / (init->Ld));

    smo->k_slide = float2ctrl(init->k_slide);

    ctl_init_lp_filter(&smo->filter_e_alpha, init->f_ctrl, init->fc_e);
    ctl_init_lp_filter(&smo->filter_e_beta, init->f_ctrl, init->fc_e);
    ctl_init_lp_filter(&smo->filter_spd, init->f_ctrl, init->fc_omega);

    ctl_init_pid_ser(&smo->pid_pll, init->pid_kp, init->pid_Ti, init->pid_Td, init->f_ctrl);
    ctl_set_pid_limit(&smo->pid_pll, init->spd_max_limit, init->spd_min_limit);

    smo->spd_sf = float2ctrl((30.0f / CTL_PARAM_CONST_PI) * init->f_ctrl / init->speed_base_rpm / init->pole_pairs);
    smo->wr = 0;

    smo->theta_compensate = float2ctrl(init->speed_base_rpm / 60.0f / init->fc_e * init->pole_pairs);
}

// void ctl_init_pmsm_smo(ctl_pmsm_smo_observer_t *smo)
//{
//     smo->e_alpha_est = 0;
//     smo->e_beta_est = 0;
//
//     smo->z_alpha = 0;
//     smo->z_beta = 0;
//
//     smo->i_alpha = 0;
//     smo->i_beta = 0;
//     smo->u_alpha = 0;
//     smo->u_beta = 0;
//
//     smo->i_alpha_est = 0;
//     smo->i_beta_est = 0;
//
//     smo->k1 = GMP_CONST_1;
//     smo->k2 = float2ctrl(0.01);
//     smo->k3 = 0;
//
//     smo->k_filter_e = GMP_CONST_1_OVER_2;
//     smo->k_filter_omega = GMP_CONST_1_OVER_2;
//
//     smo->k_slide = float2ctrl(10.0);
//
//     smo->theta_est = 0;
//     ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);
//
//     ctl_init_pid(&smo->pid_pll);
//
//     smo->spd_sf = GMP_CONST_1;
//     smo->spd_est_pu = 0;
//     smo->wr = 0;
// }

// void ctl_init_pmsm_smo(pmsm_smo_observer_t *smo, parameter_gt Rs, parameter_gt Ld, parameter_gt Lq, parameter_gt
// f_ctrl,
//                        parameter_gt fc_e, parameter_gt fc_omega, ctrl_gt pid_kp, ctrl_gt pid_Ti, ctrl_gt pid_Td,
//                        ctrl_gt spd_max_limit, ctrl_gt spd_min_limit, // unit p.u.
//                        ctrl_gt k_slide, parameter_gt speed_base_rpm, uint16_t pole_pairs)
//{
//     smo->e_alpha_est = 0;
//     smo->e_beta_est = 0;
//
//     smo->z_alpha = 0;
//     smo->z_beta = 0;
//
//     smo->i_alpha = 0;
//     smo->i_beta = 0;
//     smo->u_alpha = 0;
//     smo->u_beta = 0;
//
//     smo->i_alpha_est = 0;
//     smo->i_beta_est = 0;
//
//     smo->k1 = float2ctrl(1 / (Ld * f_ctrl));
//     smo->k2 = float2ctrl(Rs / (Ld * f_ctrl));
//     smo->k3 = float2ctrl((Ld - Lq) / (Ld * f_ctrl));
//
//     smo->theta_est = 0;
//     ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);
//
//     smo->k_slide = k_slide;
//
//     smo->k_filter_e = ctl_helper_lp_filter(f_ctrl, fc_e);
//     smo->k_filter_omega = ctl_helper_lp_filter(f_ctrl, fc_omega);
//
//     ctl_init_pid(&smo->pid_pll, pid_kp, pid_Ti, pid_Td, f_ctrl);
//     ctl_set_pid_limit(&smo->pid_pll, spd_min_limit, spd_max_limit);
//
//     smo->spd_sf = float2ctrl((60 / 2 / PI) * f_ctrl / speed_base_rpm / pole_pairs);
//     smo->spd_est_pu = 0;
//     smo->wr = 0;
// }
//
// void ctl_init_pmsm_smo_via_consultant(pmsm_smo_observer_t *smo,
//                                       // use it to calculate controller parameters
//                                       ctl_pmsm_dsn_consultant_t *dsn,
//                                       // use it to calculate controller parameters
//                                       ctl_motor_driver_consultant_t *drv,
//                                       // use it to per unit controller
//                                       ctl_pmsm_nameplate_consultant_t *np, ctrl_gt pid_kp, ctrl_gt pid_ki,
//                                       ctrl_gt pid_kd, ctrl_gt k_slide)
//{
//     ctl_init_pmsm_smo(smo, dsn->Rs, dsn->Ld, dsn->Lq, drv->control_law_freq,
//                       drv->speed_closeloop_bw * (parameter_gt)6.28, drv->speed_closeloop_bw * (parameter_gt)6.28,
//                       pid_kp, pid_ki, pid_kd, float2ctrl(1.2),
//                       -float2ctrl(1.2), // unit p.u.
//                       k_slide, np->rated_speed_rpm, dsn->pole_pair);
// }

//////////////////////////////////////////////////////////////////////////
// acm speed calculator (slip observer)

#include <ctl/component/motor_control/observer/acm.pos_calc.h>

void ctl_init_im_spd_calc(
    // IM speed calculate object
    ctl_im_spd_calc_t* calc,
    // rotor parameters, unit Ohm, H
    parameter_gt Rr, parameter_gt Lr,
    // ACM Rotor, per-unit Speed, unit rpm
    parameter_gt rotor_base,
    // ACM pole pairs
    uint16_t pole_pairs,
    // base electrical frequency(Hz), ISR frequency (Hz)
    parameter_gt freq_base, parameter_gt isr_freq)
{
    //  Rotor time constant (sec)
    parameter_gt Tr = Lr / Rr;

    // ACM flux synchronous speed
    parameter_gt sync_spd = 60 * freq_base / pole_pairs;

    // constant using in magnetizing current calculation
    // calc->kr = float2ctrl(1 / isr_freq * Tr);
    calc->kr = float2ctrl(1 / isr_freq + Tr);

    // calc->kt = float2ctrl(1 / (Tr * 2 * PI * freq_base));
    calc->kt = float2ctrl(1 / (CTL_PARAM_CONST_2PI * freq_base));

    calc->ktheta = float2ctrl(freq_base / isr_freq);

    calc->ksync = float2ctrl(rotor_base / sync_spd);

    // clear parameters
    calc->imds = 0;
    calc->slip = float2ctrl(1.0);
    calc->omega_s = 0;
    calc->enc.elec_position = 0;
}

//////////////////////////////////////////////////////////////////////////
// pmsm hfi

#include <ctl/component/motor_control/observer/pmsm.hfi.h>

void ctl_init_pmsm_hfi(
    // HFI handle
    pmsm_hfi_t* hfi,
    // HFI Initialize object
    ctl_hfi_init_t* init)
{

    //
    // hfi output/intermediate section initiate
    //
    hfi->ud_inj = 0;
    hfi->iq_demodulate = 0;
    hfi->theta_error = 0;
    hfi->wr = 0;

    //
    // hfi control entity initiate
    //

    // modulation
    ctl_init_sine_generator(&hfi->hfi_sincos_gen,
                            0,                           // pu
                            init->f_hfi / init->f_ctrl); // pu
    hfi->hfi_inj_amp = init->u_amp_hfi;

    // iq lowpass filter
    ctl_filter_IIR2_setup_t iq_lp_setup;
    iq_lp_setup.fc = init->iq_lp_fc;
    iq_lp_setup.filter_type = FILTER_IIR2_TYPE_LOWPASS;
    iq_lp_setup.fs = init->f_ctrl;
    iq_lp_setup.gain = 1;
    iq_lp_setup.q = 1 / init->iq_lp_damp / 2;

    ctl_init_filter_iir2(&hfi->iq_lp_filter, &iq_lp_setup);

    // PLL
    ctl_init_pid_ser(&hfi->pid_pll, init->pid_kp, init->pid_Ti, init->pid_Td, init->f_ctrl);
    ctl_set_pid_limit(&hfi->pid_pll, init->spd_max_limit, init->spd_min_limit);

    hfi->spd_sf = float2ctrl((30.0f / CTL_PARAM_CONST_PI) * init->f_ctrl / init->speed_base_rpm);
    hfi->pole_pairs = init->pole_pairs;
    hfi->theta_r_est = 0;
}
