
#include <gmp_core.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>

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
    const ctl_hfi_init_t* init)
{

    //
    // hfi output/intermediate section initiate
    //
    hfi->ud_inj = 0;
    hfi->iq_demodulate = 0;
    hfi->theta_error = 0;
    hfi->wr_est = 0;

    //
    // hfi control entity initiate
    //

    // modulation
    ctl_init_sine_generator(&hfi->hfi_sincos_gen,
                            0,                           // pu
                            init->f_hfi / init->f_ctrl); // pu
    hfi->hfi_inj_amp = init->u_amp_hfi;

    // iq lowpass filter
    ctl_init_biquad_lpf(&hfi->iq_lp_filter, init->f_ctrl, init->iq_lp_fc, 1.0f / init->iq_lp_damp / 2);

    // PLL
    ctl_init_pid_ser(&hfi->pid_pll, init->pid_kp, init->pid_Ti, init->pid_Td, init->f_ctrl);
    ctl_set_pid_limit(&hfi->pid_pll, init->spd_max_limit, init->spd_min_limit);

    hfi->spd_sf = float2ctrl((30.0f / CTL_PARAM_CONST_PI) * init->f_ctrl / init->speed_base_rpm);
    hfi->pole_pairs = init->pole_pairs;
    hfi->theta_r_est = 0;
}

//////////////////////////////////////////////////////////////////////////
// ACM flux observer

#include <ctl/component/motor_control/observer/acm.fo.h>

void ctl_init_im_fo(im_fo_t* fo, ctrl_gt rs_star, ctrl_gt ls_star, ctrl_gt lr_star, ctrl_gt lm_star, ctrl_gt pole_pairs,
                    ctrl_gt ts_s, ctrl_gt wc_rps)
{
    fo->psi_r.dat[0] = 0;
    fo->psi_r.dat[1] = 0;
    fo->psi_s.dat[0] = 0;
    fo->psi_s.dat[1] = 0;
    fo->psi_r_mag = 0;
    fo->torque = 0;

    fo->rs = rs_star;
    fo->k_ts = ts_s;
    fo->k_lpf = CTL_CTRL_CONST_1 - ctl_mul(wc_rps, ts_s);

    ctrl_gt sigma = CTL_CTRL_CONST_1 - ctl_div(ctl_mul(lm_star, lm_star), ctl_mul(ls_star, lr_star));
    fo->k_sigma_ls = ctl_mul(sigma, ls_star);
    fo->k_rotor_flux = ctl_div(lr_star, lm_star);
    fo->k_torque = ctl_mul(ctl_mul(CTL_CTRL_CONST_3_OVER_2, pole_pairs), ctl_div(lm_star, lr_star));
}

//////////////////////////////////////////////////////////////////////////
// ACM Flux SMO
#include <ctl/component/motor_control/observer/acm.smo.h>

void ctl_init_acm_smo(ctl_acm_smo_t* smo, const ctl_acm_smo_init_t* init)
{
    // Clear all states
    smo->encif.position = 0;
    //ctl_vector2_clear(&smo->encif.position);
    smo->spdif.speed = 0.0f;
    ctl_vector2_clear(&smo->i_s_est);
    ctl_vector2_clear(&smo->psi_r_est);
    ctl_vector2_clear(&smo->z);
    smo->omega_e_est = 0.0f;
    smo->theta_est = 0.0f;

    // Initialize controllers
    ctl_init_pid_ser(&smo->pid_pll, init->pll_kp, init->pll_ki, 0, init->f_ctrl);
    ctl_init_lp_filter(&smo->filter_spd, init->f_ctrl, init->speed_lpf_fc);

    // Store parameters
    smo->k_slide = init->k_slide;
    smo->pole_pairs = (ctrl_gt)init->pole_pairs;
    smo->Ts = 1.0f / init->f_ctrl;

    // Pre-calculate model coefficients for optimization
    parameter_gt sigma = 1.0f - (init->Lm * init->Lm) / (init->Ls * init->Lr);
    parameter_gt Tr = init->Lr / init->Rr;

    smo->c1 = float2ctrl(1.0f / (sigma * init->Ls));
    smo->c2 = float2ctrl((init->Rs / (sigma * init->Ls)) + ((1.0f - sigma) / (sigma * Tr)));
    smo->c3 = float2ctrl(init->Lm / (sigma * init->Ls * Tr));
    smo->c4 = float2ctrl(init->Lm / (sigma * init->Ls));
    smo->c5 = float2ctrl(init->Lm / Tr);

    // Calculate speed scaling factor
    ctrl_gt base_speed_rad_s = (ctrl_gt)init->speed_base_rpm * CTL_PARAM_CONST_2PI / 60.0f;
    smo->speed_pu_sf = 1.0f / base_speed_rad_s;
}

//////////////////////////////////////////////////////////////////////////
// ACM MPC

#include <ctl/component/motor_control/current_loop/acm_mpc.h>

// Table of the 8 standard voltage vectors in the alpha-beta frame.
const ctl_vector2_t MPC_VOLTAGE_VECTORS_NORMALIZED_ACM[8] = {
    {{float2ctrl(0.0f), float2ctrl(0.0f)}},        // V0
    {{float2ctrl(1.0f), float2ctrl(0.0f)}},        // V1
    {{float2ctrl(0.5f), float2ctrl(0.866025f)}},   // V2
    {{float2ctrl(-0.5f),float2ctrl( 0.866025f)}},  // V3
    {{float2ctrl(-1.0f),float2ctrl( 0.0f)}},       // V4
    {{float2ctrl(-0.5f),float2ctrl( -0.866025f)}}, // V5
    {{float2ctrl(0.5f), float2ctrl(-0.866025f)}},  // V6
    {{float2ctrl(0.0f), float2ctrl(0.0f)}}         // V7
};

void ctl_init_acm_mpc(ctl_acm_mpc_controller_t* mpc, const ctl_acm_mpc_init_t* init)
{
    mpc->optimal_vector_index = 0;
    ctl_vector2_clear(&mpc->psi_r_est);
    mpc->flux_ref_sq = 0;
    mpc->lambda_flux = float2ctrl(50.0f); // Default weighting factor, should be tuned.
    mpc->Ts = float2ctrl(1.0f / (ctrl_gt)init->f_ctrl);
    mpc->pole_pairs = (ctrl_gt)init->pole_pairs;

    // Pre-calculate coefficients for the discrete-time model to optimize the step function.
    // Based on Euler discretization of the continuous-time ACM model in the stationary frame.
    parameter_gt sigma = 1.0f - (init->Lm * init->Lm) / (init->Ls * init->Lr);
    parameter_gt Tr = init->Lr / init->Rr;
    parameter_gt sigma_ls = sigma * init->Ls;

    mpc->c_iss = float2ctrl(1.0f - mpc->Ts * (init->Rs / sigma_ls + (1.0f - sigma) / (sigma * Tr)));
    mpc->c_is_u = float2ctrl(mpc->Ts / sigma_ls);
    mpc->c_is_pr = float2ctrl(mpc->Ts * init->Lm / (sigma_ls * init->Lr * Tr));
    mpc->c_is_pr_w = float2ctrl(mpc->Ts * init->Lm / (sigma_ls * init->Lr));
    mpc->c_pr_is = float2ctrl(mpc->Ts * init->Lm / Tr);
    mpc->c_pr_pr = float2ctrl(1.0f - mpc->Ts / Tr);
}


//////////////////////////////////////////////////////////////////////////
// BLDC Hall

#include <ctl/component/motor_control/observer/bldc.hall.h>

const ctrl_gt HALL_STATE_TO_ANGLE_PU[7] = {
    float2ctrl(0.0f),        // Unused state 0
    float2ctrl(0.0f),        // State 1: 0 degrees
    float2ctrl(1.0f / 6.0f), // State 2: 60 degrees
    float2ctrl(2.0f / 6.0f), // State 3: 120 degrees
    float2ctrl(3.0f / 6.0f), // State 4: 180 degrees
    float2ctrl(4.0f / 6.0f), // State 5: 240 degrees
    float2ctrl(5.0f / 6.0f)  // State 6: 300 degrees
};

void ctl_init_bldc_hall_estimator(ctl_bldc_hall_estimator_t* est, uint16_t pole_pairs, parameter_gt timer_freq_hz,
                                  parameter_gt speed_base_rpm, parameter_gt speed_filter_fc_hz,
                                  parameter_gt isr_freq_hz)
{
    //ctl_vector2_clear(&est->encif.position);
    est->encif.position = 0;
    //ctl_vector2_clear(&est->spdif.speed);
    est->spdif.speed = 0;
    est->last_hall_state = 0;
    est->last_capture_time = 0;
    est->sector_time_delta = 0.0f;
    est->estimated_speed_rad_per_tick = 0.0f;
    est->coarse_angle_pu = 0.0f;

    est->pole_pairs = (ctrl_gt)pole_pairs;
    est->sector_rad = CTL_PARAM_CONST_PI / 3.0f;

    // Calculate speed filter coefficient
    ctrl_gt wc = 2.0f * CTL_PARAM_CONST_PI * (ctrl_gt)speed_filter_fc_hz;
    ctrl_gt ts = 1.0f / (ctrl_gt)isr_freq_hz;
    est->speed_filter_k = wc * ts / (1.0f + wc * ts);

    // Calculate scale factor for converting rad/tick to p.u. speed
    // p.u. speed = (rad/tick * timer_freq) / (base_rpm * 2*pi/60 * pole_pairs)
    ctrl_gt base_speed_rad_s = (ctrl_gt)speed_base_rpm * CTL_PARAM_CONST_2PI / 60.0f;
    est->ticks_to_pu_sf = (ctrl_gt)timer_freq_hz / (base_speed_rad_s * est->pole_pairs);
}

//////////////////////////////////////////////////////////////////////////
// PMSM Flux observer

#include <ctl/component/motor_control/observer/pmsm.fo.h>

void ctl_init_pmsm_fo(pmsm_fo_t* fo, ctrl_gt wb_ls_star, ctrl_gt psi_pm_star)
{
    fo->flux.dat[0] = 0;
    fo->flux.dat[1] = 0;
    fo->flux_mag = 0;
    fo->torque = 0;

    fo->wb_ls = wb_ls_star;
    fo->psi_pm = psi_pm_star;

    // Pre-calculate the inverse for the torque calculation if psi_pm is not zero.
    if (psi_pm_star != 0)
    {
        fo->inv_psi_pm = ctl_div(CTL_CTRL_CONST_1, psi_pm_star);
    }
    else
    {
        fo->inv_psi_pm = 0;
    }
}

//////////////////////////////////////////////////////////////////////////
// PMSM HFI

#include <ctl/component/motor_control/observer/pmsm.hfi.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm.smo.h>

void ctl_init_pmsm_smo(
    // SMO handle
    pmsm_smo_t* smo,
    // SMO Initialize object
    const ctl_smo_init_t* init)
{

    ctl_vector2_clear(&smo->e_est);
    ctl_vector2_clear(&smo->z);
    ctl_vector2_clear(&smo->i_est);

    smo->theta_est = 0;

    ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);

    // smo->k1 = float2ctrl(1.0f / (init->Ld * init->f_ctrl));
    smo->k1 = float2ctrl(1.0f / (init->Ld * init->f_ctrl) * init->u_base / init->i_base);
    smo->k2 = float2ctrl(init->Rs / (init->Ld * init->f_ctrl));
    // smo->k3 = float2ctrl((init->Ld - init->Lq) / (init->Ld * init->f_ ctrl));
    smo->k3 = float2ctrl((init->Ld - init->Lq) / (init->Ld));

    smo->k_slide = float2ctrl(init->k_slide);

    ctl_init_lp_filter(&smo->filter_e[0], init->f_ctrl, init->fc_e);
    ctl_init_lp_filter(&smo->filter_e[1], init->f_ctrl, init->fc_e);
    ctl_init_lp_filter(&smo->filter_spd, init->f_ctrl, init->fc_omega);

    ctl_init_pid_ser(&smo->pid_pll, init->pid_kp, init->pid_Ti, init->pid_Td, init->f_ctrl);
    ctl_set_pid_limit(&smo->pid_pll, init->spd_max_limit, init->spd_min_limit);

    smo->spd_sf = float2ctrl((30.0f / CTL_PARAM_CONST_PI) * init->f_ctrl / init->speed_base_rpm / init->pole_pairs);
    smo->wr_est = 0;

    smo->theta_compensate = float2ctrl(init->speed_base_rpm / 60.0f / init->fc_e * init->pole_pairs);
}
