
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Motor controller basic structure

#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>

void ctl_auto_tuning_mtr_current_ctrl(mtr_current_init_t* init)
{
    init->current_adc_fc = init->fs / 3;
    init->voltage_adc_fc = init->fs / 3;

    parameter_gt tau = 1.5f / init->fs;
    // 3 ~ 5 is available
    init->current_loop_bw = 1.0f / (3.0f * tau * CTL_PARAM_CONST_2PI);

    // controller delay
    parameter_gt control_delay = CTL_PARAM_CONST_2PI * init->current_loop_bw * tau;

    // input filter delay
    // Create a LPF object and calculate phase lag
    ctl_filter_IIR1_t temp_filter;
    ctl_init_filter_iir1_lpf(&temp_filter, init->fs, init->current_adc_fc);
    parameter_gt filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_bw);

    // current controller phase lag
    init->current_phase_lag = control_delay + filter_delay;

    // calculate PI parameter based on band-width
    parameter_gt lambda = 1.0f / init->current_loop_bw;

    parameter_gt Td = init->mtr_Ld / init->mtr_Rs;
    //    parameter_gt Kd = 1.0f / init->mtr_Rs;

    parameter_gt Tq = init->mtr_Lq / init->mtr_Rs;
    //    parameter_gt Kq = 1.0f / init->mtr_Rs;

    // per unit gain: I_base / V_base
    parameter_gt kp_scale = init->i_base / init->v_base;

    // kp = Ldq * BW
    init->kpd = init->mtr_Ld / (lambda + tau) * kp_scale;
    init->kpq = init->mtr_Lq / (lambda + tau) * kp_scale;

    init->kid = 1 / Td;
    init->kiq = 1 / Tq;
}

void ctl_init_mtr_current_ctrl(mtr_current_ctrl_t* mc, mtr_current_init_t* init)
{
    int i;

    for (i = 0; i < 3; ++i)
    {
        ctl_init_filter_iir1_lpf(&mc->filter_iabc[i], init->fs, init->current_adc_fc);
    }

    ctl_init_filter_iir1_lpf(&mc->filter_udc, init->fs, init->voltage_adc_fc);

    ctl_init_pid(&mc->idq_ctrl[phase_d], init->kpd, init->kid, 0, init->fs);
    ctl_init_pid(&mc->idq_ctrl[phase_q], init->kpq, init->kiq, 0, init->fs);

    ctl_init_lead_form3(&mc->lead_compensator[phase_d], init->current_phase_lag, init->current_loop_bw, init->fs);
    ctl_init_lead_form3(&mc->lead_compensator[phase_q], init->current_phase_lag, init->current_loop_bw, init->fs);

    // krpm, A, V
    parameter_gt omega_base_elec = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;
    parameter_gt scale_fac = omega_base_elec * init->i_base / init->v_base;
    mc->coef_ff_decouple[phase_d] = init->mtr_Lq * scale_fac;
    mc->coef_ff_decouple[phase_q] = init->mtr_Ld * scale_fac;

    ctl_clear_mtr_current_ctrl(mc);
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

//////////////////////////////////////////////////////////////////////////
// current distributor

#include <ctl/component/motor_control/current_loop/current_distributor.h>

void ctl_init_current_distributor(ctl_current_distributor_t* dist, ctl_dist_mode_t mode, const ctrl_gt* im_axis,
                                  const ctrl_gt* alpha_values, uint32_t lut_size, ctrl_gt const_alpha_rad)
{
    dist->mode = mode;
    dist->const_alpha = const_alpha_rad;
    dist->id_ref = 0.0f;
    dist->iq_ref = 0.0f;

    if (mode == DIST_MODE_LUT_LINEAR && im_axis != NULL && alpha_values != NULL && lut_size > 1)
    {
        // Initialize the 1D LUT structure from surf_search.h
        ctl_init_lut1d(&dist->im_axis_lut, im_axis, lut_size);
        dist->alpha_values = alpha_values;
    }
    else
    {
        // If not using LUT mode or data is invalid, default to constant alpha
        dist->mode = DIST_MODE_CONST_ALPHA;
        dist->alpha_values = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////
// DTC

#include <ctl/component/motor_control/current_loop/dtc.h>

// The optimal voltage vector switching table for a 2-level DTC scheme.
uint8_t DTC_SWITCH_TABLE[6][4] = {
    // S=1      S=2      S=3      S=4      S=5      S=6
    {5, 6, 2, 3}, // Flux Sector 1
    {6, 1, 3, 4}, // Flux Sector 2
    {1, 2, 4, 5}, // Flux Sector 3
    {2, 3, 5, 6}, // Flux Sector 4
    {3, 4, 6, 1}, // Flux Sector 5
    {4, 5, 1, 2}  // Flux Sector 6
};

// Table to convert voltage vector index (0-7) to alpha-beta voltages.
ctrl_gt V_ALPHA_BETA_TABLE[8][2] = {
    {float2ctrl(0.0f), float2ctrl(0.0f)},             // V0
    {float2ctrl(0.666667f), float2ctrl(0.0f)},        // V1
    {float2ctrl(0.333333f), float2ctrl(0.577350f)},   // V2
    {float2ctrl(-0.333333f), float2ctrl(0.577350f)},  // V3
    {float2ctrl(-0.666667f), float2ctrl(0.0f)},       // V4
    {float2ctrl(-0.333333f), float2ctrl(-0.577350f)}, // V5
    {float2ctrl(0.333333f), float2ctrl(-0.577350f)},  // V6
    {float2ctrl(0.0f), float2ctrl(0.0f)}              // V7
};

void ctl_init_dtc(ctl_dtc_controller_t* dtc, const ctl_dtc_init_t* init)
{
    dtc->ts = 1.0f / (ctrl_gt)init->f_ctrl;
    dtc->rs = (ctrl_gt)init->Rs;
    dtc->pole_pairs = (ctrl_gt)init->pole_pairs;

    // Initialize hysteresis controllers
    // Flux: Output 1 means INCREASE flux
    ctl_init_hysteresis_controller(&dtc->flux_hcc, 1, init->flux_hyst_width);
    // Torque: Output 1 means INCREASE torque
    ctl_init_hysteresis_controller(&dtc->torque_hcc, 1, init->torque_hyst_width);

    // Clear state variables
    ctl_vector2_clear(&dtc->stator_flux);
    dtc->flux_mag_est = 0.0f;
    dtc->torque_est = 0.0f;
    dtc->flux_sector = 1;
    dtc->voltage_vector_index = 0; // Start with zero vector
}

//////////////////////////////////////////////////////////////////////////
// LADRC
#include <ctl/component/motor_control/current_loop/ladrc_current_controller.h>

void ctl_init_ladrc_current_pu(ctl_ladrc_current_pu_t* ladrc, parameter_gt wc_rads, parameter_gt wo_rads,
                               parameter_gt L_pu, parameter_gt omega_base, parameter_gt sample_time_s)
{
    ladrc->wc = wc_rads;
    ladrc->wo = wo_rads;
    ladrc->h = sample_time_s;

    // Calculate the system gain b0
    if (L_pu > 1e-9f)
    {
        ladrc->b0 = omega_base / L_pu;
    }
    else
    {
        ladrc->b0 = 0.0f; // Avoid division by zero
    }

    // Reset states
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->u_out_pu = 0.0f;
}

//////////////////////////////////////////////////////////////////////////
// MTPA
#include <ctl/component/motor_control/current_loop/mtpa_pu.h>

void ctl_init_mtpa_distributor_si(ctl_mtpa_distributor_t* mtpa, parameter_gt Ld, parameter_gt Lq, parameter_gt psi_f)
{
    mtpa->psi_f = (ctrl_gt)psi_f;
    mtpa->dL = (ctrl_gt)(Ld - Lq);

    if (fabsf(mtpa->dL) > MTPA_SALIENT_POLE_THRESHOLD)
    {
        mtpa->is_salient = 1;
        mtpa->psi_f_sq = mtpa->psi_f * mtpa->psi_f;
        mtpa->four_dL = 4.0f * mtpa->dL;
        mtpa->eight_dL_sq = 8.0f * mtpa->dL * mtpa->dL;
    }
    else
    {
        mtpa->is_salient = 0;
    }
    mtpa->id_ref = 0.0f;
    mtpa->iq_ref = 0.0f;
}

void ctl_init_mtpa_distributor_pu(ctl_mtpa_distributor_t* mtpa, parameter_gt Ld_pu, parameter_gt Lq_pu,
                                  parameter_gt psi_f_pu)
{
    mtpa->psi_f = (ctrl_gt)psi_f_pu;
    mtpa->dL = (ctrl_gt)(Ld_pu - Lq_pu);

    if (fabsf(mtpa->dL) > MTPA_SALIENT_POLE_THRESHOLD)
    {
        mtpa->is_salient = 1;
        mtpa->psi_f_sq = mtpa->psi_f * mtpa->psi_f;
        mtpa->four_dL = 4.0f * mtpa->dL;
        mtpa->eight_dL_sq = 8.0f * mtpa->dL * mtpa->dL;
    }
    else
    {
        mtpa->is_salient = 0;
    }
    mtpa->id_ref = 0.0f;
    mtpa->iq_ref = 0.0f;
}

//////////////////////////////////////////////////////////////////////////
// MPTV

#include <ctl/component/motor_control/current_loop/mtpv.h>
#include <ctl/component/motor_control/current_loop/mtpv_pu.h>

void ctl_init_mtpv(ctl_mtpv_controller_t* mtpv, const ctl_mtpv_init_t* init)
{
    mtpv->rs = (ctrl_gt)init->Rs;
    mtpv->ld = (ctrl_gt)init->Ld;
    mtpv->lq = (ctrl_gt)init->Lq;
    mtpv->psi_f = (ctrl_gt)init->psi_f;

    // Pre-calculate squared terms for efficiency
    mtpv->rs_sq = mtpv->rs * mtpv->rs;
    mtpv->ld_sq = mtpv->ld * mtpv->ld;
    mtpv->lq_sq = mtpv->lq * mtpv->lq;

    mtpv->id_ref = 0.0f;
    mtpv->iq_ref = 0.0f;
}

void ctl_init_mtpv_pu(ctl_mtpv_pu_controller_t* mtpv, parameter_gt Rs_pu, parameter_gt Ld_pu, parameter_gt Lq_pu,
                      parameter_gt psi_f_pu)
{
    mtpv->rs_pu = (ctrl_gt)Rs_pu;
    mtpv->ld_pu = (ctrl_gt)Ld_pu;
    mtpv->lq_pu = (ctrl_gt)Lq_pu;
    mtpv->psi_f_pu = (ctrl_gt)psi_f_pu;

    // Pre-calculate squared terms for efficiency
    mtpv->rs_sq_pu = mtpv->rs_pu * mtpv->rs_pu;
    mtpv->ld_sq_pu = mtpv->ld_pu * mtpv->ld_pu;
    mtpv->lq_sq_pu = mtpv->lq_pu * mtpv->lq_pu;

    mtpv->id_ref_pu = 0.0f;
    mtpv->iq_ref_pu = 0.0f;
}

//////////////////////////////////////////////////////////////////////////
// MPC

#include <ctl/component/motor_control/current_loop/pmsm_mpc.h>

// Table of the 8 standard voltage vectors in the alpha-beta frame.
ctl_vector2_t MPC_VOLTAGE_VECTORS_NORMALIZED[8] = {
    {{float2ctrl(0.0f), float2ctrl(0.0f)}},        // V0
    {{float2ctrl(1.0f), float2ctrl(0.0f)}},        // V1
    {{float2ctrl(0.5f), float2ctrl(0.866025f)}},   // V2
    {{float2ctrl(-0.5f), float2ctrl(0.866025f)}},  // V3
    {{float2ctrl(-1.0f), float2ctrl(0.0f)}},       // V4
    {{float2ctrl(-0.5f), float2ctrl(-0.866025f)}}, // V5
    {{float2ctrl(0.5f), float2ctrl(-0.866025f)}},  // V6
    {{float2ctrl(0.0f), float2ctrl(0.0f)}}         // V7
};

void ctl_init_mpc(ctl_mpc_controller_t* mpc, const ctl_mpc_init_t* init)
{
    mpc->optimal_vector_index = 0; // Default to zero vector
    mpc->Ld = (ctrl_gt)init->Ld;
    mpc->Lq = (ctrl_gt)init->Lq;
    mpc->psi_f = (ctrl_gt)init->psi_f;
    mpc->Ts = 1.0f / (ctrl_gt)init->f_ctrl;

    // Pre-calculate the constant parts of the discrete-time model matrices (A and B)
    // i(k+1) = A*i(k) + B*u(k) + E
    // A = [[1 - Ts*Rs/Ld, Ts*we*Lq/Ld], [-Ts*we*Ld/Lq, 1 - Ts*Rs/Lq]]
    // B = [[Ts/Ld, 0], [0, Ts/Lq]]
    // E = [0, -Ts*we*psi_f/Lq]

    ctrl_gt ts_rs_over_ld = mpc->Ts * (ctrl_gt)init->Rs / mpc->Ld;
    ctrl_gt ts_rs_over_lq = mpc->Ts * (ctrl_gt)init->Rs / mpc->Lq;

    ctl_matrix2_set(&mpc->A_const, 0, 0, 1.0f - ts_rs_over_ld);
    ctl_matrix2_set(&mpc->A_const, 0, 1, 0.0f); // Speed dependent term
    ctl_matrix2_set(&mpc->A_const, 1, 0, 0.0f); // Speed dependent term
    ctl_matrix2_set(&mpc->A_const, 1, 1, 1.0f - ts_rs_over_lq);

    ctl_matrix2_set(&mpc->B, 0, 0, mpc->Ts / mpc->Ld);
    ctl_matrix2_set(&mpc->B, 0, 1, 0.0f);
    ctl_matrix2_set(&mpc->B, 1, 0, 0.0f);
    ctl_matrix2_set(&mpc->B, 1, 1, mpc->Ts / mpc->Lq);
}
