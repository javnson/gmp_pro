
#include <gmp_core.h>





//////////////////////////////////////////////////////////////////////////
// current distributor

//#include <ctl/component/motor_control/current_loop/current_distributor.h>
//
//void ctl_init_current_distributor(ctl_current_distributor_t* dist, ctl_dist_mode_t mode, const ctrl_gt* im_axis,
//                                  const ctrl_gt* alpha_values, uint32_t lut_size, ctrl_gt const_alpha_rad)
//{
//    dist->mode = mode;
//    dist->const_alpha = const_alpha_rad;
//    dist->id_ref = 0.0f;
//    dist->iq_ref = 0.0f;
//
//    if (mode == DIST_MODE_LUT_LINEAR && im_axis != NULL && alpha_values != NULL && lut_size > 1)
//    {
//        // Initialize the 1D LUT structure from surf_search.h
//        ctl_init_lut1d(&dist->im_axis_lut, im_axis, lut_size);
//        dist->alpha_values = alpha_values;
//    }
//    else
//    {
//        // If not using LUT mode or data is invalid, default to constant alpha
//        dist->mode = DIST_MODE_CONST_ALPHA;
//        dist->alpha_values = NULL;
//    }
//}


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
// Current Distributor

#include <ctl/component/motor_control/current_loop/lut_current_distributor.h>

void ctl_init_lut_distributor(ctl_lut_distributor_t* dist, const ctl_lut_distributor_init_t* init)
{
    // 1. Convert offline configuration to real-time control variables
    if (init->v_base > 1e-6f)
    {
        dist->vs_limit = float2ctrl(init->v_lim / init->v_base);
    }
    else
    {
        dist->vs_limit = float2ctrl(0.0f); // Fallback to avoid division by zero
    }

    dist->alpha_lim_fw = float2ctrl(init->alpha_lim_fw);
    dist->alpha_neg_torque = float2ctrl(init->alpha_neg_torque);
    dist->flag_enable_fw = 1; // Enabled by default

    // 2. Initialize the Field Weakening PID controller
    ctl_init_pid(&dist->fw_pid, init->kp_fw, init->ki_fw, 0.0f, init->fs);

    // Clamp the PID output to prevent the angle from exceeding limits.
    ctl_set_pid_limit(&dist->fw_pid, init->alpha_lim_fw, 0.0f);
    ctl_set_pid_int_limit(&dist->fw_pid, init->alpha_lim_fw, 0.0f);

    // 3. Initialize the Paired Look-Up Table module
    if (init->current_alpha_table != NULL && init->lut_size > 1)
    {
        // 直接将传入的结构体数组绑定到 im_lut 中
        ctl_init_paired_lut1d(&dist->im_lut, init->current_alpha_table, init->lut_size);
    }
    else
    {
        // Safe default if LUT data is missing
        dist->im_lut.table = NULL;
        dist->im_lut.size = 0;
    }

    // 4. Clear states
    ctl_clear_lut_distributor(dist);
}

#include <ctl/component/motor_control/current_loop/const_current_distributor.h>

void ctl_init_const_distributor(ctl_const_distributor_t* dist, const ctl_const_distributor_init_t* init)
{
    // 1. Convert offline configuration to real-time control variables
    if (init->v_base > 1e-6f)
    {
        dist->vs_limit = float2ctrl(init->v_lim / init->v_base);
    }
    else
    {
        dist->vs_limit = float2ctrl(0.0f); // Fallback to avoid division by zero
    }

    dist->alpha_lim_fw = float2ctrl(init->alpha_lim_fw);
    dist->alpha_const = float2ctrl(init->alpha_const);
    dist->alpha_neg_torque = float2ctrl(init->alpha_neg_torque);

    dist->flag_enable_fw = 1; // Enabled by default

    // 2. Initialize the Field Weakening PID controller
    ctl_init_pid(&dist->fw_pid, init->kp_fw, init->ki_fw, 0.0f, init->fs);

    // The PID output directly represents the delta_alpha.
    // Clamp the PID output to prevent the angle from exceeding limits or going negative during FW.
    ctl_set_pid_limit(&dist->fw_pid, init->alpha_lim_fw, 0.0f);
    ctl_set_pid_int_limit(&dist->fw_pid, init->alpha_lim_fw, 0.0f);

    // 3. Clear states
    ctl_clear_const_distributor(dist);
}


#include <ctl/component/motor_control/current_loop/idq_current_distributor.h>

void ctl_init_idq_distributor(ctl_idq_distributor_t* dist, const ctl_idq_distributor_init_t* init)
{
    // 1. Core Parameter Mapping
    if (init->v_base > 1e-6f)
    {
        dist->vs_limit = float2ctrl(init->v_lim / init->v_base);
    }
    else
    {
        dist->vs_limit = float2ctrl(0.0f);
    }

    dist->alpha_lim_fw = float2ctrl(init->alpha_lim_fw);
    dist->alpha_const = float2ctrl(init->alpha_const);
    dist->alpha_neg_torque = float2ctrl(init->alpha_neg_torque);

    // Set default flags
    dist->mode = DIST_MODE_CONST_ALPHA; // Default to const alpha, can be changed via setter
    dist->flag_enable_fw = 1;

    // 2. Initialize the Field Weakening PID controller
    ctl_init_pid(&dist->fw_pid, init->kp_fw, init->ki_fw, 0.0f, init->fs);

    // The PID output directly represents delta_alpha.
    ctl_set_pid_limit(&dist->fw_pid, init->alpha_lim_fw, 0.0f);
    ctl_set_pid_int_limit(&dist->fw_pid, init->alpha_lim_fw, 0.0f);

    // 3. Initialize the Paired Look-Up Table module
    if (init->lut_table != NULL && init->lut_size > 1)
    {
        ctl_init_paired_lut1d(&dist->im_lut, init->lut_table, init->lut_size);
    }
    else
    {
        dist->im_lut.table = NULL;
        dist->im_lut.size = 0;
    }

    // 4. Clear states
    ctl_clear_idq_distributor(dist);
}
