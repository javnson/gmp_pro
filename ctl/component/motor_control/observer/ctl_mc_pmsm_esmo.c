
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm_esmo.h>

/**
 * @file pmsm_esmo.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the Extended-EMF Sliding Mode Observer (ESMO).
 *
 * @version 3.1
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */

#include "pmsm_esmo.h"

void ctl_init_pmsm_esmo_consultant(ctl_pmsm_esmo_t* esmo, const ctl_consultant_pmsm_t* motor,
                                   const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt fc_emf,
                                   parameter_gt ato_bw_hz, parameter_gt fault_time_ms)
{
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * pu->V_base) / (motor->Ld * pu->I_base);
    parameter_gt k2 = (motor->Rs * Ts) / motor->Ld;
    parameter_gt k3 = (motor->Ld - motor->Lq) / motor->Ld;

    esmo->k1 = float2ctrl(k1);
    esmo->k2 = float2ctrl(k2);
    esmo->k3 = float2ctrl(k3);

    esmo->sf_w_to_rad_tick = float2ctrl(pu->W_base * Ts);

    // 2. Sliding Gain & Margin Calculation
    // Ensuring the sliding gain exceeds the maximum back-EMF magnitude.
    parameter_gt e_max_pu = (pu->W_base * motor->flux_linkage) / pu->V_base;
    esmo->k_slide = float2ctrl(e_max_pu * 1.2f);

    // Configurable boundary layer margin (default 5%). Can be dynamically scheduled if needed.
    parameter_gt default_margin = 0.05f;
    esmo->z_margin = float2ctrl(default_margin);
    esmo->sf_z_margin_inv = float2ctrl(1.0f / default_margin);

    // 3. Sub-module Initialization
    ctl_init_filter_iir1_lpf(&esmo->filter_e[0], fs_safe, fc_emf);
    ctl_init_filter_iir1_lpf(&esmo->filter_e[1], fs_safe, fc_emf);

    // Initialize ATO with default wide saturation margins (+/- 1.5 PU) to handle deep field weakening.
    ctl_init_ato_pll(&esmo->ato_pll, ato_bw_hz, 1.0f, pu->W_base, fs_safe, 1.5f, -1.5f);

    // 4. Phase Compensation Constants
    parameter_gt wc = CTL_PARAM_CONST_2PI * fc_emf;
    esmo->sf_wc_inv = float2ctrl(pu->W_base / wc);

    // 5. Protection Mechanisms Setup
    esmo->current_err_limit = float2ctrl(0.3f);
    esmo->diverge_limit = (uint32_t)(fault_time_ms * fs_safe / 1000.0f);
    if (esmo->diverge_limit < 1)
        esmo->diverge_limit = 1;

    // 6. Finalize Initialization
    ctl_clear_pmsm_esmo(esmo);
    ctl_disable_pmsm_esmo(esmo);
}

/**
 * @brief Core initialization function using the bare physical parameters.
 */
void ctl_init_pmsm_esmo(ctl_pmsm_esmo_t* esmo, const ctl_pmsm_esmo_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * init->V_base) / (init->Ld * init->I_base);
    parameter_gt k2 = (init->Rs * Ts) / init->Ld;
    parameter_gt k3 = (init->Ld - init->Lq) / init->Ld;

    esmo->k1 = float2ctrl(k1);
    esmo->k2 = float2ctrl(k2);
    esmo->k3 = float2ctrl(k3);

    esmo->sf_w_to_rad_tick = float2ctrl(init->W_base * Ts);

    // 2. Sliding Gain & Margin Calculation
    parameter_gt e_max_pu = (init->W_base * init->flux_linkage) / init->V_base;
    esmo->k_slide = float2ctrl(e_max_pu * 1.2f);

    parameter_gt margin = (init->z_margin_pu > 1e-4f) ? init->z_margin_pu : 0.05f;
    esmo->z_margin = float2ctrl(margin);
    esmo->sf_z_margin_inv = float2ctrl(1.0f / margin);

    // 3. Sub-module Initialization
    ctl_init_filter_iir1_lpf(&esmo->filter_e[0], fs_safe, init->fc_emf);
    ctl_init_filter_iir1_lpf(&esmo->filter_e[1], fs_safe, init->fc_emf);

    ctl_init_ato_pll(&esmo->ato_pll, init->ato_bw_hz, 1.0f, init->W_base, fs_safe, 1.5f, -1.5f);

    // 4. Phase Compensation Constants
    parameter_gt wc = CTL_PARAM_CONST_2PI * init->fc_emf;
    esmo->sf_wc_inv = float2ctrl(init->W_base / wc);

    // 5. Protection Mechanisms Setup
    parameter_gt err_lim = (init->current_err_limit_pu > 1e-3f) ? init->current_err_limit_pu : 0.3f;
    esmo->current_err_limit = float2ctrl(err_lim);

    esmo->diverge_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (esmo->diverge_limit < 1)
        esmo->diverge_limit = 1;

    // 6. Finalize Initialization
    ctl_clear_pmsm_esmo(esmo);
    ctl_disable_pmsm_esmo(esmo);
}
