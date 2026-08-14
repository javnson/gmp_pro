
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

#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm_esmo.h>

void ctl_init_pmsm_esmo_consultant(ctl_pmsm_esmo_t* esmo, const ctl_consultant_pmsm_t* motor,
                                   const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt fc_emf,
                                   parameter_gt ato_bw_hz, parameter_gt fault_time_ms)
{
    parameter_gt fs_safe = (fs > CTL_PARAM_CONST_EPSILON) ? fs : real2param(10000.0);
    parameter_gt Ts = CTL_PARAM_CONST_1 / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * pu->V_base) / (motor->Ld * pu->I_base);
    parameter_gt k2 = (motor->Rs * Ts) / motor->Ld;
    parameter_gt k3 = (motor->Ld - motor->Lq) / motor->Ld;

    esmo->k1 = param2ctrl(k1);
    esmo->k2 = param2ctrl(k2);
    esmo->k3 = param2ctrl(k3);

    esmo->sf_w_to_rad_tick = param2ctrl(pu->W_base * Ts);

    // 2. Sliding Gain & Margin Calculation
    // Ensuring the sliding gain exceeds the maximum back-EMF magnitude.
    parameter_gt e_max_pu = (pu->W_base * motor->flux_linkage) / pu->V_base;
    esmo->k_slide = param2ctrl(e_max_pu * real2param(1.2));

    // Configurable boundary layer margin (default 5%). Can be dynamically scheduled if needed.
    parameter_gt default_margin = CTL_PARAM_CONST_1_OVER_20;
    esmo->z_margin = param2ctrl(default_margin);
    esmo->sf_z_margin_inv = param2ctrl(CTL_PARAM_CONST_1 / default_margin);

    // 3. Sub-module Initialization
    ctl_init_filter_iir1_lpf(&esmo->filter_e[0], fs_safe, fc_emf);
    ctl_init_filter_iir1_lpf(&esmo->filter_e[1], fs_safe, fc_emf);

    // Initialize ATO with default wide saturation margins (+/- 1.5 PU) to handle deep field weakening.
    ctl_init_ato_pll(&esmo->ato_pll, ato_bw_hz, CTL_PARAM_CONST_1, pu->W_base, fs_safe,
                     CTL_PARAM_CONST_3_OVER_2, -CTL_PARAM_CONST_3_OVER_2);

    // 4. Phase Compensation Constants
    parameter_gt wc = CTL_PARAM_CONST_2PI * fc_emf;
    esmo->sf_wc_inv = param2ctrl(pu->W_base / wc);

    // 5. Protection Mechanisms Setup
    esmo->current_err_limit = param2ctrl(real2param(0.3));
    esmo->diverge_limit = (uint32_t)(fault_time_ms * fs_safe / real2param(1000.0));
    if (esmo->diverge_limit < 1)
        esmo->diverge_limit = 1;

    // 6. Finalize Initialization
    ctl_clear_pmsm_esmo(esmo);
    ctl_enable_pmsm_esmo_emf_normalization(esmo);
    ctl_disable_pmsm_esmo(esmo);
}

/**
 * @brief Core initialization function using the bare physical parameters.
 */
void ctl_init_pmsm_esmo(ctl_pmsm_esmo_t* esmo, const ctl_pmsm_esmo_init_t* init)
{
    parameter_gt fs_safe = (init->fs > CTL_PARAM_CONST_EPSILON) ? init->fs : real2param(10000.0);
    parameter_gt Ts = CTL_PARAM_CONST_1 / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * init->V_base) / (init->Ld * init->I_base);
    parameter_gt k2 = (init->Rs * Ts) / init->Ld;
    parameter_gt k3 = (init->Ld - init->Lq) / init->Ld;

    esmo->k1 = param2ctrl(k1);
    esmo->k2 = param2ctrl(k2);
    esmo->k3 = param2ctrl(k3);

    esmo->sf_w_to_rad_tick = param2ctrl(init->W_base * Ts);

    // 2. Sliding Gain & Margin Calculation
    parameter_gt e_max_pu = (init->W_base * init->flux_linkage) / init->V_base;
    esmo->k_slide = param2ctrl(e_max_pu * real2param(1.2));

    parameter_gt margin =
        (init->z_margin_pu > real2param(1.0e-4)) ? init->z_margin_pu : CTL_PARAM_CONST_1_OVER_20;
    esmo->z_margin = param2ctrl(margin);
    esmo->sf_z_margin_inv = param2ctrl(CTL_PARAM_CONST_1 / margin);

    // 3. Sub-module Initialization
    ctl_init_filter_iir1_lpf(&esmo->filter_e[0], fs_safe, init->fc_emf);
    ctl_init_filter_iir1_lpf(&esmo->filter_e[1], fs_safe, init->fc_emf);

    ctl_init_ato_pll(&esmo->ato_pll, init->ato_bw_hz, CTL_PARAM_CONST_1, init->W_base, fs_safe,
                     CTL_PARAM_CONST_3_OVER_2, -CTL_PARAM_CONST_3_OVER_2);

    // 4. Phase Compensation Constants
    parameter_gt wc = CTL_PARAM_CONST_2PI * init->fc_emf;
    esmo->sf_wc_inv = param2ctrl(init->W_base / wc);

    // 5. Protection Mechanisms Setup
    parameter_gt err_lim =
        (init->current_err_limit_pu > real2param(1.0e-3)) ? init->current_err_limit_pu : real2param(0.3);
    esmo->current_err_limit = param2ctrl(err_lim);

    esmo->diverge_limit = (uint32_t)(init->fault_time_ms * fs_safe / real2param(1000.0));
    if (esmo->diverge_limit < 1)
        esmo->diverge_limit = 1;

    // 6. Finalize Initialization
    ctl_clear_pmsm_esmo(esmo);
    ctl_enable_pmsm_esmo_emf_normalization(esmo);
    ctl_disable_pmsm_esmo(esmo);
}

/**
 * @brief Auto-tunes and populates the ESMO init structure using motor base parameters.
 * @details Translates physical motor parameters into the ESMO initialization format
 * and automatically calculates the optimal bandwidths and cutoff frequencies
 * for the back-EMF filter and the Angle Tracking Observer (ATO).
 * * @param[out] esmo_init Pointer to the ESMO init structure to be populated.
 * @param[in]  cur_init  Pointer to the generic motor and current loop base configuration.
 * @param[in]  flux_linkage Permanent magnet flux linkage in Webers (Wb).
 */
void ctl_autotune_esmo_init_from_mtr(ctl_pmsm_esmo_init_t* esmo_init, const mc_foc_init_t* cur_init,
                                     parameter_gt flux_linkage)
{
    /* Reject invalid pointers and unusable base frequencies. */
    if (esmo_init == 0 || cur_init == 0 || cur_init->fs < CTL_PARAM_CONST_EPSILON ||
        cur_init->freq_base < CTL_PARAM_CONST_EPSILON)
    {
        return;
    }

    // -------------------------------------------------------------------------
    // 1. Direct physical-parameter mapping
    // -------------------------------------------------------------------------
    esmo_init->Rs = cur_init->mtr_Rs;
    esmo_init->Ld = cur_init->mtr_Ld;
    esmo_init->Lq = cur_init->mtr_Lq;
    esmo_init->flux_linkage = flux_linkage;
    esmo_init->fs = cur_init->fs;

    // -------------------------------------------------------------------------
    // 2. Per-unit base-value conversion
    // -------------------------------------------------------------------------
    esmo_init->V_base = cur_init->v_base;
    esmo_init->I_base = cur_init->i_base;

    // Convert the nominal electrical frequency in hertz to angular speed in radians per second.
    esmo_init->W_base = CTL_PARAM_CONST_2PI * cur_init->freq_base;

    // -------------------------------------------------------------------------
    // 3. Observer execution and tuning heuristics
    // -------------------------------------------------------------------------

    // Back-EMF Low-Pass Filter Cutoff Frequency (fc_emf)
    // Keep the cutoff between one tenth and one twentieth of the sampling rate to reject switching chatter.
    esmo_init->fc_emf = cur_init->fs * CTL_PARAM_CONST_1_OVER_10;

    // ATO/PLL Tracking Loop Bandwidth (ato_bw_hz)
    // Use half of the nominal electrical frequency to balance noise rejection and acceleration tracking.
    esmo_init->ato_bw_hz = cur_init->freq_base * CTL_PARAM_CONST_1_OVER_2;

    // -------------------------------------------------------------------------
    // 4. Protection margins and limits
    // -------------------------------------------------------------------------

    // Allow short loss-of-lock events during large transients before declaring divergence.
    esmo_init->fault_time_ms = real2param(50.0);

    // Declare divergence when the current tracking error exceeds 30 percent of base current.
    esmo_init->current_err_limit_pu = real2param(0.3);

    // A five-percent boundary layer limits quasi-SMC chatter.
    esmo_init->z_margin_pu = CTL_PARAM_CONST_1_OVER_20;
}
