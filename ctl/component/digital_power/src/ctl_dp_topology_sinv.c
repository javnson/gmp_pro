/**
 * @file ctl_dp_topology_sinv.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements the initialization for a preset single-phase inverter topology.
 * @version 0.2
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Functions for initializing and configuring a comprehensive single-phase inverter
 * controller, including advanced features like harmonic compensation.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Single-Phase Inverter Control
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/single_phase/single_phase_dc_ac.h>

void ctl_upgrade_sinv_param(sinv_ctrl_t* sinv, sinv_init_t* init)
{
    // --- Initialize grid synchronization modules ---
    ctl_init_single_phase_pll(&sinv->spll, init->pll_ctrl_kp, init->pll_ctrl_Ti, init->pll_ctrl_cut_freq,
                              init->base_freq, init->f_ctrl);
    ctl_init_ramp_generator_via_freq(&sinv->rg, init->f_ctrl, init->base_freq, 1, 0);

    // --- Initialize sensor signal low-pass filters ---
    ctl_init_lp_filter(&sinv->lpf_idc, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_udc, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_il, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_igrid, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_ugrid, init->f_ctrl, init->adc_filter_fc);

    // --- Initialize main controllers ---
    ctl_init_pid_Tmode(&sinv->voltage_pid, init->v_ctrl_kp, init->v_ctrl_Ti, init->v_ctrl_Td, init->f_ctrl);
    ctl_init_qpr_controller(&sinv->sinv_qpr_base, init->i_ctrl_kp, init->i_ctrl_kr, init->base_freq,
                            init->i_ctrl_cut_freq, init->f_ctrl);

    // --- Initialize harmonic compensation (Quasi-Resonant controllers) ---
    ctl_init_qr_controller(&sinv->sinv_qr_3, init->harm_ctrl_kr_3, init->base_freq * 3.0f, init->harm_ctrl_cut_freq_3,
                           init->f_ctrl);
    ctl_init_qr_controller(&sinv->sinv_qr_5, init->harm_ctrl_kr_5, init->base_freq * 5.0f, init->harm_ctrl_cut_freq_5,
                           init->f_ctrl);
    ctl_init_qr_controller(&sinv->sinv_qr_7, init->harm_ctrl_kr_7, init->base_freq * 7.0f, init->harm_ctrl_cut_freq_7,
                           init->f_ctrl);
    ctl_init_qr_controller(&sinv->sinv_qr_9, init->harm_ctrl_kr_9, init->base_freq * 9.0f, init->harm_ctrl_cut_freq_9,
                           init->f_ctrl);

    // --- Initialize AC signal measurement modules ---
    ctl_init_sine_analyzer(&sinv->ac_current_measure, 0.01f, init->base_freq * 0.8f, init->base_freq * 1.2f,
                           init->base_freq, init->f_ctrl);
    ctl_init_sine_analyzer(&sinv->ac_voltage_measure, 0.01f, init->base_freq * 0.8f, init->base_freq * 1.2f,
                           init->base_freq, init->f_ctrl);
}

void ctl_init_sinv_ctrl(sinv_ctrl_t* sinv, sinv_init_t* init)
{
    ctl_upgrade_sinv_param(sinv, init);
    ctl_clear_sinv(sinv);
    // Set a default power factor of 1.0 on initialization.
    sinv->pf_set = 1;
}

void ctl_attach_sinv_with_adc(sinv_ctrl_t* sinv, adc_ift* _udc, adc_ift* _idc, adc_ift* il, adc_ift* ugrid,
                              adc_ift* igrid)
{
    sinv->adc_idc = _idc;
    sinv->adc_udc = _udc;
    sinv->adc_il = il;
    sinv->adc_igrid = igrid;
    sinv->adc_ugrid = ugrid;
}
