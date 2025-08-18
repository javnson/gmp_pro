/**
 * @file ctl_dp_topology_inv.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements the initialization for a preset three-phase inverter topology.
 * @version 0.2
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Functions for initializing and configuring a comprehensive three-phase inverter
 * controller, including advanced features like harmonic compensation and droop control.
 */

#include <gmp_core.h>
#include <math.h>

#include <ctl/component/digital_power/three_phase/three_phase_dc_ac.h>

//////////////////////////////////////////////////////////////////////////
// Three-Phase Inverter Control
//////////////////////////////////////////////////////////////////////////

void ctl_upgrade_three_phase_inv(inv_ctrl_t* inv, three_phase_inv_init_t* init)
{
    // --- Initialize sensor signal low-pass filters ---
    ctl_init_lp_filter(&inv->lpf_udc, init->fs, init->adc_fc);
    ctl_init_lp_filter(&inv->lpf_idc, init->fs, init->adc_fc);
    for (int i = 0; i < 3; ++i)
    {
        ctl_init_lp_filter(&inv->lpf_vabc[i], init->fs, init->adc_fc);
        ctl_init_lp_filter(&inv->lpf_iabc[i], init->fs, init->adc_fc);
    }

    // --- Initialize grid synchronization modules ---
    ctl_init_pll_3ph(&inv->pll, init->freq_base, init->kp_pll_ctrl, init->Ti_pll_ctrl, 0, init->fs);
    ctl_init_ramp_generator_via_freq(&inv->rg, init->fs, init->freq_base, 1, 0);
    inv->rg_slope_default = inv->rg.slope;

    // --- Initialize main d-q axis PI controllers (positive sequence) ---
    ctl_init_pid_ser(&inv->voltage_ctrl[phase_d], init->kp_vd_ctrl, init->Ti_vd_ctrl, 0, init->fs);
    ctl_init_pid_ser(&inv->voltage_ctrl[phase_q], init->kp_vq_ctrl, init->Ti_vq_ctrl, 0, init->fs);
    ctl_init_pid_ser(&inv->current_ctrl[phase_d], init->kp_id_ctrl, init->Ti_id_ctrl, 0, init->fs);
    ctl_init_pid_ser(&inv->current_ctrl[phase_q], init->kp_iq_ctrl, init->Ti_iq_ctrl, 0, init->fs);

    // --- Initialize negative sequence d-q axis PI controllers ---
    ctl_init_pid_ser(&inv->neg_voltage_ctrl[phase_d], init->kp_vdn_ctrl, init->Ti_vdn_ctrl, 0, init->fs);
    ctl_init_pid_ser(&inv->neg_voltage_ctrl[phase_q], init->kp_vqn_ctrl, init->Ti_vqn_ctrl, 0, init->fs);
    ctl_init_pid_ser(&inv->neg_current_ctrl[phase_d], init->kp_idn_ctrl, init->Ti_idn_ctrl, 0, init->fs);
    ctl_init_pid_ser(&inv->neg_current_ctrl[phase_q], init->kp_iqn_ctrl, init->Ti_iqn_ctrl, 0, init->fs);

    // --- Initialize droop control parameters ---
    // CORRECTED: Droop parameters should be sourced from the 'init' struct.
    inv->kp_droop = float2ctrl(init->kp_droop);
    inv->kq_droop = float2ctrl(init->kq_droop);
    ctl_init_saturation(&inv->idq_droop_sat[phase_d], -init->id_lim_droop, init->id_lim_droop);
    ctl_init_saturation(&inv->idq_droop_sat[phase_q], -init->iq_lim_droop, init->iq_lim_droop);

    // --- Initialize harmonic compensation (Quasi-Resonant controllers) for 5th and 7th harmonics ---
    for (int i = 0; i < 2; ++i)
    {
        ctl_init_qr_controller(&inv->harm_qr_5[i], init->harm_ctrl_kr_5, init->freq_base * 5,
                               init->harm_ctrl_cut_freq_5, init->fs);
        ctl_init_qr_controller(&inv->harm_qr_7[i], init->harm_ctrl_kr_7, init->freq_base * 7,
                               init->harm_ctrl_cut_freq_7, init->fs);
    }

    // --- Initialize zero-sequence harmonic compensation (3rd and 9th) ---
    ctl_init_pid_ser(&inv->zero_pid, init->zero_ctrl_kp, init->zero_ctrl_Ti, 0, init->fs);
    ctl_init_qr_controller(&inv->zero_qr_3, init->zero_ctrl_kr_3, init->freq_base * 3, init->zero_ctrl_cut_freq_3,
                           init->fs);
    ctl_init_qr_controller(&inv->zero_qr_9, init->zero_ctrl_kr_9, init->freq_base * 9, init->zero_ctrl_cut_freq_9,
                           init->fs);

    // --- Set feed-forward and other parameters ---
    inv->omega_L = float2ctrl(CTL_PARAM_CONST_2PI * init->freq_base * init->Lf * init->i_base / init->v_base);
    inv->rg_freq_pu = float2ctrl(1.0);
}

void ctl_attach_three_phase_inv(inv_ctrl_t* inv, adc_ift* adc_udc, adc_ift* adc_idc, adc_ift* adc_ia, adc_ift* adc_ib,
                                adc_ift* adc_ic, adc_ift* adc_ua, adc_ift* adc_ub, adc_ift* adc_uc)
{
    inv->adc_udc = adc_udc;
    inv->adc_idc = adc_idc;

    inv->adc_iabc[phase_A] = adc_ia;
    inv->adc_iabc[phase_B] = adc_ib;
    inv->adc_iabc[phase_C] = adc_ic;

    inv->adc_vabc[phase_A] = adc_ua;
    inv->adc_vabc[phase_B] = adc_ub;
    inv->adc_vabc[phase_C] = adc_uc;
}

void ctl_init_three_phase_inv(inv_ctrl_t* inv, three_phase_inv_init_t* init)
{
    ctl_upgrade_three_phase_inv(inv, init);
    ctl_clear_three_phase_inv(inv);
}


