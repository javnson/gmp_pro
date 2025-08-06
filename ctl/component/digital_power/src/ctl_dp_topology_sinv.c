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

#include <ctl/component/digital_power/topology_preset/single_phase_dc_ac.h>

/**
 * @brief Configures and upgrades the internal components of the single-phase inverter controller.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @details This is the core initialization routine that sets up all sub-modules
 * based on the parameters provided in the init structure. It configures the PLL,
 * filters, the main QPR current controller, and multiple QR harmonic compensators.
 *
 * @param[out] sinv Pointer to the @ref sinv_ctrl_t instance to be configured.
 * @param[in] init Pointer to the @ref sinv_init_t structure containing all initialization parameters.
 */
void ctl_upgrade_sinv_param(sinv_ctrl_t* sinv, sinv_init_t* init)
{
    // --- Initialize grid synchronization modules ---
    ctl_init_single_phase_pll(&sinv->spll, init->pll_ctrl_kp, init->pll_ctrl_Ti, init->pll_ctrl_cut_freq,
                              init->base_freq, init->f_ctrl);
    ctl_init_ramp_gen_via_amp_freq(&sinv->rg, init->f_ctrl, init->base_freq, 1, 0);

    // --- Initialize sensor signal low-pass filters ---
    ctl_init_lp_filter(&sinv->lpf_idc, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_udc, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_il, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_igrid, init->f_ctrl, init->adc_filter_fc);
    ctl_init_lp_filter(&sinv->lpf_ugrid, init->f_ctrl, init->adc_filter_fc);

    // --- Initialize main controllers ---
    ctl_init_pid_ser(&sinv->voltage_pid, init->v_ctrl_kp, init->v_ctrl_Ti, init->v_ctrl_Td, init->f_ctrl);
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

/**
 * @brief Initializes the complete single-phase inverter controller.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @details This is the main entry point for initialization. It calls the upgrade
 * function to configure all parameters and then clears all runtime states.
 *
 * @param[out] sinv Pointer to the `sinv_ctrl_t` instance to be initialized.
 * @param[in] init Pointer to the `sinv_init_t` structure with all configuration parameters.
 */
void ctl_init_sinv_ctrl(sinv_ctrl_t* sinv, sinv_init_t* init)
{
    ctl_upgrade_sinv_param(sinv, init);
    ctl_clear_sinv(sinv);
    // Set a default power factor of 1.0 on initialization.
    sinv->pf_set = 1;
}

/**
 * @brief Attaches physical ADC interfaces to the single-phase inverter controller.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 *
 * @param[out] sinv Pointer to the `sinv_ctrl_t` instance.
 * @param[in] udc Pointer to the ADC interface for the DC bus voltage.
 * @param[in] idc Pointer to the ADC interface for the DC bus current.
 * @param[in] il Pointer to the ADC interface for the inductor current.
 * @param[in] ugrid Pointer to the ADC interface for the grid voltage.
 * @param[in] igrid Pointer to the ADC interface for the grid current.
 */
void ctl_attach_sinv_with_adc(sinv_ctrl_t* sinv, adc_ift* udc, adc_ift* idc, adc_ift* il, adc_ift* ugrid,
                              adc_ift* igrid)
{
    sinv->adc_idc = idc;
    sinv->adc_udc = udc;
    sinv->adc_il = il;
    sinv->adc_igrid = igrid;
    sinv->adc_ugrid = ugrid;
}

/**
 * @}
 */
