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

#include <ctl/math_block/gmp_math.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Single-Phase Inverter Control
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/sinv/sinv_core.h>

/**
 * @brief Auto-tunes the SINV Core parameters based on rigorous analytical models.
 * @param[in,out] init Pointer to the init structure.
 */
void ctl_auto_tuning_sinv_core(ctl_sinv_core_init_t* init)
{
    // 1. Default Assignments for omitted tuning targets
    if (init->current_loop_bw <= real2param(1.0e-3))
        init->current_loop_bw = init->fs / CTL_PARAM_CONST_15;
    if (init->qpr_wi <= real2param(1.0e-3))
        init->qpr_wi = CTL_PARAM_CONST_2; // Default resonant cutoff frequency: 2 Hz.

    if (init->vgrid_lead_steps <= real2param(1.0e-3))
        init->vgrid_lead_steps = CTL_PARAM_CONST_3_OVER_2; // Standard digital delay compensation.
    if (init->v_out_max_pu <= real2param(1.0e-3))
        init->v_out_max_pu = CTL_PARAM_CONST_1;

    // 2. Analytical Parameter Derivation (PU Mapping)
    parameter_gt z_base = init->v_base / init->i_base;
    parameter_gt wc = CTL_PARAM_CONST_2PI * init->current_loop_bw;

    // Kp Calculation (Plant Inductance dictates Proportional Gain)
    parameter_gt kp_si = init->L_ac * wc;
    init->kp_tuned = kp_si / z_base;

    // Kr Calculation for Fundamental (Pole-Zero Cancellation based on Resistance)
    parameter_gt kr_si = init->R_ac * wc;
    init->kr_fund_tuned = kr_si / z_base;

    // Safety fallback if R_ac is extremely small or zero
    if (init->kr_fund_tuned < (init->kp_tuned * CTL_PARAM_CONST_1_OVER_10))
    {
        init->kr_fund_tuned = init->kp_tuned * CTL_PARAM_CONST_5;
    }

    // Harmonic QR branches typically use less gain to preserve the stability margin.
    init->kr_harm_tuned = init->kr_fund_tuned * CTL_PARAM_CONST_1_OVER_2;
}

/**
 * @brief Initializes the SINV Core using the tuned parameters.
 * @note Unlike sinv_rc_core, this does NOT require an external buffer injection.
 * @param[out] core Pointer to the core structure.
 * @param[in]  init Pointer to the populated and tuned init structure.
 */
void ctl_init_sinv_core(ctl_sinv_core_t* core, const ctl_sinv_core_init_t* init)
{
    // 1. Init Fundamental QPR (Kp is only applied here)
    ctl_init_qpr_controller(&core->qpr_base, init->kp_tuned, init->kr_fund_tuned, init->freq_grid, init->qpr_wi,
                            init->fs);

    // 2. Initialize pure QR harmonic branches. No Kp storage or calculation is needed.
    ctl_init_qr_controller(&core->qr_h3, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_3, init->qpr_wi,
                           init->fs);
    ctl_init_qr_controller(&core->qr_h5, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_5, init->qpr_wi,
                           init->fs);
    ctl_init_qr_controller(&core->qr_h7, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_7, init->qpr_wi,
                           init->fs);
    ctl_init_qr_controller(&core->qr_h9, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_9, init->qpr_wi,
                           init->fs);
    ctl_init_qr_controller(&core->qr_h11, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_11, init->qpr_wi,
                           init->fs);
    ctl_init_qr_controller(&core->qr_h13, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_13, init->qpr_wi,
                           init->fs);
    ctl_init_qr_controller(&core->qr_h15, init->kr_harm_tuned, init->freq_grid * CTL_PARAM_CONST_15, init->qpr_wi,
                           init->fs);

    // 3. Init Feedforward Lead Compensator
    parameter_gt vgrid_phase_delay =
        init->vgrid_lead_steps * (CTL_PARAM_CONST_1 / init->fs) * init->freq_grid * CTL_PARAM_CONST_2PI;
    ctl_init_lead_form3(&core->vgrid_lead, vgrid_phase_delay, init->freq_grid, init->fs);

    // 4. Apply Safe Limits
    core->v_out_max = param2ctrl(init->v_out_max_pu);

    // 5. Ensure everything is explicitly disabled upon init
    core->flag_enable_ctrl = 0;
    core->flag_enable_harm_ctrl = 0;
    core->flag_enable_lead_comp = 0;

    // Safety init for interface pointers
    core->v_grid_fdbk = NULL;
    core->v_bus_fdbk = NULL;
    core->i_fdbk = NULL;

    core->current_error = CTL_CTRL_CONST_ZERO;
    core->v_out_ref = CTL_CTRL_CONST_ZERO;
    core->isr_tick = 0;
}
