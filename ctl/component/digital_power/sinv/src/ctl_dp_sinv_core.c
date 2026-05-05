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

#include <ctl/component/digital_power/sinv/sinv_core.h>

/**
 * @brief Auto-tunes the SINV Core parameters based on rigorous analytical models.
 * @param[in,out] init Pointer to the init structure.
 */
void ctl_auto_tuning_sinv_core(ctl_sinv_core_init_t* init)
{
    // 1. Default Assignments for omitted tuning targets
    if (init->current_loop_bw <= 0.001f)
        init->current_loop_bw = init->fs / 15.0f;
    if (init->qpr_wi <= 0.001f)
        init->qpr_wi = 2.0f * 3.14159265f; // Standard 2 rad/s width

    if (init->vgrid_lead_steps <= 0.001f)
        init->vgrid_lead_steps = 1.5f; // Standard digital delay compensation
    if (init->v_out_max_pu <= 0.001f)
        init->v_out_max_pu = 1.0f;

    // 2. Analytical Parameter Derivation (PU Mapping)
    parameter_gt z_base = init->v_base / init->i_base;
    parameter_gt wc = 2.0f * 3.14159265f * init->current_loop_bw;

    // Kp Calculation (Plant Inductance dictates Proportional Gain)
    parameter_gt kp_si = init->L_ac * wc;
    init->kp_tuned = kp_si / z_base;

    // Kr Calculation for Fundamental (Pole-Zero Cancellation based on Resistance)
    parameter_gt kr_si = init->R_ac * wc;
    init->kr_fund_tuned = kr_si / z_base;

    // Safety fallback if R_ac is extremely small or zero
    if (init->kr_fund_tuned < (init->kp_tuned * 0.1f))
    {
        init->kr_fund_tuned = init->kp_tuned * 5.0f;
    }

    // Heuristic: Harmonic QPRs typically require slightly less gain to maintain stability margin
    init->kr_harm_tuned = init->kr_fund_tuned * 0.5f;
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
    ctl_init_qpr_controller(&core->qpr_base, float2ctrl(init->kp_tuned), float2ctrl(init->kr_fund_tuned),
                            float2ctrl(init->freq_grid), float2ctrl(init->qpr_wi), float2ctrl(init->fs));

    // 2. Init Harmonic QPRs (Kp MUST be 0 to prevent proportional gain stacking)
    // Resonance frequency = freq_grid * Harmonic_Order
    ctl_init_qpr_controller(&core->qpr_h3, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 3.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));
    ctl_init_qpr_controller(&core->qpr_h5, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 5.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));
    ctl_init_qpr_controller(&core->qpr_h7, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 7.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));
    ctl_init_qpr_controller(&core->qpr_h9, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 9.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));
    ctl_init_qpr_controller(&core->qpr_h11, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 11.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));
    ctl_init_qpr_controller(&core->qpr_h13, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 13.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));
    ctl_init_qpr_controller(&core->qpr_h15, float2ctrl(0.0f), float2ctrl(init->kr_harm_tuned),
                            float2ctrl(init->freq_grid * 15.0f), float2ctrl(init->qpr_wi), float2ctrl(init->fs));

    // 3. Init Feedforward Lead Compensator
    parameter_gt vgrid_phase_delay = init->vgrid_lead_steps * (1.0f / init->fs) * init->freq_grid * 2.0f * 3.14159265f;
    ctl_init_lead_form3(&core->vgrid_lead, float2ctrl(vgrid_phase_delay), float2ctrl(init->freq_grid),
                        float2ctrl(init->fs));

    // 4. Apply Safe Limits
    core->v_out_max = float2ctrl(init->v_out_max_pu);

    // 5. Ensure everything is explicitly disabled upon init
    core->flag_enable_ctrl = 0;
    core->flag_enable_harm_ctrl = 0;
    core->flag_enable_lead_comp = 0;

    // Safety init for interface pointers
    core->v_grid_fdbk = NULL;
    core->v_bus_fdbk = NULL;
    core->i_fdbk = NULL;

    core->current_error = float2ctrl(0.0f);
    core->v_out_ref = float2ctrl(0.0f);
    core->isr_tick = 0;
}
