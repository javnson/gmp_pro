#include <gmp_core.h>

#include <ctl/component/digital_power/sinv/sinv_rc_core.h>

/**
 * @brief Auto-tunes the SINV RC parameters based on rigorous analytical models.
 * @param[in,out] init Pointer to the init structure.
 */
void ctl_auto_tuning_sinv_rc(ctl_sinv_rc_init_t* init)
{
    // 1. Default Assignments for omitted tuning targets
    if (init->current_loop_bw <= 0.001f)
        init->current_loop_bw = init->fs / 15.0f;
    if (init->qpr_wi <= 0.001f)
        init->qpr_wi = 2.0f * 3.14159265f;

    if (init->fdrc_q_fc <= 0.001f)
        init->fdrc_q_fc = init->fs / 10.0f; // e.g., 2000Hz for 20kHz fs
    if (init->fdrc_lead_steps <= 0.001f)
        init->fdrc_lead_steps = 3.0f; // Common plant delay compensation
    if (init->vgrid_lead_steps <= 0.001f)
        init->vgrid_lead_steps = 1.5f; // Standard digital delay compensation

    if (init->err_lpf_fc <= 0.001f)
        init->err_lpf_fc = init->freq_grid; // Smooth out 1 fundamental cycle
    if (init->v_out_max_pu <= 0.001f)
        init->v_out_max_pu = 1.0f;

    // 2. Analytical Parameter Derivation (PU Mapping)
    parameter_gt z_base = init->v_base / init->i_base;
    parameter_gt wc = 2.0f * 3.14159265f * init->current_loop_bw;

    // Kp Calculation (Plant Inductance dictates Proportional Gain)
    parameter_gt kp_si = init->L_ac * wc;
    init->kp_tuned = kp_si / z_base;

    // Kr Calculation (Plant Resistance dictates Integrator/Resonant Gain via Pole-Zero Cancellation)
    // Mapped from dq-frame PI (Ki = R * wc) to alpha-beta PR
    parameter_gt kr_si = init->R_ac * wc;
    init->kr_tuned = kr_si / z_base;

    // Safety fallback if R_ac is extremely small or zero
    if (init->kr_tuned < (init->kp_tuned * 0.1f))
    {
        init->kr_tuned = init->kp_tuned * 5.0f; // Heuristic fallback
    }

    // 3. Robustness Thresholds
    init->fdrc_gain = 0.5f;      // Universal stable learning rate
    init->err_threshold = 0.05f; // 5% of I_base triggers FDRC freeze
}

/**
 * @brief Initializes the SINV RC core using the tuned parameters.
 * @param[out] core Pointer to the core structure.
 * @param[in]  init Pointer to the populated and tuned init structure.
 * @param[in]  rc_buffer Pointer to the pre-allocated memory array for the FDRC delay line.
 * @param[in]  rc_buf_capacity The total number of elements allocated in the rc_buffer.
 */
void ctl_init_sinv_rc_core(ctl_sinv_rc_core_t* core, const ctl_sinv_rc_init_t* init, ctrl_gt* rc_buffer,
                           uint32_t rc_buf_capacity)
{
    // 1. Init QPR
    ctl_init_qpr_controller(&core->qpr_ctrl, float2ctrl(init->kp_tuned), float2ctrl(init->kr_tuned),
                            float2ctrl(init->freq_grid), float2ctrl(init->qpr_wi), float2ctrl(init->fs));

    // 2. Init FDRC (Memory Buffer Injected here)
    ctl_init_fdrc(&core->fdrc_ctrl, rc_buffer, rc_buf_capacity, init->fs, init->freq_grid, init->fdrc_q_fc,
                  init->fdrc_gain, (int32_t)init->fdrc_lead_steps);

    // 3. Init Feedforward Lead Compensator
    // Calculate phase lag angle caused by digital delay: Theta = Steps * Ts * W_grid
    parameter_gt vgrid_phase_delay = init->vgrid_lead_steps * (1.0f / init->fs) * init->freq_grid * 2.0f * 3.14159265f;
    ctl_init_lead_form3(&core->vgrid_lead, float2ctrl(vgrid_phase_delay), float2ctrl(init->freq_grid),
                        float2ctrl(init->fs));

    // 4. Init Transient Error Filter
    ctl_init_filter_iir1_lpf(&core->err_filter, init->fs, init->err_lpf_fc);

    // 5. Apply Safe Limits & Thresholds
    core->v_out_max = float2ctrl(init->v_out_max_pu);
    core->fdrc_err_th = float2ctrl(init->err_threshold);

    // 6. Ensure everything is explicitly disabled upon init
    core->flag_enable_ctrl = 0;
    core->flag_enable_fdrc = 0;
    core->flag_enable_lead_comp = 0;

    // Clear history states
    ctl_clear_qpr_controller(&core->qpr_ctrl);
    ctl_clear_lead(&core->vgrid_lead);
    ctl_clear_filter_iir1(&core->err_filter);

    core->current_error = float2ctrl(0.0f);
    core->error_lpf_abs = float2ctrl(0.0f);
    core->v_out_ref = float2ctrl(0.0f);
    core->isr_tick = 0;
}
