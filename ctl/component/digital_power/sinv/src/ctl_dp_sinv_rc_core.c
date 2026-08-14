#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/sinv/sinv_rc_core.h>

/**
 * @brief Auto-tunes the SINV RC parameters based on rigorous analytical models.
 * @param[in,out] init Pointer to the init structure.
 */
void ctl_auto_tuning_sinv_rc(ctl_sinv_rc_init_t* init)
{
    // 1. Default Assignments for omitted tuning targets
    if (init->current_loop_bw <= real2param(1.0e-3))
        init->current_loop_bw = init->fs / CTL_PARAM_CONST_15;
    if (init->qpr_wi <= real2param(1.0e-3))
        init->qpr_wi = CTL_PARAM_CONST_2;

    if (init->fdrc_q_fc <= real2param(1.0e-3))
        init->fdrc_q_fc = init->fs / CTL_PARAM_CONST_10; // For example, 2 kHz at a 20 kHz sample rate.
    if (init->fdrc_min_freq <= real2param(1.0e-3))
        init->fdrc_min_freq = init->freq_grid * CTL_PARAM_CONST_9_OVER_10;
    if (init->fdrc_lead_steps <= real2param(1.0e-3))
        init->fdrc_lead_steps = CTL_PARAM_CONST_3; // Common plant-delay compensation.
    if (init->vgrid_lead_steps <= real2param(1.0e-3))
        init->vgrid_lead_steps = CTL_PARAM_CONST_3_OVER_2; // Standard digital-delay compensation.

    if (init->err_lpf_fc <= real2param(1.0e-3))
        init->err_lpf_fc = init->freq_grid; // Smooth over one fundamental cycle.
    if (init->v_out_max_pu <= real2param(1.0e-3))
        init->v_out_max_pu = CTL_PARAM_CONST_1;

    // 2. Analytical Parameter Derivation (PU Mapping)
    parameter_gt z_base = init->v_base / init->i_base;
    parameter_gt wc = CTL_PARAM_CONST_2PI * init->current_loop_bw;

    // Kp Calculation (Plant Inductance dictates Proportional Gain)
    parameter_gt kp_si = init->L_ac * wc;
    init->kp_tuned = kp_si / z_base;

    // Kr Calculation (Plant Resistance dictates Integrator/Resonant Gain via Pole-Zero Cancellation)
    // Mapped from dq-frame PI (Ki = R * wc) to alpha-beta PR
    parameter_gt kr_si = init->R_ac * wc;
    init->kr_tuned = kr_si / z_base;

    // Safety fallback if R_ac is extremely small or zero
    if (init->kr_tuned < (init->kp_tuned * CTL_PARAM_CONST_1_OVER_10))
    {
        init->kr_tuned = init->kp_tuned * CTL_PARAM_CONST_5; // Heuristic fallback.
    }

    // 3. Robustness Thresholds
    if (init->fdrc_gain <= real2param(1.0e-3))
        init->fdrc_gain = CTL_PARAM_CONST_1_OVER_10;
    if (init->err_threshold <= real2param(1.0e-3))
        init->err_threshold = CTL_PARAM_CONST_1_OVER_20;
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
    ctl_init_qpr_controller(&core->qpr_ctrl, init->kp_tuned, init->kr_tuned, init->freq_grid, init->qpr_wi,
                            init->fs);

    // 2. Init FDRC (Memory Buffer Injected here)
    ctl_init_fdrc(&core->fdrc_ctrl, rc_buffer, rc_buf_capacity, init->fs, init->fdrc_min_freq, init->fdrc_q_fc,
                  init->fdrc_gain, (int32_t)init->fdrc_lead_steps);

    // 3. Init Feedforward Lead Compensator
    // Calculate phase lag angle caused by digital delay: Theta = Steps * Ts * W_grid
    parameter_gt vgrid_phase_delay =
        init->vgrid_lead_steps * (CTL_PARAM_CONST_1 / init->fs) * init->freq_grid * CTL_PARAM_CONST_2PI;
    ctl_init_lead_form3(&core->vgrid_lead, vgrid_phase_delay, init->freq_grid, init->fs);

    // 4. Init Transient Error Filter
    ctl_init_filter_iir1_lpf(&core->err_filter, init->fs, init->err_lpf_fc);

    // 5. Apply Safe Limits & Thresholds
    core->v_out_max = param2ctrl(init->v_out_max_pu);
    core->fdrc_err_th = param2ctrl(init->err_threshold);
    ctl_set_sinv_rc_fundamental_frequency(core, init->freq_grid);

    // 6. Ensure everything is explicitly disabled upon init
    core->flag_enable_ctrl = 0;
    core->flag_enable_fdrc = 0;
    core->flag_enable_lead_comp = 0;

    // Clear all history states and outputs.
    ctl_clear_sinv_rc_core(core);
}
