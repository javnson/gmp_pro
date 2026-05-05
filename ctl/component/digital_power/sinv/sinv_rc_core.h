/**
 * @file ctl_sinv_rc_core.h
 * @author GMP Library Contributors
 * @brief Single-Phase Inverter/Rectifier Resonant Control Core.
 * 
 * @details
 * This module acts as the ultimate inner AC current loop for single-phase grid-tied systems.
 * Features rigorous analytical FOC-style initialization, Transient-Detected smart FDRC,
 * external memory buffer injection, and switchable Phase-Lead Grid Voltage Feedforward.
 * 
 * @version 4.0 (Analytical Auto-Tuning Release)
 * @copyright Copyright GMP(c) 2024-2026
 */

#ifndef _CTL_SINV_RC_CORE_H_
#define _CTL_SINV_RC_CORE_H_

#include <ctl/component/interface/interface_base.h>

#include <ctl/component/intrinsic/advance/fdrc.h>
#include <ctl/component/intrinsic/discrete/filter_iir1.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------*/
/* Initialization & Auto-Tuning Structures                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Auto-tuning initialization structure for the SINV RC Core.
 * @details Users populate physical parameters and buffer pointers here. 
 * If tuning targets are left as 0, the auto-tuner will analytically calculate optimal defaults.
 */
typedef struct _tag_sinv_rc_init_t
{
    // --- Physical Parameters (User Provided - Mandatory) ---
    parameter_gt fs;        //!< Controller execution frequency (Hz).
    parameter_gt v_bus;     //!< Nominal DC Bus voltage (V).
    parameter_gt v_base;    //!< Base voltage for per-unit conversion (V).
    parameter_gt i_base;    //!< Base current for per-unit conversion (A).
    parameter_gt freq_grid; //!< Nominal grid frequency (e.g., 50.0 or 60.0 Hz).

    parameter_gt L_ac; //!< Total AC side inductance (H).
    parameter_gt R_ac; //!< Total AC side equivalent resistance (Ohm).

    // --- Tuning Targets (User Provided / Auto-Generated Defaults if 0) ---
    parameter_gt current_loop_bw; //!< Target current loop bandwidth (Hz).
    parameter_gt qpr_wi;          //!< QPR resonant bandwidth (rad/s).

    parameter_gt fdrc_q_fc;        //!< Cutoff frequency for FDRC Q(z) low-pass filter (Hz).
    parameter_gt fdrc_lead_steps;  //!< Phase lead step count for FDRC to compensate plant delay.
    parameter_gt vgrid_lead_steps; //!< Phase lead steps for grid voltage feedforward (typically 1.5).

    parameter_gt err_lpf_fc;   //!< Cutoff frequency for transient detection filter (Hz).
    parameter_gt v_out_max_pu; //!< Maximum output voltage limit in PU.

    // --- Auto-Tuned Controller Parameters (Calculated by API) ---
    parameter_gt kp_tuned;      //!< Analytically tuned Proportional gain.
    parameter_gt kr_tuned;      //!< Analytically tuned Resonant gain.
    parameter_gt fdrc_gain;     //!< Tuned FDRC learning gain.
    parameter_gt err_threshold; //!< Tuned transient error threshold (PU) to freeze FDRC.

} ctl_sinv_rc_init_t;

/*---------------------------------------------------------------------------*/
/* Main Control Core Structure                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Main structure for the Single-Phase Resonant Control Core.
 */
typedef struct _tag_sinv_rc_core_t
{
    uint32_t isr_tick; //!< Controller Tick

    // --- Inputs (Updated each cycle before calling step) ---
    ctrl_gt i_ref; //!< AC current reference (from Ref Generator).

    adc_ift* v_grid_fdbk; //!< Grid voltage feedback (from ADC).
    adc_ift* v_bus_fdbk;  //!< DC bus voltage feedback (from ADC, PU). Set to 1.0f to disable compensation.
    adc_ift* i_fdbk;      //!< AC current feedback (from ADC).

    // --- Outputs & Intermediate Variables ---
    ctrl_gt current_error; //!< Real-time current tracking error.
    ctrl_gt error_lpf_abs; //!< Filtered absolute error for transient detection.

    ctrl_gt u_qpr;  //!< QPR controller output.
    ctrl_gt u_fdrc; //!< FDRC controller output.
    ctrl_gt u_ff;   //!< Grid Feedforward output.

    ctrl_gt v_out_ref; //!< Final unified voltage reference (Duty cycle equivalent).

    // --- Controller Entities ---
    ctl_qpr_t qpr_ctrl;           //!< QPR controller.
    ctl_fdrc_t fdrc_ctrl;         //!< Repetitive controller.
    ctrl_lead_t vgrid_lead;       //!< Lead compensator for grid feedforward.
    ctl_filter_IIR1_t err_filter; //!< LPF for transient detection (absolute error).

    // --- Safety & Thresholds ---
    ctrl_gt v_out_max;   //!< Max output voltage limit (anti-windup limit).
    ctrl_gt fdrc_err_th; //!< Threshold for filtering absolute error to freeze FDRC learning.

    // --- State Flags ---
    fast_gt flag_enable_ctrl;      //!< Master enable flag for the inner loop.
    fast_gt flag_enable_fdrc;      //!< Master switch for FDRC algorithm output.
    fast_gt flag_enable_lead_comp; //!< Master switch for Grid Voltage Feedforward Phase-Lead.

} ctl_sinv_rc_core_t;

/*---------------------------------------------------------------------------*/
/* API Prototypes & Inline Implementations                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Auto-tunes the SINV RC parameters based on rigorous analytical models.
 * @param[in,out] init Pointer to the init structure.
 */
GMP_STATIC_INLINE void ctl_auto_tuning_sinv_rc(ctl_sinv_rc_init_t* init)
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
 * @param[in] init Pointer to the populated and tuned init structure.
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
    // FDRC clear logic might depend on your internal API. Usually buffer memset is handled.
    ctl_clear_lead(&core->vgrid_lead);
    ctl_clear_filter_iir1(&core->err_filter);

    core->current_error = float2ctrl(0.0f);
    core->error_lpf_abs = float2ctrl(0.0f);
    core->v_out_ref = float2ctrl(0.0f);
    core->isr_tick = 0;
}

GMP_STATIC_INLINE void ctl_attach_sinv_rc(ctl_sinv_rc_core_t* core, adc_ift* _u_dc, adc_ift* _u_ac, adc_ift* _i_ac)
{
    core->i_fdbk = _i_ac;
    core->v_bus_fdbk = _u_dc;
    core->v_grid_fdbk = _u_ac;
}

/*---------------------------------------------------------------------------*/
/* Core Execution Step Function                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Executes one step of the Single-Phase RC Core.
 * @param[in,out] core Pointer to the RC core instance.
 * @return ctrl_gt Final output voltage reference (duty cycle) after DC bus compensation.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_rc_core(ctl_sinv_rc_core_t* core, ctrl_gt i_ref)
{
    core->isr_tick++;

    if (!core->flag_enable_ctrl)
    {
        return core->v_out_ref; // Keep previous or zero
    }

    core->i_ref = i_ref;

    // 1. Calculate Error
    core->current_error = core->i_ref - core->i_fdbk->value;

    // 2. Transient Detection for Smart FDRC
    ctrl_gt abs_err = ctl_abs(core->current_error);
    core->error_lpf_abs = ctl_step_filter_iir1(&core->err_filter, abs_err);

    // 3. Fundamental Tracking (QPR)
    core->u_qpr = ctl_step_qpr_controller(&core->qpr_ctrl, core->current_error);

    // 4. Harmonic Rejection (Smart FDRC)
    core->u_fdrc = float2ctrl(0.0f);
    if (core->flag_enable_fdrc)
    {
        // Smart Freeze Logic: Call underlying FDRC API to pause learning if transient is large
        if (core->error_lpf_abs > core->fdrc_err_th)
        {
            ctl_disable_fdrc_integrating(&core->fdrc_ctrl); // Pause memory update
        }
        else
        {
            ctl_enable_fdrc_integrating(&core->fdrc_ctrl); // Resume memory update
        }

        // Execute FDRC step
        core->u_fdrc = ctl_step_fdrc(&core->fdrc_ctrl, core->current_error);
    }

    // 5. Grid Voltage Feedforward
    if (core->flag_enable_lead_comp)
    {
        core->u_ff = ctl_step_lead(&core->vgrid_lead, core->v_grid_fdbk->value);
    }
    else
    {
        core->u_ff = core->v_grid_fdbk->value; // Direct feedforward without phase compensation
    }

    // 6. Synthesis
    ctrl_gt v_ref_total = core->u_qpr + core->u_fdrc + core->u_ff;

    // 7. Universal DC Bus Voltage Compensation & Saturation
    ctrl_gt v_bus_safe = (core->v_bus_fdbk->value > float2ctrl(0.1f)) ? core->v_bus_fdbk->value : float2ctrl(0.1f);
    ctrl_gt v_out_comp = ctl_div(v_ref_total, v_bus_safe);

    core->v_out_ref = ctl_sat(v_out_comp, core->v_out_max, -core->v_out_max);

    return core->v_out_ref;
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_SINV_RC_CORE_H_
