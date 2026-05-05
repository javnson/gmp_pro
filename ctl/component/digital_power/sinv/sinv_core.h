/**
 * @file single_phase_dc_ac.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset single-phase DC/AC inverter controller.
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2025
 */

/**
 * @defgroup CTL_TOPOLOGY_SINV_CORE_API Single-Phase Inverter Core (Multi-QPR) API
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines a lightweight, purely QPR-based inner current loop core for single-phase inverters.
 * @details This module replaces the advanced Repetitive Control (FDRC) with a parallel 
 * bank of Quasi-Proportional-Resonant (QPR) controllers targeting specific odd harmonics 
 * (3rd, 5th, 7th, 9th, 11th, 13th, 15th). It provides identical interface bindings to 
 * `sinv_rc_core` for seamless swapping.
 */

#ifndef _FILE_CTL_SINV_CORE_H_
#define _FILE_CTL_SINV_CORE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

/*---------------------------------------------------------------------------*/
/* Initialization & Auto-Tuning Structures                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Auto-tuning initialization structure for the Multi-QPR SINV Core.
 */
typedef struct _tag_sinv_core_init_t
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
    parameter_gt current_loop_bw;  //!< Target current loop bandwidth (Hz).
    parameter_gt qpr_wi;           //!< QPR resonant bandwidth (rad/s).
    parameter_gt vgrid_lead_steps; //!< Phase lead steps for grid voltage feedforward (typically 1.5).
    parameter_gt v_out_max_pu;     //!< Maximum output voltage limit in PU.

    // --- Auto-Tuned Controller Parameters (Calculated by API) ---
    parameter_gt kp_tuned;      //!< Analytically tuned Proportional gain.
    parameter_gt kr_fund_tuned; //!< Analytically tuned Resonant gain for the fundamental frequency.
    parameter_gt kr_harm_tuned; //!< Analytically tuned Resonant gain for harmonic frequencies.

} ctl_sinv_core_init_t;

/*---------------------------------------------------------------------------*/
/* Main Control Core Structure                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Main structure for the Multi-QPR Control Core.
 */
typedef struct _tag_sinv_core_t
{
    uint32_t isr_tick; //!< Controller Tick counter.

    // --- Inputs (Updated each cycle before calling step) ---
    ctrl_gt i_ref; //!< AC current reference (from Ref Generator).

    // --- Hardware Interface Bindings (Zero-copy Pointers) ---
    adc_ift* v_grid_fdbk; //!< Grid voltage feedback interface pointer (from ADC).
    adc_ift* v_bus_fdbk;  //!< DC bus voltage feedback interface pointer (from ADC, PU).
    adc_ift* i_fdbk;      //!< AC current feedback interface pointer (from ADC).

    // --- Outputs & Intermediate Variables ---
    ctrl_gt current_error; //!< Real-time current tracking error.

    ctrl_gt u_qpr_base; //!< Fundamental QPR output.
    ctrl_gt u_qpr_harm; //!< Sum of all harmonic QPR outputs.
    ctrl_gt u_ff;       //!< Grid Feedforward output.

    ctrl_gt v_out_ref; //!< Final unified voltage reference (Duty cycle equivalent).

    // --- Controller Entities (Parallel Bank) ---
    ctl_qpr_t qpr_base; //!< Fundamental (1st) tracking.
    ctl_qpr_t qpr_h3;   //!< 3rd Harmonic rejection.
    ctl_qpr_t qpr_h5;   //!< 5th Harmonic rejection.
    ctl_qpr_t qpr_h7;   //!< 7th Harmonic rejection.
    ctl_qpr_t qpr_h9;   //!< 9th Harmonic rejection.
    ctl_qpr_t qpr_h11;  //!< 11th Harmonic rejection.
    ctl_qpr_t qpr_h13;  //!< 13th Harmonic rejection.
    ctl_qpr_t qpr_h15;  //!< 15th Harmonic rejection.

    ctrl_lead_t vgrid_lead; //!< Lead compensator for grid feedforward.

    // --- Safety & Thresholds ---
    ctrl_gt v_out_max; //!< Max output voltage limit (anti-windup limit).

    // --- State Flags ---
    fast_gt flag_enable_ctrl;      //!< Master enable flag for the inner loop.
    fast_gt flag_enable_harm_ctrl; //!< Master switch for Harmonic QPRs (Replaces flag_enable_fdrc).
    fast_gt flag_enable_lead_comp; //!< Master switch for Grid Voltage Feedforward Phase-Lead.

} ctl_sinv_core_t;

/*---------------------------------------------------------------------------*/
/* API Prototypes & Inline Implementations                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Auto-tunes the SINV Core parameters based on rigorous analytical models.
 * @param[in,out] init Pointer to the init structure.
 */
void ctl_auto_tuning_sinv_core(ctl_sinv_core_init_t* init);

/**
 * @brief Initializes the SINV Core using the tuned parameters.
 * @note Unlike sinv_rc_core, this does NOT require an external buffer injection.
 * @param[out] core Pointer to the core structure.
 * @param[in]  init Pointer to the populated and tuned init structure.
 */
void ctl_init_sinv_core(ctl_sinv_core_t* core, const ctl_sinv_core_init_t* init);

/**
 * @brief Binds ADC interfaces to the Core for zero-copy data fetching.
 * @note Mirrors `ctl_attach_sinv_rc`.
 */
GMP_STATIC_INLINE void ctl_attach_sinv_core(ctl_sinv_core_t* core, adc_ift* _u_dc, adc_ift* _u_ac, adc_ift* _i_ac)
{
    core->i_fdbk = _i_ac;
    core->v_bus_fdbk = _u_dc;
    core->v_grid_fdbk = _u_ac;
}

/**
 * @brief Clears history states of all internal filters and controllers.
 */
GMP_STATIC_INLINE void ctl_clear_sinv_core(ctl_sinv_core_t* core)
{
    ctl_clear_qpr_controller(&core->qpr_base);
    ctl_clear_qpr_controller(&core->qpr_h3);
    ctl_clear_qpr_controller(&core->qpr_h5);
    ctl_clear_qpr_controller(&core->qpr_h7);
    ctl_clear_qpr_controller(&core->qpr_h9);
    ctl_clear_qpr_controller(&core->qpr_h11);
    ctl_clear_qpr_controller(&core->qpr_h13);
    ctl_clear_qpr_controller(&core->qpr_h15);
    ctl_clear_lead(&core->vgrid_lead);

    core->current_error = float2ctrl(0.0f);
    core->v_out_ref = float2ctrl(0.0f);
}

/*---------------------------------------------------------------------------*/
/* Core Execution Step Function                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Executes one step of the Multi-QPR Single-Phase Core.
 * @details Replaces the FDRC algorithm with a parallel QPR bank.
 * 
 * @param[in,out] core Pointer to the core instance.
 * @param[in]     i_ref Instantaneous AC current reference command (PU).
 * @return ctrl_gt Final output voltage reference (duty cycle PU) after DC bus compensation.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_core(ctl_sinv_core_t* core, ctrl_gt i_ref)
{
    core->isr_tick++;

    if (!core->flag_enable_ctrl)
    {
        return core->v_out_ref; // Keep previous or zero
    }

    core->i_ref = i_ref;

    // 1. Calculate Error (Direct read from interface pointer)
    core->current_error = core->i_ref - core->i_fdbk->value;

    // 2. Fundamental Tracking (Base QPR - Proportional + Resonant)
    core->u_qpr_base = ctl_step_qpr_controller(&core->qpr_base, core->current_error);

    // 3. Harmonic Rejection (Parallel QPR Bank - Purely Resonant)
    core->u_qpr_harm = float2ctrl(0.0f);
    if (core->flag_enable_harm_ctrl)
    {
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h3, core->current_error);
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h5, core->current_error);
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h7, core->current_error);
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h9, core->current_error);
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h11, core->current_error);
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h13, core->current_error);
        core->u_qpr_harm += ctl_step_qpr_controller(&core->qpr_h15, core->current_error);
    }

    // 4. Grid Voltage Feedforward
    if (core->flag_enable_lead_comp)
    {
        core->u_ff = ctl_step_lead(&core->vgrid_lead, core->v_grid_fdbk->value);
    }
    else
    {
        core->u_ff = core->v_grid_fdbk->value; // Direct feedforward without phase compensation
    }

    // 5. Synthesis
    ctrl_gt v_ref_total = core->u_qpr_base + core->u_qpr_harm + core->u_ff;

    // 6. Universal DC Bus Voltage Compensation & Saturation
    ctrl_gt v_bus_safe = (core->v_bus_fdbk->value > float2ctrl(0.1f)) ? core->v_bus_fdbk->value : float2ctrl(0.1f);
    ctrl_gt v_out_comp = ctl_div(v_ref_total, v_bus_safe);

    core->v_out_ref = ctl_sat(v_out_comp, core->v_out_max, -core->v_out_max);

    return core->v_out_ref;
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_SINV_CORE_H_

/**
 * @}
 */
