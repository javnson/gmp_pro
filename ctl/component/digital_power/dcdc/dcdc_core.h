/**
 * @file ctl_dcdc_core.h
 * @author GMP Library Contributors
 * @brief High-Performance DC-DC Core Supporting Decoupled Unidirectional & Bidirectional Pipeling.
 * @version 4.00
 * @date 2026-05-28
 *
 * @details Optimizations applied:
 * 1. Normalized both fault and fault_mask registers to use the visual bitfield union.
 * 2. Eliminated current direction tracking (HCC) overhead from unidirectional processing flows.
 * 3. Dedicated discrete atomic execution paths for bidirectional systems to update flow vectors.
 * 4. Maintained fully branchless inner loops execution for compiled release targets.
 *
 * @copyright Copyright GMP(c) 2026
 */

#ifndef _FILE_TXT_CTL_DCDC_CORE_H_
#define _FILE_TXT_CTL_DCDC_CORE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/hysteresis_controller.h>

/**
 * @brief 16-bit visual bitfield union for concurrent system diagnostics and masking masks.
 */
typedef union _tag_dcdc_fault {
    uint16_t all; //!< Complete register status word access for rapid masking
    struct
    {
        uint16_t vin_ovp : 1;      //!< Input over-voltage trip tracking bit
        uint16_t vin_uvp : 1;      //!< Input under-voltage trip tracking bit
        uint16_t vout_ovp : 1;     //!< Output over-voltage trip tracking bit
        uint16_t iload_ocp : 1;    //!< Output load over-current trip tracking bit
        uint16_t il_ocp : 1;       //!< Main power inductor over-current trip tracking bit
        uint16_t unknown_mode : 1; //!< controller is set to an unknown mode
        uint16_t reserved : 10;    //!< Prospective diagnostic expansion bits allocation
    } bits;
} dcdc_fault_t;

/**
 * @brief Unified operational cascade tracking modes configuration.
 */
typedef enum _tag_dcdc_mode
{
    DCDC_MODE_OPEN_LOOP = 0,       //!< Open-loop direct manipulation path
    DCDC_MODE_CURRENT_LOOP = 1,    //!< Constant-Current (CC) inner regulation path
    DCDC_MODE_VOLTAGE_LOOP = 2,    //!< Standard Constant-Voltage (CV) cascade path
    DCDC_MODE_PARALLEL_COMPETE = 3 //!< Parallel Competitive Loop (Common for LLC/PSFB plants)
} dcdc_mode_e;

/**
 * @brief Layered structured protection boundary parameters object.
 */
typedef struct _tag_dcdc_bounds
{
    ctrl_gt vin_max;   //!< High trip voltage limit for input supply rails (PU)
    ctrl_gt vin_min;   //!< Low trip voltage limit for input supply rails (PU)
    ctrl_gt vout_max;  //!< High trip voltage limit for output capacitive banks (PU)
    ctrl_gt iload_max; //!< High trip current limit for consumer terminals load (PU)
    ctrl_gt il_max;    //!< Absolute safe peak threshold for inductor current streams (PU)
} dcdc_bounds_t;

/**
 * @brief Universal Cascade Dual-Loop DC-DC Controller Context Structure.
 */
typedef struct _tag_dcdc_core_t
{
    uint32_t isr_tick; //!< Micro-architectural task scheduling pulse clock counter

    // --- Mode & Visual Diagnostic Status Register Maps ---
    dcdc_mode_e target_mode; //!< Configuration targeted routing directive
    dcdc_fault_t fault;      //!< Visual error bitfield register for diagnostics tracking
    dcdc_fault_t fault_mask; //!< Visual error bitfield register to block chosen diagnostic flags

    // --- Hardware Data Access Links (Zero-Copy Intercepts) ---
    adc_ift* v_in_fdbk;   //!< External reference link pointing to input voltage (PU)
    adc_ift* v_out_fdbk;  //!< External reference link pointing to output terminal voltage (PU)
    adc_ift* i_L_fdbk;    //!< External reference link pointing to main inductor current (PU)
    adc_ift* i_load_fdbk; //!< External reference link pointing to consumption load current (PU)

    // --- Signal Purifiers & Anti-aliasing Pre-processors ---
    ctl_low_pass_filter_t lpf_v_out; //!< Low-frequency behavior analyzer for output voltage tracking
    ctl_low_pass_filter_t lpf_i_L;   //!< High-attenuation signal cleaner for quadrant estimation tracking

    // --- Quadrant Estimators & Zero-Crossing Arbiters ---
    ctl_hysteresis_controller_t dir_detect; //!< Hysteresis core blocking bounce around zero-crossings
    fast_gt flag_direction;                 //!< 1 identifies forward/charging flow, 0 identifies reverse flow

    // --- The Three Fundamental Signal Command Interfaces ---
    ctrl_gt v_out_set_raw; //!< Var 1: User requested terminal voltage voltage reference index (PU)
    ctrl_gt i_out_set_raw; //!< Var 2: User specified or cascade generated inner current index (PU)
    ctrl_gt v_out_ff;      //!< Var 3: Direct baseline voltage modifier / open loop direct command (PU)

    // --- Zero-Overhead Extension Feedforward Connections ---
    ctrl_gt i_ff; //!< Branchless inner current loop predictive compensation hook (PU)
    ctrl_gt v_ff; //!< Branchless final loop nominal voltage predictive compensation hook (PU)

    // --- Dynamic Slew Profiles Tracking limiters ---
    ctl_slope_limiter_t v_ramp; //!< Trajectory pathway slope constraint engine for voltage (PU)
    ctl_slope_limiter_t i_ramp; //!< Trajectory pathway slope constraint engine for current (PU)

    // --- Loop Regulatory Control Processors ---
    ctl_pid_t v_loop_pi; //!< Outer terminal tracking loops compensator block
    ctl_pid_t i_loop_pi; //!< Inner reactive current loops compensator block

    // --- Layered Structural Limits ---
    dcdc_bounds_t bounds; //!< Boundaries definitions cluster object

    // --- Waveform Instrumentation Perspectives ---
    ctrl_gt v_out_ref; //!< Active reference track matching soft voltage limits (PU)
    ctrl_gt i_L_ref;   //!< Active reference track matching soft current limits (PU)
    ctrl_gt delta_v;   //!< PRIMARY OUTPUT: Processed regulatory compensation quantity (PU)

    // --- Execution Gate Controller Switches ---
    fast_gt flag_enable; //!< Processing kernel loop block validation gate switch

} ctl_dcdc_core_t;

/*---------------------------------------------------------------------------*/
/* 4. Lifecycles, Diagnostics & Config Services                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Populates boundary tracking limits inside the structural safety sub-object.
 */
GMP_STATIC_INLINE void ctl_set_dcdc_core_bounds(ctl_dcdc_core_t* core, ctrl_gt vin_max, ctrl_gt vin_min,
                                                ctrl_gt vout_max, ctrl_gt iload_max, ctrl_gt il_max)
{
    core->bounds.vin_max = vin_max;
    core->bounds.vin_min = vin_min;
    core->bounds.vout_max = vout_max;
    core->bounds.iload_max = iload_max;
    core->bounds.il_max = il_max;
}

/**
 * @brief Flushes all bits inside the error bitfield register back to healthy status.
 */
GMP_STATIC_INLINE void ctl_clear_dcdc_fault(ctl_dcdc_core_t* core)
{
    core->fault.all = DCDC_FAULT_BIT_NONE;
}

/**
 * @brief Rapid bitwise verification executing unmasked latched fault validation checks.
 * @return 1 if an unmasked system fault bit is high, 0 if system is healthy.
 */
GMP_STATIC_INLINE fast_gt ctl_get_dcdc_fault_status(const ctl_dcdc_core_t* core)
{
    return ((core->fault.all & core->fault_mask.all) != DCDC_FAULT_BIT_NONE) ? 1 : 0;
}

/**
 * @brief Universal safety windows checking processor.
 */
GMP_STATIC_INLINE uint16_t ctl_step_dcdc_protect_sub(ctl_dcdc_core_t* core)
{
    if (core->v_in_fdbk->value > core->bounds.vin_max)
        core->fault.bits.vin_ovp = 1;
    if (core->v_in_fdbk->value < core->bounds.vin_min)
        core->fault.bits.vin_uvp = 1;
    if (core->v_out_fdbk->value > core->bounds.vout_max)
        core->fault.bits.vout_ovp = 1;

    if (core->i_load_fdbk && (core->i_load_fdbk->value > core->bounds.iload_max))
        core->fault.bits.iload_ocp = 1;

    if (ctl_abs(core->i_L_fdbk->value) > core->bounds.il_max)
        core->fault.bits.il_ocp = 1;

    return (core->fault.all & core->fault_mask.all);
}

/**
 * @brief Background on-demand service querying running cascade cross status.
 */
GMP_STATIC_INLINE dcdc_mode_e ctl_get_dcdc_actual_mode(const ctl_dcdc_core_t* core)
{
    if (core->target_mode == DCDC_MODE_VOLTAGE_LOOP)
    {
        if ((core->v_loop_pi.out >= core->v_loop_pi.out_max) || (core->v_loop_pi.out <= core->v_loop_pi.out_min))
        {
            return DCDC_MODE_CURRENT_LOOP;
        }
    }
    return core->target_mode;
}

/*---------------------------------------------------------------------------*/
/* 5. Unidirectional Control Law Substations (Zero HCC Tracking Overhead)    */
/*---------------------------------------------------------------------------*/

GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_unidir_openloop_sub(ctl_dcdc_core_t* core)
{
    ctl_clear_pid(&core->v_loop_pi);
    ctl_clear_pid(&core->i_loop_pi);
    core->flag_direction = 1; // Default locked forward for unidirectional setups
    core->delta_v = core->v_out_ff + core->v_ff;
    return core->delta_v;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_unidir_current_sub(ctl_dcdc_core_t* core)
{
    ctl_clear_pid(&core->v_loop_pi);
    core->flag_direction = 1;
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, core->i_out_set_raw);
    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    core->delta_v = ctl_step_pid_ser(&core->i_loop_pi, i_err) + core->v_ff;
    return core->delta_v;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_unidir_voltage_sub(ctl_dcdc_core_t* core)
{
    core->flag_direction = 1;
    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_raw);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    ctrl_gt i_demand_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err) + core->i_ff;
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, i_demand_raw);

    ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->i_L_ref);

    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    core->delta_v = ctl_step_pid_ser(&core->i_loop_pi, i_err) + core->v_ff;
    return core->delta_v;
}

/*---------------------------------------------------------------------------*/
/* 6. Bidirectional Control Law Substations (Active Flow-Direction Tracks)  */
/*---------------------------------------------------------------------------*/

/**
 * @brief Updates LPF signal purifiers and runs the HCC tracker for bidirectional flow.
 */
GMP_STATIC_INLINE void ctl_dcdc_core_refresh_direction_sub(ctl_dcdc_core_t* core)
{
    ctl_step_lowpass_filter(&core->lpf_i_L, core->i_L_fdbk->value);
    ctl_set_hysteresis_target(&core->dir_detect, float2ctrl(0.0f));
    core->flag_direction =
        ctl_step_hysteresis_controller(&core->dir_detect, ctl_get_lowpass_filter_result(&core->lpf_i_L));
}

GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_bidir_openloop_sub(ctl_dcdc_core_t* core)
{
    ctl_clear_pid(&core->v_loop_pi);
    ctl_clear_pid(&core->i_loop_pi);
    ctl_dcdc_core_refresh_direction_sub(core); // Active flow direction update
    core->delta_v = core->v_out_ff + core->v_ff;
    return core->delta_v;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_bidir_current_sub(ctl_dcdc_core_t* core)
{
    ctl_clear_pid(&core->v_loop_pi);
    ctl_dcdc_core_refresh_direction_sub(core);
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, core->i_out_set_raw);
    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    core->delta_v = ctl_step_pid_ser(&core->i_loop_pi, i_err) + core->v_ff;
    return core->delta_v;
}

GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_bidir_voltage_sub(ctl_dcdc_core_t* core)
{
    ctl_dcdc_core_refresh_direction_sub(core);
    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_raw);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    ctrl_gt i_demand_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err) + core->i_ff;
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, i_demand_raw);

    ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->i_L_ref);

    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    core->delta_v = ctl_step_pid_ser(&core->i_loop_pi, i_err) + core->v_ff;
    return core->delta_v;
}

/**
 * @brief Atomic execution block for Parallel Competitive (Voltage/Current) regulation.
 * @details Both loops execute independently and compete using a MIN-selector. 
 * The losing loop is forced into back-calculation clamping to guarantee glitch-free handover.
 * Suitable for resonance power plants such as LLC converters.
 * * @param[in,out] core Pointer to the controller core instance.
 * @return Final won control voltage correction term delta_v (PU).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_parallel_compete_sub(ctl_dcdc_core_t* core)
{
    // 1. Run independent profile trajectory rate limiters
    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_raw);
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, core->i_out_set_raw);

    // 2. Compute independent regulatory tracking errors
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;

    // 3. Execute both compensators concurrently to obtain candidate demands
    // Note: In parallel competitive structure, both PIDs temporarily use their own internal state
    ctrl_gt v_demand = ctl_step_pid_ser(&core->v_loop_pi, v_err);
    ctrl_gt i_demand = ctl_step_pid_ser(&core->i_loop_pi, i_err);

    // 4. MIN-Selector Arbitrator (Competitive Handover)
    // The loop requesting a smaller control voltage (safer, less energetic) wins the execution port
    if (v_demand <= i_demand)
    {
        // Voltage loop wins: Active constant-voltage regulation
        core->delta_v = v_demand + core->v_out_ff + core->v_ff;

        // Clamp absolute hardware safety limits inside the PID bounds
        core->delta_v = ctl_sat(core->delta_v, core->i_loop_pi.out_max, core->i_loop_pi.out_min);

        // === PARALLEL ANTI-WINDUP ===
        // Current loop lost. Forcefully overwrite the current loop's integrator
        // using the winning voltage loop's current localized output.
        // This stops the losing current loop from drifting away.
        ctl_pid_clamping_correction_using_real_output(&core->i_loop_pi, core->delta_v - core->v_ff - core->v_out_ff);
    }
    else
    {
        // Current loop wins: Active constant-current limiting
        core->delta_v = i_demand + core->v_out_ff + core->v_ff;
        core->delta_v = ctl_sat(core->delta_v, core->i_loop_pi.out_max, core->i_loop_pi.out_min);

        // === PARALLEL ANTI-WINDUP ===
        // Voltage loop lost. Forcefully overwrite the voltage loop's integrator
        // based on the winning current loop's localized output.
        ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->delta_v - core->v_ff - core->v_out_ff);
    }

    return core->delta_v;
}

/*---------------------------------------------------------------------------*/
/* 7. Lab Evaluation & Parameters Debugging Unified Router Entries           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Debug router customized for single-direction DCDC plants.
 * @note Skips HCC execution entirely to optimize clock cycles.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_unidir_unified(ctl_dcdc_core_t* core)
{
    core->isr_tick++;
    if (!core->flag_enable)
        return float2ctrl(0.0f);

    switch (core->target_mode)
    {
    case DCDC_MODE_OPEN_LOOP:
        ctl_step_dcdc_unidir_openloop_sub(core);
        break;
    case DCDC_MODE_CURRENT_LOOP:
        ctl_step_dcdc_unidir_current_sub(core);
        break;
    case DCDC_MODE_VOLTAGE_LOOP:
        ctl_step_dcdc_unidir_voltage_sub(core);
        break;
    case DCDC_MODE_PARALLEL_COMPETE:
        ctl_step_dcdc_parallel_compete_sub(core);
        break;

    default:
        // wrong mode
        core->delta_v = 0;
        core->fault.bits.unknown_mode = 1;
        break;
    }
    ctl_step_dcdc_protect_sub(core);
    return core->delta_v;
}

/**
 * @brief Debug router customized for bidirectional DCDC energy flows.
 * @note Actively monitors LPF and HCC to yield pristine flow direction states.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_bidir_unified(ctl_dcdc_core_t* core)
{
    core->isr_tick++;
    if (!core->flag_enable)
        return float2ctrl(0.0f);

    switch (core->target_mode)
    {
    case DCDC_MODE_OPEN_LOOP:
        ctl_step_dcdc_bidir_openloop_sub(core);
        break;
    case DCDC_MODE_CURRENT_LOOP:
        ctl_step_dcdc_bidir_current_sub(core);
        break;
    case DCDC_MODE_VOLTAGE_LOOP:
        ctl_step_dcdc_bidir_voltage_sub(core);
        break;
    }
    ctl_step_dcdc_protect_sub(core);
    return core->delta_v;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_TXT_CTL_DCDC_CORE_H_

/**
 * @defgroup CTL_TOPOLOGY_DCDC_CORE_API Unified Dual-Loop DC-DC Core API
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines a unified dual-loop (Voltage + Current) controller for various DC-DC topologies.
 * @details This core outputs an "Equivalent Modulator Voltage" (v_pwm_req).
 * Features load feedforward, steady-state voltage feedforward, dual-slope limiters, 
 * rigorous cascaded anti-windup, and per-unit (PU) automated tuning.
 */

#ifndef _FILE_CTL_DCDC_CORE_H_
#define _FILE_CTL_DCDC_CORE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

/*---------------------------------------------------------------------------*/
/* Initialization & Auto-Tuning Structures                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Auto-tuning initialization structure for the DC-DC Core.
 * @details Users input physical SI units. The API calculates discrete Per-Unit (PU) gains.
 */
typedef struct _tag_dcdc_core_init_t
{
    // --- System Bases (Mandatory for PU Transformation) ---
    parameter_gt v_base; //!< Base voltage for PU system (V).
    parameter_gt i_base; //!< Base current for PU system (A).

    // --- Physical Parameters (Mandatory) ---
    parameter_gt fs;     //!< Controller execution frequency (Hz).
    parameter_gt L_main; //!< Main power inductor (H).
    parameter_gt C_out;  //!< Output filter capacitor (F).

    // --- Parasitic Parameters (Optional, set to 0.0f if unknown) ---
    parameter_gt r_L; //!< Inductor Equivalent Series Resistance (ESR, Ohms).
    parameter_gt r_C; //!< Capacitor Equivalent Series Resistance (ESR, Ohms).

    // --- Operating Points ---
    parameter_gt v_in_nom;   //!< Nominal input voltage (V).
    parameter_gt v_in_min;   //!< Minimum input voltage (V) - Critical for Boost/FSBB RHPZ.
    parameter_gt v_out_nom;  //!< Nominal output voltage (V).
    parameter_gt i_out_max;  //!< Maximum output load current (A).
    parameter_gt r_load_min; //!< Minimum expected load resistance (Ohms) -> v_out_nom / i_out_max.

    // --- Tuning Targets ---
    parameter_gt i_loop_bw; //!< Target current loop bandwidth (Hz).
    parameter_gt v_loop_bw; //!< Target voltage loop bandwidth (Hz).

    // --- Physical Limits ---
    parameter_gt i_L_max;   //!< Inductor current maximum limit (A).
    parameter_gt i_L_min;   //!< Inductor current minimum limit (A). (e.g., 0 for strictly Unidirectional)
    parameter_gt v_req_max; //!< Maximum allowed driving voltage (V).
    parameter_gt v_req_min; //!< Minimum allowed driving voltage (V). (Typically 0 for DC)

    // --- Auto-Tuned PU Controller Parameters (Calculated by API) ---
    parameter_gt kp_i_pu, ki_i_pu; //!< PU Current Loop PI gains.
    parameter_gt kp_v_pu, ki_v_pu; //!< PU Voltage Loop PI gains.

} ctl_dcdc_core_init_t;

/*---------------------------------------------------------------------------*/
/* Main Control Core Structure                                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Unified Data Structure for Dual-Loop DC-DC Converters.
 */
typedef struct _tag_dcdc_core_t
{
    uint32_t isr_tick;

    // --- Hardware Interface Bindings (Zero-copy Pointers) ---
    adc_ift* v_in_fdbk;   //!< Input voltage feedback (PU).
    adc_ift* v_out_fdbk;  //!< Output voltage feedback (PU).
    adc_ift* i_L_fdbk;    //!< Inductor current feedback (PU).
    adc_ift* i_load_fdbk; //!< Output load current feedback (PU). Optional.

    // --- User Setpoints & Limiters ---
    ctrl_gt v_out_set_raw; //!< User-commanded target voltage (PU).
    ctrl_gt i_out_set_raw; //!< Current command target current (PU).

    ctrl_gt v_out_ff; //!< User Voltage feed-foward

    ctl_slope_limiter_t v_ramp; //!< Voltage reference slope limiter.
    ctl_slope_limiter_t i_ramp; //!< Current reference slope limiter.

    // --- Dual-Loop Controllers ---
    // Limits are strictly enforced INSIDE these PID objects to prevent duplication.
    ctl_pid_t v_loop_pi; //!< Voltage outer loop PI controller. Output is target I_L (PU).
    ctl_pid_t i_loop_pi; //!< Current inner loop PI controller. Output is delta Voltage (PU).

    ctrl_gt load_ff_gain; //!< Load current feedforward gain (0.0 to disable).

    // --- Outputs & Intermediate Variables ---
    ctrl_gt v_out_ref; //!< Real-time ramped voltage reference (PU).
    ctrl_gt i_L_ref;   //!< Real-time inner loop current reference (PU).
    ctrl_gt v_pwm_req; //!< Final Equivalent Modulator Target Voltage (PU). Range: [0, V_max].

    // --- State Flags ---
    fast_gt flag_enable;              //!< Master enable flag.
    fast_gt flag_enable_load_ff;      //!< Enable load current feedforward.
    fast_gt flag_enable_current_loop; //!< Enable Current control
    fast_gt flag_enable_voltage_loop; //!< Enable votlage control

} ctl_dcdc_core_t;

/*---------------------------------------------------------------------------*/
/* API Prototypes & Auto-Tuning Implementations                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Auto-tunes the DC-DC Core parameters specifically for BUCK topology (with PU mapping).
 */
GMP_STATIC_INLINE void ctl_auto_tuning_dcdc_buck(ctl_dcdc_core_init_t* init)
{
    parameter_gt z_base = init->v_base / init->i_base;

    // Default Bandwidths
    if (init->i_loop_bw <= 0.001f)
        init->i_loop_bw = init->fs / 10.0f;
    if (init->v_loop_bw <= 0.001f)
        init->v_loop_bw = init->i_loop_bw / 5.0f;

    parameter_gt wc_i = 2.0f * 3.14159265f * init->i_loop_bw;
    parameter_gt wc_v = 2.0f * 3.14159265f * init->v_loop_bw;

    // --- Current Loop (SI -> PU) ---
    // Kp = L * wc
    parameter_gt kp_i_si = init->L_main * wc_i;
    // Ki logic: Pole-zero cancellation if r_L is provided, else standard rule of thumb
    parameter_gt ki_i_si = (init->r_L > 0.0001f) ? (kp_i_si * (init->r_L / init->L_main)) : (kp_i_si * (wc_i / 10.0f));

    init->kp_i_pu = kp_i_si / z_base;
    init->ki_i_pu = ki_i_si / z_base;

    // --- Voltage Loop (SI -> PU) ---
    // Kp = C * wc
    parameter_gt kp_v_si = init->C_out * wc_v;
    parameter_gt r_load = (init->r_load_min > 0.01f) ? init->r_load_min : (init->v_out_nom / init->i_out_max);

    // Ki logic: Cancel low frequency pole formed by load and ESR if possible
    parameter_gt ki_v_si = kp_v_si * (1.0f / (init->C_out * (r_load + init->r_C)));
    if (init->r_C <= 0.0001f)
        ki_v_si = kp_v_si * (wc_v / 10.0f); // Fallback

    init->kp_v_pu = kp_v_si * z_base;
    init->ki_v_pu = ki_v_si * z_base;
}

/**
 * @brief Auto-tunes the DC-DC Core parameters specifically for BOOST topology (with PU & RHPZ).
 */
GMP_STATIC_INLINE void ctl_auto_tuning_dcdc_boost(ctl_dcdc_core_init_t* init)
{
    parameter_gt z_base = init->v_base / init->i_base;

    // 1. Current Loop Tuning (Same physical plant as Buck: V_L = sL + R_L)
    if (init->i_loop_bw <= 0.001f)
        init->i_loop_bw = init->fs / 10.0f;
    parameter_gt wc_i = 2.0f * 3.14159265f * init->i_loop_bw;
    parameter_gt kp_i_si = init->L_main * wc_i;
    parameter_gt ki_i_si = (init->r_L > 0.0001f) ? (kp_i_si * (init->r_L / init->L_main)) : (kp_i_si * (wc_i / 10.0f));

    init->kp_i_pu = kp_i_si / z_base;
    init->ki_i_pu = ki_i_si / z_base;

    // 2. RHPZ Calculation (Worst case: Min Vin, Max Load)
    parameter_gt d_max = 1.0f - (init->v_in_min / init->v_out_nom);
    if (d_max < 0.1f)
        d_max = 0.5f;

    parameter_gt r_load = (init->r_load_min > 0.01f) ? init->r_load_min : (init->v_out_nom / init->i_out_max);
    parameter_gt w_rhpz = (r_load * (1.0f - d_max) * (1.0f - d_max)) / init->L_main;

    // 3. Voltage Loop Bandwidth Suppression (Force BW < 1/4 RHPZ)
    parameter_gt safe_v_bw = (w_rhpz / (2.0f * 3.14159265f)) / 4.0f;
    if (init->v_loop_bw <= 0.001f || init->v_loop_bw > safe_v_bw)
    {
        init->v_loop_bw = safe_v_bw;
    }

    // 4. Voltage Loop Tuning
    parameter_gt wc_v = 2.0f * 3.14159265f * init->v_loop_bw;
    parameter_gt kp_v_si = (init->C_out * wc_v) / (1.0f - d_max);
    parameter_gt ki_v_si = kp_v_si * (1.0f / (init->C_out * (r_load + init->r_C)));

    init->kp_v_pu = kp_v_si * z_base;
    init->ki_v_pu = ki_v_si * z_base;
}

/**
 * @brief Auto-tunes the DC-DC Core parameters specifically for 4-Switch Buck-Boost (FSBB).
 * @details Inherits RHPZ constraints from the worst-case Boost operating mode.
 */
GMP_STATIC_INLINE void ctl_auto_tuning_dcdc_fsbb(ctl_dcdc_core_init_t* init)
{
    // FSBB tuning operates similarly to Boost because the system must be stable
    // when it transitions into the Boost region (where the RHPZ appears).
    // The same RHPZ bandwidth suppression logic from Boost is applied here.
    ctl_auto_tuning_dcdc_boost(init);
}

/**
 * @brief Initializes the unified DC-DC core.
 */
void ctl_init_dcdc_core(ctl_dcdc_core_t* core, const ctl_dcdc_core_init_t* init, parameter_gt v_slope_pu,
                        parameter_gt i_slope_pu);

/**
 * @brief Binds ADC interfaces to the Core.
 */
GMP_STATIC_INLINE void ctl_attach_dcdc_core(ctl_dcdc_core_t* core, adc_ift* _v_in, adc_ift* _v_out, adc_ift* _i_L,
                                            adc_ift* _i_load)
{
    core->v_in_fdbk = _v_in;
    core->v_out_fdbk = _v_out;
    core->i_L_fdbk = _i_L;
    core->i_load_fdbk = _i_load;
}

/*---------------------------------------------------------------------------*/
/* Core Execution Step Functions (Zero-Branching)                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Executes one step for a BUCK converter.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_buck(ctl_dcdc_core_t* core)
{
    if (!core->flag_enable)
        return float2ctrl(0.0f);

    // 1. Voltage Loop & Ramp
    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_raw);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    core->i_out_set_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err);

    // 2. Load Feedforward
    if (core->flag_enable_load_ff && core->i_load_fdbk)
    {
        core->v_out_set_raw += ctl_mul(core->i_load_fdbk->value, core->load_ff_gain);
    }

    // 3. Current Slew Rate Limiter (Limiting relies on the slope limiter and the PID's internal out_max)
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, core->v_out_set_raw);

    // === CASCADED ANTI-WINDUP 1 ===
    // Inform the outer loop if its output was clamped by slope or absolute limits
    ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->i_L_ref);

    // 4. Current Loop
    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    ctrl_gt v_adj = ctl_step_pid_ser(&core->i_loop_pi, i_err);

    // 5. Topology Steady-State Feedforward & Clamping
    // For Buck, V_switch = V_out + V_L_adj
    ctrl_gt v_ff = core->v_out_fdbk->value;
    ctrl_gt v_req_raw = v_ff + v_adj;

    // Apply the absolute limits strictly defined inside the PID object (e.g., [0, V_max])
    core->v_pwm_req = ctl_sat(v_req_raw, core->i_loop_pi.out_max, core->i_loop_pi.out_min);

    // === CASCADED ANTI-WINDUP 2 ===
    // If the required voltage exceeds hardware bounds (like > Vin or < 0), back-calculate the inner loop!
    ctl_pid_clamping_correction_using_real_output(&core->i_loop_pi, core->v_pwm_req - v_ff);

    return core->v_pwm_req;
}

/**
 * @brief Executes one step for a BOOST converter.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_boost(ctl_dcdc_core_t* core)
{
    if (!core->flag_enable)
        return float2ctrl(0.0f);

    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_raw);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    core->v_out_set_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err);

    if (core->flag_enable_load_ff && core->i_load_fdbk)
    {
        ctrl_gt v_in_safe = (core->v_in_fdbk->value > float2ctrl(0.1f)) ? core->v_in_fdbk->value : float2ctrl(0.1f);
        ctrl_gt duty_eff = float2ctrl(1.0f) - ctl_div(v_in_safe, core->v_out_fdbk->value);
        ctrl_gt boost_ff_gain = ctl_div(core->load_ff_gain, float2ctrl(1.0f) - duty_eff);
        core->v_out_set_raw += ctl_mul(core->i_load_fdbk->value, boost_ff_gain);
    }

    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, core->v_out_set_raw);
    ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->i_L_ref);

    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    ctrl_gt v_adj = ctl_step_pid_ser(&core->i_loop_pi, i_err);

    // For Boost, V_L_adj = V_in - V_switch  =>  V_switch = V_in - V_L_adj
    ctrl_gt v_ff = core->v_in_fdbk->value;
    ctrl_gt v_req_raw = v_ff - v_adj;

    // Use PID's built-in [0, V_max] constraints
    core->v_pwm_req = ctl_sat(v_req_raw, core->i_loop_pi.out_max, core->i_loop_pi.out_min);

    // Back-calculate inner loop
    ctl_pid_clamping_correction_using_real_output(&core->i_loop_pi, v_ff - core->v_pwm_req);

    return core->v_pwm_req;
}

/**
 * @brief Executes one step for a 4-Switch Buck-Boost (FSBB) converter.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dcdc_fsbb(ctl_dcdc_core_t* core)
{
    if (!core->flag_enable)
        return float2ctrl(0.0f);

    if (core->flag_enable_voltage_loop)
    {
        core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_raw);
        ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
        core->v_out_set_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err);

        if (core->flag_enable_load_ff && core->i_load_fdbk)
        {
            ctrl_gt ratio = (core->v_out_fdbk->value > core->v_in_fdbk->value)
                                ? ctl_div(core->v_out_fdbk->value, core->v_in_fdbk->value)
                                : float2ctrl(1.0f);
            core->v_out_set_raw += ctl_mul(core->i_load_fdbk->value, ratio);
        }
    }
    else
    {
        // keep core->v_out_set_raw as user input
        ctl_clear_pid(&core->v_loop_pi);
    }

    ctrl_gt v_adj;

    if (core->flag_enable_current_loop)
    {
        core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, core->v_out_set_raw);
        ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->i_L_ref);

        ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
        v_adj = ctl_step_pid_ser(&core->i_loop_pi, i_err);

        // For FSBB, the H-bridge output voltage needs to match V_out
        ctrl_gt v_ff = core->v_out_fdbk->value;
        ctrl_gt v_req_raw = v_ff + v_adj + core->v_out_ff;

        core->v_pwm_req = ctl_sat(v_req_raw, core->i_loop_pi.out_max, core->i_loop_pi.out_min);
        ctl_pid_clamping_correction_using_real_output(&core->i_loop_pi, core->v_pwm_req - v_ff);
    }
    else
    {
        ctl_clear_pid(&core->i_loop_pi);

        ctrl_gt v_req_raw = core->v_out_ff;
        core->v_pwm_req = ctl_sat(v_req_raw, core->i_loop_pi.out_max, core->i_loop_pi.out_min);
    }

    return core->v_pwm_req;
}

GMP_STATIC_INLINE void ctl_clear_dcdc_core(ctl_dcdc_core_t* core)
{
    ctl_clear_pid(&core->i_loop_pi);
    ctl_clear_pid(&core->v_loop_pi);

    ctl_clear_slope_limiter(&core->v_ramp);
    ctl_clear_slope_limiter(&core->i_ramp);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_DCDC_CORE_H_
