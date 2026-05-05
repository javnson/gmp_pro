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
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/slope_f_pu.h>

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
    ctrl_gt v_out_set_user; //!< User-commanded target voltage (PU).

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
    fast_gt flag_enable;         //!< Master enable flag.
    fast_gt flag_enable_load_ff; //!< Enable load current feedforward.

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
                        parameter_gt i_slope_pu)
{
    // Init PI Controllers
    ctl_init_pid_Tmode(&core->v_loop_pi, float2ctrl(init->kp_v_pu), float2ctrl(init->kp_v_pu / init->ki_v_pu),
                       float2ctrl(0), float2ctrl(init->fs));
    ctl_init_pid_Tmode(&core->i_loop_pi, float2ctrl(init->kp_i_pu), float2ctrl(init->kp_i_pu / init->ki_i_pu),
                       float2ctrl(0), float2ctrl(init->fs));

    // --- Configure Strict PID Limits (PU) ---
    // 1. Voltage loop outputs Current. Limit it via i_L_max / i_L_min.
    ctrl_gt limit_i_max = float2ctrl(init->i_L_max / init->i_base);
    ctrl_gt limit_i_min = float2ctrl(init->i_L_min / init->i_base);
    ctl_set_pid_limit(&core->v_loop_pi, limit_i_max, limit_i_min);
    ctl_set_pid_int_limit(&core->v_loop_pi, limit_i_max, limit_i_min);

    // 2. Current loop handles Voltage. Limit it via v_req_max / v_req_min.
    // NOTE: Minimum voltage is strictly limited here (e.g., to 0 for unidirectional DC output).
    ctrl_gt limit_v_max = float2ctrl(init->v_req_max / init->v_base);
    ctrl_gt limit_v_min = float2ctrl(init->v_req_min / init->v_base);
    ctl_set_pid_limit(&core->i_loop_pi, limit_v_max, limit_v_min);
    ctl_set_pid_int_limit(&core->i_loop_pi, limit_v_max, limit_v_min);

    // Init Slope Limiters (Units/sec)
    ctl_init_slope_limiter(&core->v_ramp, float2ctrl(v_slope_pu), float2ctrl(-v_slope_pu), float2ctrl(init->fs));
    ctl_init_slope_limiter(&core->i_ramp, float2ctrl(i_slope_pu), float2ctrl(-i_slope_pu), float2ctrl(init->fs));

    core->flag_enable = 0;
    core->flag_enable_load_ff = 0;
    core->load_ff_gain = float2ctrl(1.0f);

    // Null pointers
    core->v_in_fdbk = NULL;
    core->v_out_fdbk = NULL;
    core->i_L_fdbk = NULL;
    core->i_load_fdbk = NULL;
}

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
    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_user);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    ctrl_gt i_ref_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err);

    // 2. Load Feedforward
    if (core->flag_enable_load_ff && core->i_load_fdbk)
    {
        i_ref_raw += ctl_mul(core->i_load_fdbk->value, core->load_ff_gain);
    }

    // 3. Current Slew Rate Limiter (Limiting relies on the slope limiter and the PID's internal out_max)
    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, i_ref_raw);

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

    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_user);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    ctrl_gt i_ref_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err);

    if (core->flag_enable_load_ff && core->i_load_fdbk)
    {
        ctrl_gt v_in_safe = (core->v_in_fdbk->value > float2ctrl(0.1f)) ? core->v_in_fdbk->value : float2ctrl(0.1f);
        ctrl_gt duty_eff = float2ctrl(1.0f) - ctl_div(v_in_safe, core->v_out_fdbk->value);
        ctrl_gt boost_ff_gain = ctl_div(core->load_ff_gain, float2ctrl(1.0f) - duty_eff);
        i_ref_raw += ctl_mul(core->i_load_fdbk->value, boost_ff_gain);
    }

    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, i_ref_raw);
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

    core->v_out_ref = ctl_step_slope_limiter(&core->v_ramp, core->v_out_set_user);
    ctrl_gt v_err = core->v_out_ref - core->v_out_fdbk->value;
    ctrl_gt i_ref_raw = ctl_step_pid_ser(&core->v_loop_pi, v_err);

    if (core->flag_enable_load_ff && core->i_load_fdbk)
    {
        ctrl_gt ratio = (core->v_out_fdbk->value > core->v_in_fdbk->value)
                            ? ctl_div(core->v_out_fdbk->value, core->v_in_fdbk->value)
                            : float2ctrl(1.0f);
        i_ref_raw += ctl_mul(core->i_load_fdbk->value, ratio);
    }

    core->i_L_ref = ctl_step_slope_limiter(&core->i_ramp, i_ref_raw);
    ctl_pid_clamping_correction_using_real_output(&core->v_loop_pi, core->i_L_ref);

    ctrl_gt i_err = core->i_L_ref - core->i_L_fdbk->value;
    ctrl_gt v_adj = ctl_step_pid_ser(&core->i_loop_pi, i_err);

    // For FSBB, the H-bridge output voltage needs to match V_out
    ctrl_gt v_ff = core->v_out_fdbk->value;
    ctrl_gt v_req_raw = v_ff + v_adj;

    core->v_pwm_req = ctl_sat(v_req_raw, core->i_loop_pi.out_max, core->i_loop_pi.out_min);
    ctl_pid_clamping_correction_using_real_output(&core->i_loop_pi, core->v_pwm_req - v_ff);

    return core->v_pwm_req;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_DCDC_CORE_H_
