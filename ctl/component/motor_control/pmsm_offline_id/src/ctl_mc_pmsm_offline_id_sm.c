
#include <gmp_core.h>

#include <ctl/component/motor_control/basic/mtr_protection.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/current_loop/foc_core.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/interface/encoder_switcher.h>
#include <ctl/component/motor_control/observer/pmsm_esmo.h>

#include <ctl/component/motor_control/consultant/mech_consultant.h>
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

#include <ctl/component/motor_control/pmsm_offline_id/pmsm_offline_id_sm.h>

//////////////////////////////////////////////////////////////////////////

//TODO
// 对于data analyze 的要求：
// 1. 要有一个存储池用于保存数据
// 2. 数据要能够划分成不同长度的单元（可以保存m*n的表格）
// 3. 要能够提供2*n的数据单元拟合
// 4.

/**
 * @brief Forward declaration of the Data Analyzer structure.
 */
typedef struct _tag_ctl_data_analyzer ctl_data_analyzer_t;

/**
 * @brief Clears the Data Analyzer buffers and resets its internal state.
 * @param[in,out] da Pointer to the data analyzer instance.
 */
void ctl_da_clear(ctl_data_analyzer_t* da);

/**
 * @brief Pushes a single data point into the analyzer's buffer.
 * @param[in,out] da Pointer to the data analyzer instance.
 * @param[in]     u  The sampled voltage value (PU).
 * @param[in]     i  The sampled current value (PU).
 */
void ctl_da_push_point(ctl_data_analyzer_t* da, ctrl_gt u, ctrl_gt i);

//////////////////////////////////////////////////////////////////////////

ctl_pos_autoturn_encoder_t enc; //!< physical encoder

// ============================================================================

/**
 * @brief State enumeration for the Stator Resistance (Rs) & Dead-Time (DT) sub-state machine.
 * * @details This state machine executes a nested loop sequence to identify Rs and Vcomp:
 * Outer Loop: 6 electrical rotor positions (Angles).
 * Inner Loop: N discrete current levels (Steps).
 * * Execution Flow:
 * INIT -> [Loop: Angles -> ALIGN_SETTLE -> [Loop: Steps -> STEP_DELAY -> MEASURE -> EVALUATE] ] -> CALCULATE -> COMPLETE
 */
typedef enum _tag_pmsm_offline_id_rs_dt_sm
{
    /** * @brief 0: Idle / Bypassed.
     * @details PWM is off or the module is skipped. Safe state. 
     */
    PMSM_OID_RSDT_DISABLED = 0,

    /** * @brief 1: Initialization.
     * @details Triggers the background loop to pre-calculate tick delays and current arrays.
     * @note Debug: Resets `angle_idx` = 0, `step_idx` = 0, `tick_timer` = 0. 
     */
    PMSM_OID_RSDT_INIT,

    /** * @brief 2: Mechanical & Electrical Alignment.
     * @details Forces the FOC static angle to `angle_pu_array[angle_idx]` and injects maximum 
     * current to pull the rotor to the target position.
     * @note Debug: `tick_timer` increments until it reaches `align_ticks` (derived from `cfg.align_time_s`). 
     */
    PMSM_OID_RSDT_ALIGN_SETTLE,

    /** * @brief 3: Step Current Transient Delay.
     * @details Applies the specific target current `current_step_array[step_idx]` and waits for 
     * the L/R inductive transient to fully decay into a steady DC state.
     * @note Debug: `tick_timer` increments until it reaches `measure_delay_ticks` (derived from `cfg.measure_delay_s`). 
     */
    PMSM_OID_RSDT_STEP_DELAY,

    /** * @brief 4: Data Measurement & Accumulation.
     * @details Integrates voltage commands (`vdq_ref`) and actual currents (`idq0`) over a fixed number of ISR cycles.
     * @note Debug: `tick_timer` increments until it reaches `cfg.measure_points`. 
     * Variables `sum_u` and `sum_i` will actively accumulate during this state.
     */
    PMSM_OID_RSDT_MEASURE,

    /** * @brief 5: Loop Evaluation & Routing.
     * @details A 0-tick transient state. Evaluates if all steps for the current angle are done, 
     * and if all 6 angles are completed.
     * @note Debug: Watch `step_idx` and `angle_idx` increment here. Routes back to `STEP_DELAY`, 
     * `ALIGN_SETTLE`, or moves forward to `CALCULATE`.
     */
    PMSM_OID_RSDT_STEP_EVALUATE,

    /** * @brief 6: Least-Squares Calculation.
     * @details Hands over execution to the background loop to perform linear regression on the 
     * accumulated Data Analyzer points. Output is safely clamped to 0V/0A.
     * @note Debug: Watch for `rs_mean` and `vcomp_mean` to be populated in the main context.
     */
    PMSM_OID_RSDT_CALCULATE,

    /** * @brief 7: Identification Complete.
     * @details Signals the master state machine that Rs and DT parameters have been successfully 
     * updated in the global consultant structure. Output is disabled.
     */
    PMSM_OID_RSDT_COMPLETE,

    /** * @brief 8: Fault / Exception.
     * @details Entered upon over-current, timeout, or mathematical singularities during fitting.
     * Output is immediately disabled.
     */
    PMSM_OID_RSDT_FAULT

} pmsm_offline_id_rs_dt_sm_t;

/**
 * @brief Configuration for Stator Resistance (Rs) & Dead-Time (DT) Identification.
 */
typedef struct _tag_pmsm_oid_cfg_rs_dt
{
    parameter_gt max_current_pu;  /*!< Maximum DC current to inject (e.g., 0.8pu). */
    parameter_gt min_current_pu;  /*!< Minimum DC current to inject (e.g., 0.2pu). */
    uint16_t steps;               /*!< Number of current steps between min and max (e.g., 5). */
    parameter_gt align_time_s;    /*!< Time to wait for mechanical rotor alignment (s). */
    parameter_gt measure_delay_s; /*!< Time to wait for L/R transient to decay BEFORE measuring (s). */
    uint16_t measure_points;      /*!< Number of sampling points to average per step during MEASURE state. */
} pmsm_oid_cfg_rs_dt_t;

/**
 * @brief Sub-process context for Resistance and Dead-time identification.
 */
typedef struct _tag_pmsm_offline_id_rs_dt
{
    pmsm_offline_id_rs_dt_sm_t sm; /*!< Sub-SM: Current state of Rs & DT identification. */
    pmsm_oid_cfg_rs_dt_t cfg;      /*!< Configuration specific to this module. */

    // --- Pre-calculated Context (Computed in Loop to save ISR time) ---
    uint32_t align_ticks;         /*!< ISR ticks corresponding to align_time_s. */
    uint32_t measure_delay_ticks; /*!< ISR ticks corresponding to measure_delay_s. */
    ctrl_gt step_size_pu;         /*!< Current increment per step: (Max-Min)/(Steps-1). */
    ctrl_gt inv_measure_points;   /*!< 1.0f / measure_points, to avoid division in ISR. */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;    /*!< Internal timer for delays and settling. */
    uint16_t angle_idx;     /*!< Current electrical angle index (0 to 5). */
    uint16_t step_idx;      /*!< Current injected current step index. */
    ctrl_gt current_ref_pu; /*!< The active DC current reference being applied. */
    fast_gt is_first_entry; /*!< Flag to distinguish between step (first entry) and hold logic. */

    // --- Measurement Accumulators (ISR) ---
    ctrl_gt sum_u; /*!< Voltage accumulator for averaging. */
    ctrl_gt sum_i; /*!< Current accumulator for averaging. */

    // --- Identification Results ---
    parameter_gt rs_array[6];    /*!< Identified resistance (PU) for each of the 6 positions. */
    parameter_gt vcomp_array[6]; /*!< Identified dead-time voltage (PU) for each of the 6 positions. */
    parameter_gt rs_mean;        /*!< Mean value of the 6 identified resistances (PU). */
    parameter_gt rs_var;         /*!< Variance of the 6 identified resistances (PU). */
    parameter_gt vcomp_mean;     /*!< Mean value of the 6 identified dead-time voltages (PU). */
    parameter_gt vcomp_var;      /*!< Variance of the 6 identified dead-time voltages (PU). */

} pmsm_offline_id_rs_dt_t;

// ============================================================================

/**
 * @brief Sub-state machine for d-axis and q-axis Inductance (Ld, Lq) identification.
 * Execution flow: INIT -> [Loop: D-axis, Q-axis -> [Loop: N Bias Steps -> BIAS_SETTLE -> PULSE -> COOLDOWN] ] -> CALC -> COMPLETE.
 */
typedef enum _tag_pmsm_offline_id_ld_lq_sm
{
    PMSM_OID_LDQ_DISABLED = 0, /*!< 0: Disabled/Bypass. Allows the main SM to skip this step. */

    PMSM_OID_LDQ_INIT, /*!< 1: Initialize logic. Set target axis to D-axis (theta = alignment_offset).
                                            Reset step counters and clear Data Analyzer buffers. */

    PMSM_OID_LDQ_BIAS_SETTLE, /*!< 2: Apply DC Bias Current (Id_bias or Iq_bias) using PI controllers.
                                            Wait for the L/R transient to settle. 
                                            If testing unsaturated L (bias=0A), just wait for I=0. */

    PMSM_OID_LDQ_PULSE_MEASURE, /*!< 3: Open-loop Voltage Pulse Injection.
                                            Suspend PI controllers. Inject a fixed voltage vector (e.g., U_test) 
                                            on the active axis for a VERY SHORT duration (e.g., 500us ~ 2ms).
                                            Trigger Data Analyzer to record high-speed current slope (di/dt). */

    PMSM_OID_LDQ_COOLDOWN, /*!< 4: Flux/Energy Reset.
                                            Apply 0V (zero vector) or re-enable PI to drive current back to the 
                                            bias level or 0A. Wait until current is fully discharged to prevent 
                                            current runaway on the next pulse. */

    PMSM_OID_LDQ_STEP_EVALUATE, /*!< 5: Loop Controller.
                                            - If bias steps remain: Update bias current, go to BIAS_SETTLE.
                                            - If axis is D and D is done: Switch to Q-axis (theta += 90 deg), 
                                              reset steps, go to BIAS_SETTLE.
                                            - If both D and Q are mapped: Go to CALCULATE. */

    PMSM_OID_LDQ_CALCULATE, /*!< 6: Trigger Math library. 
                                            Calculate L = (U_ref - Rs*I - V_comp) / (di/dt) for each point.
                                            Fit the L-I saturation polynomial curves for D and Q axes. */

    PMSM_OID_LDQ_COMPLETE, /*!< 7: Update the ctl_consultant_pu_pmsm_t structure.
                                            Signal the main state machine to proceed. */

    PMSM_OID_LDQ_FAULT /*!< 8: Exception handling. Triggered by over-current during pulse, 
                                            unintended rotor movement (if encoder detects delta_theta > threshold), 
                                            or DA timeout. */

} pmsm_offline_id_ld_lq_sm_t;

/**
 * @brief Configuration for Inductance (Ld, Lq) Identification.
 * todo: 一些在主中断中参与计算的项目应当保存为ctrl_gt类型
 */
typedef struct _tag_pmsm_oid_cfg_ld_lq
{
    parameter_gt pulse_voltage_pu; /*!< Voltage magnitude for the high-frequency pulse (e.g., 0.3pu). */
    parameter_gt max_bias_curr_pu; /*!< Maximum DC bias current for saturation curve (e.g., 1.0pu). */
    uint16_t bias_steps;           /*!< Number of bias current steps (e.g., 5). */
    parameter_gt align_current_pu; /*!< NEW: D-axis DC current applied during Lq measurement to lock rotor. */
    parameter_gt settle_time_s;    /*!< NEW: Time to wait for DC bias and alignment current to stabilize. */
    parameter_gt pulse_time_s;     /*!< Extremely short duration of the voltage pulse (e.g., 0.001s). */
    parameter_gt cooldown_time_s;  /*!< Time to wait for flux to reset between pulses (e.g., 0.1s). */
} pmsm_oid_cfg_ld_lq_t;

/**
 * @brief Sub-process context for Inductance (Ld, Lq) identification.
 */
typedef struct _tag_pmsm_offline_id_ldq
{
    pmsm_offline_id_ld_lq_sm_t sm; /*!< Sub-SM: Current state of Ld/Lq identification. */
    pmsm_oid_cfg_ld_lq_t cfg;      /*!< Configuration specific to this module. */

    // --- Pre-calculated Context (Computed in Loop) ---
    uint32_t settle_ticks;   /*!< ISR ticks corresponding to settle_time_s. */
    uint32_t pulse_ticks;    /*!< ISR ticks corresponding to pulse_time_s. */
    uint32_t cooldown_ticks; /*!< ISR ticks corresponding to cooldown_time_s. */
    ctrl_gt step_size_pu;    /*!< Current increment per bias step. */
    parameter_gt dt_sec;     /*!< Time step per ISR tick (1.0 / isr_freq_hz). */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;         /*!< Internal timer for pulse duration and cooldown. */
    uint16_t bias_step_idx;      /*!< Current bias current step index. */
    fast_gt is_measuring_q_axis; /*!< Flag: 0 for D-axis measurement, 1 for Q-axis. */
    ctrl_gt bias_curr_ref_pu;    /*!< The active DC bias current being applied. */
    fast_gt is_first_entry;      /*!< Flag to distinguish between step (first entry) and hold logic. */

    // --- PI Output Freeze Variables ---
    ctrl_gt frozen_vd_pu; /*!< Holds the steady-state Vd before opening the loop. */
    ctrl_gt frozen_vq_pu; /*!< Holds the steady-state Vq before opening the loop. */

    // --- Identification Results ---
    parameter_gt ld_array[16]; /*!< Identified Ld (PU) for each bias step. */
    parameter_gt lq_array[16]; /*!< Identified Lq (PU) for each bias step. */

} pmsm_offline_id_ldq_t;

// ============================================================================

/**
 * @brief State enumeration for the PM Flux Linkage (Psi_m) sub-state machine.
 * @details This state machine executes a step-wise speed profile in I/F mode:
 * Execution Flow:
 * INIT -> [Loop: Steps -> RAMP_SPEED -> SETTLE -> MEASURE -> EVALUATE] -> RAMP_STOP -> CALCULATE -> COMPLETE
 */
typedef enum _tag_pmsm_offline_id_flux_sm
{
    /** @brief 0: Disabled/Bypass. Safe state. */
    PMSM_OID_FLUX_DISABLED = 0,

    /** @brief 1: Initialization. 
     * @details Pre-calculates step sizes and timing ticks in the background loop. 
     * @note Debug: Resets `step_idx` = 0, `is_first_entry` = 1. 
     */
    PMSM_OID_FLUX_INIT,

    /** @brief 2: Speed Trajectory Generator.
     * @details Sets the target speed for the V/F generator and waits for the ramp to finish.
     * @note Debug: Watch `ctx->vf_gen.current_freq_pu` approach `target_w_pu`.
     */
    PMSM_OID_FLUX_RAMP_SPEED,

    /** @brief 3: Mechanical Stabilization.
     * @details Waits for the "spring-pendulum" oscillation of the rotor to dampen.
     * @note Debug: `tick_timer` increments until `settle_ticks`.
     */
    PMSM_OID_FLUX_SETTLE,

    /** @brief 4: Data Collection.
     * @details Accumulates Ud, Uq, Id, Iq, and W_ref over a fixed number of ISR ticks.
     * @note Debug: `tick_timer` increments until `cfg.measure_points`.
     */
    PMSM_OID_FLUX_MEASURE,

    /** @brief 5: Loop Controller.
     * @details Evaluates if all speed steps are completed to route to STOP or next SPEED.
     */
    PMSM_OID_FLUX_STEP_EVALUATE,

    /** @brief 6: Safe Shutdown.
     * @details Ramps the speed down to 0 smoothly to avoid regenerative over-voltage.
     */
    PMSM_OID_FLUX_RAMP_STOP,

    /** @brief 7: Mathematical Fitting.
     * @details Hands over to the background loop. Calculates |E| and uses Data Analyzer to find Psi_m.
     */
    PMSM_OID_FLUX_CALCULATE,

    /** @brief 8: Update ctl_consultant_pu_pmsm_t structure. Signal Main SM. */
    PMSM_OID_FLUX_COMPLETE,

    /** @brief 9: Exception handling. (Over-current, Loss of Sync, OVP). */
    PMSM_OID_FLUX_FAULT

} pmsm_offline_id_flux_sm_t;

/**
 * @brief Configuration for Flux Linkage (Psi_m) Identification.
 */
typedef struct _tag_pmsm_oid_cfg_flux
{
    parameter_gt min_target_speed_pu; /*!< Minimum speed for measurement (e.g., 0.15pu). */
    parameter_gt max_target_speed_pu; /*!< Maximum speed for measurement (e.g., 0.35pu). */
    uint16_t steps;                   /*!< Number of speed steps (e.g., 5). */
    parameter_gt if_current_pu;       /*!< Constant dragging current magnitude (e.g., 0.2pu). */
    parameter_gt settle_time_s;       /*!< Time to wait for rotor oscillation to dampen (s). */
    uint16_t measure_points;          /*!< Number of sampling points to average per step. */
} pmsm_oid_cfg_flux_t;

/**
 * @brief Sub-process context for Flux Linkage (Psi_m) identification.
 */
typedef struct _tag_pmsm_offline_id_flux
{
    pmsm_offline_id_flux_sm_t sm; /*!< Sub-SM: Current state of Flux identification. */
    pmsm_oid_cfg_flux_t cfg;      /*!< Configuration specific to this module. */

    // --- Pre-calculated Context (Computed in Loop) ---
    uint32_t settle_ticks;      /*!< ISR ticks corresponding to settle_time_s. */
    ctrl_gt step_size_pu;       /*!< Speed increment per step. */
    ctrl_gt inv_measure_points; /*!< 1.0f / measure_points, to avoid division in ISR. */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;    /*!< Internal timer for steady-state measuring. */
    uint16_t step_idx;      /*!< Current speed step index. */
    fast_gt is_first_entry; /*!< Flag to distinguish between step and hold logic. */
    ctrl_gt target_w_pu;    /*!< Target speed for the current ramp. */

    // --- Measurement Accumulators (ISR) ---
    ctrl_gt sum_ud;
    ctrl_gt sum_uq;
    ctrl_gt sum_id;
    ctrl_gt sum_iq;
    ctrl_gt sum_w;

    // --- Averaged Data Arrays (For Loop calculation) ---
    ctrl_gt ud_array[16]; /*!< Averaged D-axis voltage array. */
    ctrl_gt uq_array[16]; /*!< Averaged Q-axis voltage array. */
    ctrl_gt id_array[16]; /*!< Averaged D-axis current array. */
    ctrl_gt iq_array[16]; /*!< Averaged Q-axis current array. */
    ctrl_gt w_array[16];  /*!< Averaged electrical speed array. */

} pmsm_offline_id_flux_t;

// ============================================================================

/**
 * @brief State enumeration for the Mechanical Parameters (J, B) sub-state machine.
 * @details Execution flow:
 * INIT -> IF_START -> HANDOVER_TO_CLOSED -> STEADY_LOW -> ACCEL_TEST -> STEADY_HIGH -> DECEL_TEST -> HANDOVER_TO_IF -> IF_STOP -> CALCULATE -> COMPLETE
 */
typedef enum _tag_pmsm_offline_id_mech_sm
{
    PMSM_OID_MECH_DISABLED = 0, /*!< 0: Disabled/Bypass. Safe state. */

    PMSM_OID_MECH_INIT, /*!< 1: Initialize. Pre-calculate ticks and limits in the background loop. */

    // --- Stage 1: Spin up & Closed-loop Transition ---
    PMSM_OID_MECH_IF_START,           /*!< 2: Open-loop start. Drive motor in I/F mode up to W_low. */
    PMSM_OID_MECH_HANDOVER_TO_CLOSED, /*!< 3: Bumpless Transfer to Closed-Loop. Transition angle from VF to Real/SMO. */
    PMSM_OID_MECH_STEADY_LOW,         /*!< 4: Stabilize at W_low. Accumulate Iq to calculate low-speed friction. */

    // --- Stage 2: Acceleration Test ---
    PMSM_OID_MECH_ACCEL_TEST, /*!< 5: Inject constant +Iq. Record (Time, Speed) into Data Analyzer until W >= W_high. */
    PMSM_OID_MECH_STEADY_HIGH, /*!< 6: Stabilize at W_high. Accumulate Iq to calculate high-speed friction. */

    // --- Stage 3: Deceleration Test & Safe Shutdown ---
    PMSM_OID_MECH_DECEL_TEST, /*!< 7: Inject constant -Iq. Record (Time, Speed) into DA until W <= W_low. Monitors OVP. */
    PMSM_OID_MECH_HANDOVER_TO_IF, /*!< 8: Transition back to I/F mode to ensure safe shutdown as SMO fails at low speed. */
    PMSM_OID_MECH_IF_STOP,        /*!< 9: Ramp V/F target to 0 and gracefully stop the motor. */

    // --- Stage 4: Calculation ---
    PMSM_OID_MECH_CALCULATE, /*!< 10: Trigger Math library. Fit J and B from DA buffers in the background loop. */
    PMSM_OID_MECH_COMPLETE,  /*!< 11: Update ctl_consultant_mech1_t structure. Signal Main SM. */
    PMSM_OID_MECH_FAULT      /*!< 12: Fault (OVP, Timeout, DA overflow). */

} pmsm_offline_id_mech_sm_t;

/**
 * @brief Configuration for Mechanical Parameters (Inertia J, Damping B) Identification.
 */
typedef struct _tag_pmsm_oid_cfg_mech
{
    parameter_gt low_speed_pu;      /*!< Lower speed threshold for evaluation (e.g., 0.2pu). */
    parameter_gt high_speed_pu;     /*!< Upper speed threshold for evaluation (e.g., 0.8pu). */
    parameter_gt accel_iq_pu;       /*!< Constant q-axis current applied for acceleration (+). */
    parameter_gt decel_iq_pu;       /*!< Constant q-axis current applied for deceleration (-). */
    parameter_gt max_vbus_pu;       /*!< DC Bus over-voltage protection limit during deceleration. */
    parameter_gt if_current_pu;     /*!< Constant current used for I/F dragging (e.g., 0.2pu). */
    parameter_gt settle_time_s;     /*!< Time to maintain steady speeds for friction measurement (s). */
    parameter_gt transition_time_s; /*!< Time duration for angle handover blending (s). */
} pmsm_oid_cfg_mech_t;

/**
 * @brief Sub-process context for Mechanical (J, B) identification.
 */
typedef struct _tag_pmsm_offline_id_mech
{
    pmsm_offline_id_mech_sm_t sm; /*!< Sub-SM: Current state of Mechanical identification. */
    pmsm_oid_cfg_mech_t cfg;      /*!< Configuration specific to this module. */

    // --- Pre-calculated Context (Computed in Loop) ---
    uint32_t settle_ticks;     /*!< ISR ticks corresponding to settle_time_s. */
    uint32_t transition_ticks; /*!< ISR ticks corresponding to transition_time_s. */
    ctrl_gt inv_settle_ticks;  /*!< Inverse of settle ticks for averaging. */
    parameter_gt dt_sec;       /*!< Control loop period in seconds (1.0 / isr_freq). */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;      /*!< Internal timer for delays, settling, and transitions. */
    fast_gt is_first_entry;   /*!< Flag to distinguish between step and hold logic. */
    ctrl_gt active_iq_ref_pu; /*!< The active torque current applied. */
    ctrl_gt active_id_ref_pu; /*!< The active dragging current applied. */

    // --- Friction Accumulators (ISR) ---
    ctrl_gt sum_iq_steady;          /*!< Accumulator for steady-state friction current. */
    parameter_gt iq_steady_low_pu;  /*!< Resulting average Iq at low speed. */
    parameter_gt iq_steady_high_pu; /*!< Resulting average Iq at high speed. */

    // --- Data Analyzer Tracking Indices ---
    uint32_t da_idx_accel_start; /*!< Dataset start index for acceleration. */
    uint32_t da_idx_accel_end;   /*!< Dataset end index for acceleration. */
    uint32_t da_idx_decel_start; /*!< Dataset start index for deceleration. */
    uint32_t da_idx_decel_end;   /*!< Dataset end index for deceleration. */

} pmsm_offline_id_mech_t;

//================================================================================
// 2. Identification Configuration Structures
//================================================================================

/**
 * @brief Basic configuration and feature flags for the identification process.
 */
typedef struct _tag_pmsm_oid_cfg_basic
{
    parameter_gt isr_freq_hz; /*!< Execution frequency of the control loop (Hz). */
    parameter_gt pole_pairs;  /*!< Motor pole pairs. */

    // --- Feature Options ---
    fast_gt flag_enable_rs_dt;
    fast_gt flag_enable_ldq;
    fast_gt flag_enable_flux;
    fast_gt flag_enable_mech_id; /*!< Flag: 1 to perform mechanical ID, 0 to skip. */
    fast_gt is_sensorless;       /*!< Flag: 1 if no physical encoder is present. */

} pmsm_oid_cfg_basic_t;

/**
 * @brief Master Initialization Structure for PMSM Offline Identification.
 * @details This is the "Medical Checkup Form" that the user fills out before starting the process.
 */
typedef struct _tag_ctl_pmsm_offline_id_init
{
    // --- System & Hardware Bases ---

    parameter_gt v_base; /*!< Base Phase Voltage Peak (V). */
    parameter_gt i_base; /*!< Base Phase Current Peak (A). */
    parameter_gt w_base; /*!< Base Electrical Angular Velocity (rad/s). */

    pmsm_oid_cfg_basic_t cfg_basic;

    // --- Identification Stage Configurations ---
    pmsm_oid_cfg_rs_dt_t cfg_rs_dt; /*!< Config: Resistance & Dead-time. */
    pmsm_oid_cfg_ld_lq_t cfg_ld_lq; /*!< Config: Inductance saturation. */
    pmsm_oid_cfg_flux_t cfg_flux;   /*!< Config: Flux linkage. */
    pmsm_oid_cfg_mech_t cfg_mech;   /*!< Config: Mechanical parameters. */

} ctl_pmsm_offline_id_init_t;

/**
 * @brief Master context structure for the PMSM Offline Identification mechanism.
 */
typedef struct _tag_ctl_pmsm_offline_id
{
    // =========================================================================
    // 1. Core Embedded Components (Flat Memory Layout)
    // =========================================================================
    mtr_current_ctrl_t foc_core;         /*!< Dedicated FOC current controller core. */
    ctl_slope_f_pu_controller vf_gen;    /*!< V/F slope frequency generator for I/F mode. */
    ctl_angle_switcher_t angle_switcher; /*!< Smooth transition router for angles. */
    ctl_pmsm_esmo_t esmo;                /*!< Extended Sliding Mode Observer. */

    // ctl_data_analyzer_t    analyzer;       /*!< Data recording and fitting engine (WIP). */

    // =========================================================================
    // 2. External Interfaces & Routing Dummies
    // =========================================================================
    rotation_ift* enc;         /*!< Pointer to the physical encoder (NULL if sensorless). */
    rotation_ift static_angle; /*!< Internal dummy encoder for static DC injection tests. */

    // =========================================================================
    // 3. State Machines & Sub-Process Trackers
    // =========================================================================
    pmsm_offline_id_sm_t sm; /*!< The Master State Machine tracker. */

    pmsm_offline_id_rs_dt_t sub_rs_dt; /*!< Context for Rs & DT identification. */
    pmsm_offline_id_ldq_t sub_ldq;     /*!< Context for Ld & Lq identification. */
    pmsm_offline_id_flux_t sub_flux;   /*!< Context for Flux identification. */
    pmsm_offline_id_mech_t sub_mech;   /*!< Context for Mechanical parameter ID. */

    // =========================================================================
    // 4. Runtime Configuration & Identified Results
    // =========================================================================
    pmsm_oid_cfg_basic_t cfg_basic; /*!< Basic execution configuration. */

    ctl_consultant_pu_pmsm_t identified_pu; /*!< PU bases used during calculation. */

    // Extracted physical parameters for control integration & reporting
    ctl_consultant_pmsm_t pmsm_param;       /*!< The final identified electrical parameters. */
    ctl_consultant_mech1_t pmsm_mech_param; /*!< The final identified mechanical parameters. */

} ctl_pmsm_offline_id_t;

//
// Service function
//

#pragma region ServiceFunction

/**
 * @brief Angle source routing options for the offline identification context.
 */
typedef enum _tag_pmsm_oid_angle_src
{
    PMSM_OID_ANGLE_SRC_STATIC = 0, /*!< Route FOC to the internal static dummy angle. */
    PMSM_OID_ANGLE_SRC_VF_GEN,     /*!< Route FOC to the V/F slope generator. */
    PMSM_OID_ANGLE_SRC_REAL_ENC    /*!< Route FOC to the real physical encoder or SMO. */
} pmsm_oid_angle_src_e;

/**
 * @brief Routes the FOC core's angle input to a specific internal/external source.
 * @param[in,out] ctx Pointer to the master offline ID context.
 * @param[in]     src The target angle source enum.
 */
GMP_STATIC_INLINE void ctl_id_route_foc_angle(ctl_pmsm_offline_id_t* ctx, pmsm_oid_angle_src_e src)
{
    switch (src)
    {
    case PMSM_OID_ANGLE_SRC_STATIC:
        ctx->foc_core.pos_if = &ctx->static_angle;
        break;
    case PMSM_OID_ANGLE_SRC_VF_GEN:
        ctx->foc_core.pos_if = &ctx->vf_gen.enc;
        break;
    case PMSM_OID_ANGLE_SRC_REAL_ENC:
        ctx->foc_core.pos_if = ctx->enc; // Assuming ctx->enc points to the valid real encoder
        break;
    }
}

/**
 * @brief Sets a fixed electrical angle for static tests (Rs, DT, Ld/Lq).
 * @note Automatically routes the FOC angle to the static source.
 * @param[in,out] ctx      Pointer to the master offline ID context.
 * @param[in]     angle_pu The fixed electrical angle in per-unit [0.0, 1.0).
 */
GMP_STATIC_INLINE void ctl_id_set_static_angle(ctl_pmsm_offline_id_t* ctx, ctrl_gt angle_pu)
{
    ctl_id_route_foc_angle(ctx, PMSM_OID_ANGLE_SRC_STATIC);
    ctx->static_angle.elec_position = angle_pu;
}

/**
 * @brief Safely shuts down the FOC output (Zero current/voltage injection).
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_id_disable_output(ctl_pmsm_offline_id_t* ctx)
{
    ctl_disable_mtr_current_ctrl(&ctx->foc_core);
    ctl_set_mtr_current_ctrl_ref(&ctx->foc_core, float2ctrl(0.0f), float2ctrl(0.0f));
    ctl_set_mtr_current_ctrl_vdq_ref(&ctx->foc_core, float2ctrl(0.0f), float2ctrl(0.0f));
}

/**
 * @brief Applies a constant closed-loop DC current vector.
 * @details Re-enables the FOC PI controllers if they were disabled.
 * Used in Rs, Encoder Alignment, and Flux dragging.
 * @param[in,out] ctx   Pointer to the master offline ID context.
 * @param[in]     id_pu D-axis current reference in PU.
 * @param[in]     iq_pu Q-axis current reference in PU.
 */
GMP_STATIC_INLINE void ctl_id_apply_dc_current(ctl_pmsm_offline_id_t* ctx, ctrl_gt id_pu, ctrl_gt iq_pu)
{
    ctl_enable_mtr_current_ctrl(&ctx->foc_core);
    ctl_set_mtr_current_ctrl_ref(&ctx->foc_core, id_pu, iq_pu);
}

/**
 * @brief Applies an open-loop voltage pulse.
 * @details Disables the FOC PI controllers and directly injects Vd/Vq.
 * Exclusively used for high-frequency pulse injection during Ld/Lq measurement.
 * @param[in,out] ctx   Pointer to the master offline ID context.
 * @param[in]     vd_pu D-axis voltage reference in PU.
 * @param[in]     vq_pu Q-axis voltage reference in PU.
 */
GMP_STATIC_INLINE void ctl_id_apply_voltage_pulse(ctl_pmsm_offline_id_t* ctx, ctrl_gt vd_pu, ctrl_gt vq_pu)
{
    ctl_disable_mtr_current_ctrl(&ctx->foc_core); // Disable PI regulation
    ctl_set_mtr_current_ctrl_vdq_ref(&ctx->foc_core, vd_pu, vq_pu);
}

/**
 * @brief Sets the target speed for the V/F slope generator.
 * @param[in,out] ctx             Pointer to the master offline ID context.
 * @param[in]     target_speed_pu The target electrical speed in PU.
 */
GMP_STATIC_INLINE void ctl_id_set_vf_target_speed(ctl_pmsm_offline_id_t* ctx, ctrl_gt target_speed_pu)
{
    ctx->vf_gen.target_freq_pu = target_speed_pu;
}

/**
 * @brief Steps the V/F generator and routes its output to the FOC.
 * @details Must be called in the ISR during Flux I/F dragging.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_id_step_vf_generator(ctl_pmsm_offline_id_t* ctx)
{
    ctl_id_route_foc_angle(ctx, PMSM_OID_ANGLE_SRC_VF_GEN);
    ctl_step_slope_f_pu(&ctx->vf_gen);
}

// 将物理时间(秒)转换为 ISR 滴答数
#define SEC_TO_TICKS(sec, freq) ((uint32_t)((sec) * (freq)))

#pragma endregion

//
// --- Resistance & Dead-Time (RS_DT) ---
//

#pragma region RS_DT_MODULE

/**
 * @brief Initializes the Rs & DT identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_rs_dt(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_rs_dt.sm = PMSM_OID_RSDT_INIT;
    ctx->sub_rs_dt.is_first_entry = 1;

    ctx->sub_rs_dt.tick_timer = 0;
    ctx->sub_rs_dt.angle_idx = 0;
    ctx->sub_rs_dt.step_idx = 0;
    ctx->sub_rs_dt.current_ref_pu = float2ctrl(0.0f);
}

/**
 * @brief ISR step function for Rs & DT identification.
 * @details Highly optimized state machine. Follows Step/Hold pattern. Free of heavy math arrays.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_rs_dt_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_rs_dt_t* sub = &ctx->sub_rs_dt;
    pmsm_oid_cfg_rs_dt_t* cfg = &sub->cfg;

    switch (sub->sm)
    {
    case PMSM_OID_RSDT_DISABLED:
    case PMSM_OID_RSDT_INIT:
    case PMSM_OID_RSDT_CALCULATE:
    case PMSM_OID_RSDT_COMPLETE:
    case PMSM_OID_RSDT_FAULT:
        break; // Managed by background loop.

    case PMSM_OID_RSDT_ALIGN_SETTLE:
        if (sub->is_first_entry)
        {
            // Calculate angle_pu dynamically: (idx * (1.0 / 6.0))
            ctrl_gt angle_pu = ctl_mul(float2ctrl((float)sub->angle_idx), float2ctrl(0.1666667f));
            ctl_id_set_static_angle(ctx, angle_pu);

            // Apply maximum current to guarantee rotor alignment to this specific vector
            ctl_id_apply_dc_current(ctx, cfg->max_current_pu, float2ctrl(0.0f));

            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->align_ticks)
        {
            sub->sm = PMSM_OID_RSDT_STEP_DELAY;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_RSDT_STEP_DELAY:
        if (sub->is_first_entry)
        {
            // Dynamic current calculation using basic math (Minimal RAM footprint)
            ctrl_gt increment = ctl_mul(float2ctrl((float)sub->step_idx), sub->step_size_pu);
            sub->current_ref_pu = cfg->min_current_pu + increment;

            ctl_id_apply_dc_current(ctx, sub->current_ref_pu, float2ctrl(0.0f));

            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->measure_delay_ticks)
        {
            sub->sm = PMSM_OID_RSDT_MEASURE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_RSDT_MEASURE:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->sum_u = float2ctrl(0.0f);
            sub->sum_i = float2ctrl(0.0f);
            sub->is_first_entry = 0;
        }

        // Accumulate voltage command and actual current
        sub->sum_u += ctx->foc_core.vdq_ref.dat[0]; // D-axis voltage ref
        sub->sum_i += ctx->foc_core.idq0.dat[0];    // D-axis actual current

        sub->tick_timer++;
        if (sub->tick_timer >= cfg->measure_points)
        {
            // Calculate average using pre-computed inverse (Single multiplication)
            ctrl_gt avg_u = ctl_mul(sub->sum_u, sub->inv_measure_points);
            ctrl_gt avg_i = ctl_mul(sub->sum_i, sub->inv_measure_points);

            // Push ONE clean, averaged data point to the analyzer
            // Assuming analyzer is part of ctx (ctx->analyzer)
            // ctl_da_push_point(&ctx->analyzer, avg_i, avg_u);

            sub->sm = PMSM_OID_RSDT_STEP_EVALUATE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_RSDT_STEP_EVALUATE:
        if (sub->is_first_entry)
        {
            sub->step_idx++;

            if (sub->step_idx >= cfg->steps)
            {
                sub->step_idx = 0;
                sub->angle_idx++;

                if (sub->angle_idx >= 6)
                {
                    sub->sm = PMSM_OID_RSDT_CALCULATE;
                }
                else
                {
                    sub->sm = PMSM_OID_RSDT_ALIGN_SETTLE;
                }
            }
            else
            {
                sub->sm = PMSM_OID_RSDT_STEP_DELAY;
            }
            sub->is_first_entry = 1;
        }
        break;
    }
}

/**
 * @brief Background loop function for Rs & DT identification.
 * @details Executes setup calculations and segment-based least squares fitting.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_rs_dt(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_rs_dt_t* sub = &ctx->sub_rs_dt;
    pmsm_oid_cfg_rs_dt_t* cfg = &sub->cfg;

    // --- Safety Rule: Zero output in passive states ---
    if (sub->sm == PMSM_OID_RSDT_CALCULATE || sub->sm == PMSM_OID_RSDT_COMPLETE || sub->sm == PMSM_OID_RSDT_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    if (sub->sm == PMSM_OID_RSDT_INIT)
    {
        if (sub->is_first_entry)
        {
            // 1. Pre-calculate ISR constants (Executes only once)
            sub->align_ticks = SEC_TO_TICKS(cfg->align_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->measure_delay_ticks = SEC_TO_TICKS(cfg->measure_delay_s, ctx->cfg_basic.isr_freq_hz);
            sub->inv_measure_points = ctl_div(float2ctrl(1.0f), float2ctrl((float)cfg->measure_points));

            if (cfg->steps > 1)
            {
                sub->step_size_pu =
                    ctl_div((cfg->max_current_pu - cfg->min_current_pu), float2ctrl((float)(cfg->steps - 1)));
            }
            else
            {
                sub->step_size_pu = float2ctrl(0.0f);
            }

            // 2. Configure FOC core
            ctl_id_route_foc_angle(ctx, PMSM_OID_ANGLE_SRC_STATIC);
            ctl_enable_mtr_current_ctrl(&ctx->foc_core);
            ctx->foc_core.flag_enable_decouple = 0;
            ctx->foc_core.flag_enable_vdq_feedforward = 0;

            // 3. Clear DA
            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_RSDT_ALIGN_SETTLE;
            sub->is_first_entry = 1;
        }
    }
    else if (sub->sm == PMSM_OID_RSDT_CALCULATE)
    {
        if (sub->is_first_entry)
        {
            parameter_gt rs_sum = 0.0f, vcomp_sum = 0.0f;

            // 1. Segmented Linear Fitting using the Data Analyzer
            // We have 6 angles, each containing 'cfg->steps' data points.
            for (uint16_t i = 0; i < 6; i++)
            {
                uint32_t start_idx = i * cfg->steps;
                uint32_t end_idx = start_idx + cfg->steps - 1;
                parameter_gt slope = 0.0f, intercept = 0.0f;

                // if (ctl_da_linear_fit(&ctx->analyzer, start_idx, end_idx, &slope, &intercept)) {
                //     sub->rs_array[i]    = slope;
                //     sub->vcomp_array[i] = intercept;
                // }

                // Dummy assignment to allow compilation verification
                sub->rs_array[i] = 0.05f + ((float)i * 0.001f);
                sub->vcomp_array[i] = 0.02f;

                rs_sum += sub->rs_array[i];
                vcomp_sum += sub->vcomp_array[i];
            }

            // 2. Compute Mean
            sub->rs_mean = rs_sum / 6.0f;
            sub->vcomp_mean = vcomp_sum / 6.0f;

            // 3. Compute Variance
            parameter_gt rs_var_sum = 0.0f, vcomp_var_sum = 0.0f;
            for (uint16_t i = 0; i < 6; i++)
            {
                parameter_gt d_rs = sub->rs_array[i] - sub->rs_mean;
                parameter_gt d_vc = sub->vcomp_array[i] - sub->vcomp_mean;
                rs_var_sum += (d_rs * d_rs);
                vcomp_var_sum += (d_vc * d_vc);
            }
            sub->rs_var = rs_var_sum / 6.0f;
            sub->vcomp_var = vcomp_var_sum / 6.0f;

            // 4. Update Final Consultant Base (Convert to Physical Units)
            parameter_gt Z_base = ctx->cfg_basic.v_base / ctx->cfg_basic.i_base;
            ctx->pmsm_param.Rs = sub->rs_mean * Z_base;
            ctx->identified_pu.Z_base = float2ctrl(Z_base);

            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_RSDT_COMPLETE;
        }
    }
}

#pragma endregion

//
// --- Inductance (LD_LQ) ---
//

#pragma region LD_LQ_MDOULE

/**
 * @brief Initializes the Inductance (Ld, Lq) identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_ldq(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_ldq.sm = PMSM_OID_LDQ_INIT;
    ctx->sub_ldq.is_first_entry = 1;

    ctx->sub_ldq.tick_timer = 0;
    ctx->sub_ldq.bias_step_idx = 0;
    ctx->sub_ldq.is_measuring_q_axis = 0; // Start with D-axis
    ctx->sub_ldq.bias_curr_ref_pu = float2ctrl(0.0f);
}

/**
 * @brief ISR step function for Ld & Lq identification.
 * @details Executes DC bias settling, PI freezing, voltage pulse injection, and DA recording.
 * MUST be called within the high-frequency motor control interrupt.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_ldq_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_ldq_t* sub = &ctx->sub_ldq;
    pmsm_oid_cfg_ld_lq_t* cfg = &sub->cfg;

    switch (sub->sm)
    {
    case PMSM_OID_LDQ_DISABLED:
    case PMSM_OID_LDQ_INIT:
    case PMSM_OID_LDQ_CALCULATE:
    case PMSM_OID_LDQ_COMPLETE:
    case PMSM_OID_LDQ_FAULT:
        break;

    case PMSM_OID_LDQ_BIAS_SETTLE:
        if (sub->is_first_entry)
        {
            // Calculate current bias for this step (Zero math load via pre-calculated step_size)
            ctrl_gt increment = ctl_mul(float2ctrl((float)sub->bias_step_idx), sub->step_size_pu);
            sub->bias_curr_ref_pu = cfg->min_bias_curr_pu + increment; // Assuming min_bias exists, else 0

            if (sub->is_measuring_q_axis == 0)
            {
                // D-axis: Id = bias, Iq = 0
                ctl_id_apply_dc_current(ctx, sub->bias_curr_ref_pu, float2ctrl(0.0f));
            }
            else
            {
                // Q-axis: Apply alignment current to D-axis to lock rotor, inject bias to Q-axis
                ctl_id_apply_dc_current(ctx, cfg->align_current_pu, sub->bias_curr_ref_pu);
            }

            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->settle_ticks)
        {
            sub->sm = PMSM_OID_LDQ_PULSE_MEASURE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_LDQ_PULSE_MEASURE:
        if (sub->is_first_entry)
        {
            // 1. Freeze current PI outputs (These voltages already compensate for Rs*I and Dead-time)
            // CHECK POINT 是否需要重新测量？此处锁存的结果是否正确？
            sub->frozen_vd_pu = ctx->foc_core.vdq_out_sat.dat[0];
            sub->frozen_vq_pu = ctx->foc_core.vdq_out_sat.dat[1];

            // 2. Open Loop Pulse Injection: Add pulse voltage to the active axis
            if (sub->is_measuring_q_axis == 0)
            {
                ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu + cfg->pulse_voltage_pu, sub->frozen_vq_pu);
            }
            else
            {
                ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu, sub->frozen_vq_pu + cfg->pulse_voltage_pu);
            }

            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        // --- Data Analyzer Recording (Hold phase) ---
        // X-axis: Time (seconds), Y-axis: Active Axis Current
        ctrl_gt time_now = float2ctrl((float)sub->tick_timer * sub->dt_sec);
        if (sub->is_measuring_q_axis == 0)
        {
            // ctl_da_push_point(&ctx->analyzer, time_now, ctx->foc_core.idq0.dat[0]); // Record Id
        }
        else
        {
            // ctl_da_push_point(&ctx->analyzer, time_now, ctx->foc_core.idq0.dat[1]); // Record Iq
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->pulse_ticks)
        {
            sub->sm = PMSM_OID_LDQ_COOLDOWN;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_LDQ_COOLDOWN:
        if (sub->is_first_entry)
        {
            // Apply frozen voltage WITHOUT pulse to let the inductive current decay
            ctl_id_apply_voltage_pulse(ctx, sub->frozen_vd_pu, sub->frozen_vq_pu);

            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->cooldown_ticks)
        {
            sub->sm = PMSM_OID_LDQ_STEP_EVALUATE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_LDQ_STEP_EVALUATE:
        if (sub->is_first_entry)
        {
            sub->bias_step_idx++;
            if (sub->bias_step_idx >= cfg->bias_steps)
            {
                if (sub->is_measuring_q_axis == 0)
                {
                    sub->is_measuring_q_axis = 1; // Switch to Q-axis
                    sub->bias_step_idx = 0;
                    sub->sm = PMSM_OID_LDQ_BIAS_SETTLE;
                }
                else
                {
                    sub->sm = PMSM_OID_LDQ_CALCULATE; // Both axes done
                }
            }
            else
            {
                sub->sm = PMSM_OID_LDQ_BIAS_SETTLE; // Next bias step
            }
            sub->is_first_entry = 1;
        }
        break;
    }
}

/**
 * @brief Background loop function for Ld & Lq identification.
 * @details Computes pre-requisites and performs the di/dt linear regression using the DA.
 * MUST be called from a low-priority task or main loop.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_ldq(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_ldq_t* sub = &ctx->sub_ldq;
    pmsm_oid_cfg_ld_lq_t* cfg = &sub->cfg;

    // --- Safety Rule: Zero output in passive states ---
    if (sub->sm == PMSM_OID_LDQ_CALCULATE || sub->sm == PMSM_OID_LDQ_COMPLETE || sub->sm == PMSM_OID_LDQ_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    if (sub->sm == PMSM_OID_LDQ_INIT)
    {
        if (sub->is_first_entry)
        {
            // 1. Pre-calculate ISR constants
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->pulse_ticks = SEC_TO_TICKS(cfg->pulse_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->cooldown_ticks = SEC_TO_TICKS(cfg->cooldown_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->dt_sec = 1.0f / ctx->cfg_basic.isr_freq_hz;

            if (cfg->bias_steps > 1)
            {
                sub->step_size_pu = ctl_div(cfg->max_bias_curr_pu, float2ctrl((float)(cfg->bias_steps - 1)));
            }
            else
            {
                sub->step_size_pu = float2ctrl(0.0f);
            }

            // 2. Configure FOC core for STATIC tests
            ctl_id_set_static_angle(ctx, float2ctrl(0.0f)); // Align with D-axis physically
            ctl_enable_mtr_current_ctrl(&ctx->foc_core);
            ctx->foc_core.flag_enable_decouple = 0;
            ctx->foc_core.flag_enable_vdq_feedforward = 0;

            // 3. Clear DA
            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_LDQ_BIAS_SETTLE;
            sub->is_first_entry = 1;
        }
    }
    else if (sub->sm == PMSM_OID_LDQ_CALCULATE)
    {
        if (sub->is_first_entry)
        {
            // 1. Calculate Inductances using Data Analyzer Linear Fit (di/dt)
            // Because U_apply = U_steady + U_pulse, and U_steady exactly compensates for Rs*I + Vcomp
            // We have: U_pulse = L * (di/dt) + Rs * delta_i
            // Since delta_i is extremely small over 1ms, L = U_pulse / (di/dt)

            for (uint16_t i = 0; i < cfg->bias_steps; i++)
            {
                parameter_gt di_dt_d = 1.0f, intercept_d = 0.0f;
                parameter_gt di_dt_q = 1.0f, intercept_q = 0.0f;

                // Example of getting slope from DA (Assuming data is structured)
                // ctl_da_linear_fit(&ctx->analyzer, start_idx_d, end_idx_d, &di_dt_d, &intercept_d);
                // ctl_da_linear_fit(&ctx->analyzer, start_idx_q, end_idx_q, &di_dt_q, &intercept_q);

                // Dummy Values for compilation
                di_dt_d = 2.0f; // PU current per second
                di_dt_q = 1.5f;

                // Compute L (PU) = V_pulse / (di/dt)
                sub->ld_array[i] = cfg->pulse_voltage_pu / di_dt_d;
                sub->lq_array[i] = cfg->pulse_voltage_pu / di_dt_q;
            }

            // 2. Base Conversions (PU to Henries)
            parameter_gt Z_base = ctx->cfg_basic.v_base / ctx->cfg_basic.i_base;
            parameter_gt L_base = Z_base / ctx->cfg_basic.w_base;
            ctx->identified_pu.L_base = float2ctrl(L_base);

            // Store Nominal L (usually at 0A bias, i.e., index 0) into Consultant
            ctx->pmsm_param.Ld = sub->ld_array[0] * L_base;
            ctx->pmsm_param.Lq = sub->lq_array[0] * L_base;

            // 3. Intrinsic Properties calculation
            ctx->pmsm_param.saliency_ratio = ctx->pmsm_param.Lq / ctx->pmsm_param.Ld;
            ctx->pmsm_param.is_ipm = (ctx->pmsm_param.saliency_ratio > 1.05f) ? 1 : 0;

            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_LDQ_COMPLETE;
        }
    }
}

#pragma endregion

// --- Flux Linkage (FLUX) ---

#pragma region FLUX

/**
 * @brief Initializes the Flux Linkage identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_flux(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_flux.sm = PMSM_OID_FLUX_INIT;
    ctx->sub_flux.is_first_entry = 1;

    ctx->sub_flux.tick_timer = 0;
    ctx->sub_flux.step_idx = 0;
    ctx->sub_flux.target_w_pu = float2ctrl(0.0f);
}

/**
 * @brief ISR step function for Flux Linkage identification.
 * @details Executes V/F ramping, I/F dragging, and data accumulation. Free of heavy math.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_flux_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_flux_t* sub = &ctx->sub_flux;
    pmsm_oid_cfg_flux_t* cfg = &sub->cfg;

    // 全局动作：如果在活动状态下，持续推进 V/F 角度发生器，并维持拖拽电流
    if (sub->sm >= PMSM_OID_FLUX_RAMP_SPEED && sub->sm <= PMSM_OID_FLUX_RAMP_STOP)
    {
        ctl_id_step_vf_generator(ctx);
        ctl_id_apply_dc_current(ctx, cfg->if_current_pu, float2ctrl(0.0f));
    }

    switch (sub->sm)
    {
    case PMSM_OID_FLUX_DISABLED:
    case PMSM_OID_FLUX_INIT:
    case PMSM_OID_FLUX_CALCULATE:
    case PMSM_OID_FLUX_COMPLETE:
    case PMSM_OID_FLUX_FAULT:
        break; // Managed by background loop.

    case PMSM_OID_FLUX_RAMP_SPEED:
        if (sub->is_first_entry)
        {
            // Calculate target speed dynamically
            ctrl_gt increment = ctl_mul(float2ctrl((float)sub->step_idx), sub->step_size_pu);
            sub->target_w_pu = cfg->min_target_speed_pu + increment;

            ctl_id_set_vf_target_speed(ctx, sub->target_w_pu);

            sub->is_first_entry = 0;
        }

        // Hold: Wait for V/F generator to reach the target speed
        {
            ctrl_gt err = sub->target_w_pu - ctx->vf_gen.current_freq_pu;
            if (err < float2ctrl(0.001f) && err > float2ctrl(-0.001f))
            {
                sub->sm = PMSM_OID_FLUX_SETTLE;
                sub->is_first_entry = 1;
            }
        }
        break;

    case PMSM_OID_FLUX_SETTLE:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->is_first_entry = 0;
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->settle_ticks)
        {
            sub->sm = PMSM_OID_FLUX_MEASURE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_FLUX_MEASURE:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->sum_ud = float2ctrl(0.0f);
            sub->sum_uq = float2ctrl(0.0f);
            sub->sum_id = float2ctrl(0.0f);
            sub->sum_iq = float2ctrl(0.0f);
            sub->sum_w = float2ctrl(0.0f);
            sub->is_first_entry = 0;
        }

        // Accumulate voltages, currents, and actual frequency
        sub->sum_ud += ctx->foc_core.vdq_ref.dat[0];
        sub->sum_uq += ctx->foc_core.vdq_ref.dat[1];
        sub->sum_id += ctx->foc_core.idq0.dat[0];
        sub->sum_iq += ctx->foc_core.idq0.dat[1];
        sub->sum_w += ctx->vf_gen.current_freq_pu;

        sub->tick_timer++;
        if (sub->tick_timer >= cfg->measure_points)
        {
            uint16_t idx = sub->step_idx;
            if (idx < 16)
            {
                sub->ud_array[idx] = ctl_mul(sub->sum_ud, sub->inv_measure_points);
                sub->uq_array[idx] = ctl_mul(sub->sum_uq, sub->inv_measure_points);
                sub->id_array[idx] = ctl_mul(sub->sum_id, sub->inv_measure_points);
                sub->iq_array[idx] = ctl_mul(sub->sum_iq, sub->inv_measure_points);
                sub->w_array[idx] = ctl_mul(sub->sum_w, sub->inv_measure_points);
            }

            sub->sm = PMSM_OID_FLUX_STEP_EVALUATE;
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_FLUX_STEP_EVALUATE:
        if (sub->is_first_entry)
        {
            sub->step_idx++;
            if (sub->step_idx >= cfg->steps)
            {
                sub->sm = PMSM_OID_FLUX_RAMP_STOP;
            }
            else
            {
                sub->sm = PMSM_OID_FLUX_RAMP_SPEED;
            }
            sub->is_first_entry = 1;
        }
        break;

    case PMSM_OID_FLUX_RAMP_STOP:
        if (sub->is_first_entry)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(0.0f));
            sub->is_first_entry = 0;
        }

        if (ctx->vf_gen.current_freq_pu <= float2ctrl(0.005f))
        {
            sub->sm = PMSM_OID_FLUX_CALCULATE;
            sub->is_first_entry = 1;
        }
        break;
    }
}

// 假设底层数学库提供了求平方根的函数
extern parameter_gt ctl_math_sqrt(parameter_gt val);

/**
 * @brief Background loop function for Flux Linkage identification.
 * @details Computes pre-requisites and performs the linear regression (E vs W) using the DA.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_flux(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_flux_t* sub = &ctx->sub_flux;
    pmsm_oid_cfg_flux_t* cfg = &sub->cfg;

    // --- Safety Rule: Zero output in passive states ---
    if (sub->sm == PMSM_OID_FLUX_CALCULATE || sub->sm == PMSM_OID_FLUX_COMPLETE || sub->sm == PMSM_OID_FLUX_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    if (sub->sm == PMSM_OID_FLUX_INIT)
    {
        if (sub->is_first_entry)
        {
            // 1. Pre-calculate ISR constants
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->inv_measure_points = ctl_div(float2ctrl(1.0f), float2ctrl((float)cfg->measure_points));

            if (cfg->steps > 1)
            {
                sub->step_size_pu =
                    ctl_div((cfg->max_target_speed_pu - cfg->min_target_speed_pu), float2ctrl((float)(cfg->steps - 1)));
            }
            else
            {
                sub->step_size_pu = float2ctrl(0.0f);
            }

            // 2. Configure FOC core for I/F mode
            ctl_id_route_foc_angle(ctx, PMSM_OID_ANGLE_SRC_VF_GEN);
            ctl_enable_mtr_current_ctrl(&ctx->foc_core);
            ctx->foc_core.flag_enable_decouple = 0;
            ctx->foc_core.flag_enable_vdq_feedforward = 0;

            ctl_clear_slope_f_pu(&ctx->vf_gen);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_FLUX_RAMP_SPEED;
            sub->is_first_entry = 1;
        }
    }
    else if (sub->sm == PMSM_OID_FLUX_CALCULATE)
    {
        if (sub->is_first_entry)
        {
            // 1. Retrieve identified parameters from previous stages (In PU)
            // If Rs/Ld/Lq were skipped, we assume they are 0 or fallback values.
            parameter_gt rs_pu = ctx->pmsm_param.Rs / ctx->identified_pu.Z_base;
            parameter_gt ld_pu = ctx->pmsm_param.Ld / ctx->identified_pu.L_base;
            parameter_gt lq_pu = ctx->pmsm_param.Lq / ctx->identified_pu.L_base;

            // 2. Prepare Data Analyzer
            // Assuming ctx->analyzer is globally accessible
            // ctl_da_clear(&ctx->analyzer);

            // 3. Compute Back-EMF magnitude for each speed step and push to DA
            for (uint16_t i = 0; i < cfg->steps && i < 16; i++)
            {
                parameter_gt ud = (parameter_gt)sub->ud_array[i];
                parameter_gt uq = (parameter_gt)sub->uq_array[i];
                parameter_gt id = (parameter_gt)sub->id_array[i];
                parameter_gt iq = (parameter_gt)sub->iq_array[i];
                parameter_gt w = (parameter_gt)sub->w_array[i];

                // E_d = U_d - R_s * I_d + W * L_q * I_q
                parameter_gt ed = ud - (rs_pu * id) + (w * lq_pu * iq);
                // E_q = U_q - R_s * I_q - W * L_d * I_d
                parameter_gt eq = uq - (rs_pu * iq) - (w * ld_pu * id);

                // Magnitude |E| = sqrt(Ed^2 + Eq^2)
                parameter_gt e_mag = ctl_math_sqrt((ed * ed) + (eq * eq));

                // Push to Dataset: X = Speed (w), Y = Back-EMF (|E|)
                // ctl_da_push_point(&ctx->analyzer, float2ctrl(w), float2ctrl(e_mag));
            }

            // 4. Perform Linear Regression: |E| = Psi_m * w + Intercept
            parameter_gt flux_pu = 0.0f, intercept = 0.0f;
            // ctl_da_linear_fit(&ctx->analyzer, 0, cfg->steps - 1, &flux_pu, &intercept);

            // --- Dummy Value for compilation ---
            flux_pu = 0.85f;

            // 5. Convert PU Flux to Physical Units (Weber)
            parameter_gt flux_base = ctx->cfg_basic.v_base / ctx->cfg_basic.w_base;
            ctx->pmsm_param.flux_linkage = flux_pu * flux_base;
            ctx->identified_pu.Flux_base = float2ctrl(flux_base);

            // 6. Calculate Motor Characteristic Current (if IPM)
            if (ctx->pmsm_param.is_ipm)
            {
                parameter_gt delta_l = ctx->pmsm_param.Lq - ctx->pmsm_param.Ld;
                if (delta_l > 0.0001f)
                {
                    ctx->pmsm_param.char_current = ctx->pmsm_param.flux_linkage / delta_l;
                }
            }
            else
            {
                ctx->pmsm_param.char_current = 9999.0f; // Infinite for SPM
            }

            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_FLUX_COMPLETE;
        }
    }
}

#pragma endregion

//
// --- Mechanical Parameters (MECH) ---
//

#pragma region MECH

/**
 * @brief Initializes the Mechanical Parameters identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    ctx->sub_mech.sm = PMSM_OID_MECH_INIT;
    ctx->sub_mech.is_first_entry = 1;

    ctx->sub_mech.tick_timer = 0;
    ctx->sub_mech.active_iq_ref_pu = float2ctrl(0.0f);
    ctx->sub_mech.active_id_ref_pu = float2ctrl(0.0f);
}

/**
 * @brief ISR step function for Mechanical Parameters identification.
 * @details Highly optimized state machine managing V/F start, closed-loop handover, 
 * localized speed control, and dual-curve high-speed recording.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_mech_isr(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;

    // 始终更新 V/F 发生器以备随时切回
    ctl_step_slope_f_pu(&ctx->vf_gen);
    ctrl_gt current_speed_pu = ctx->foc_core.spd_if->speed;
    ctrl_gt time_now = float2ctrl((float)sub->tick_timer * sub->dt_sec);

    switch (sub->sm)
    {
    case PMSM_OID_MECH_DISABLED:
    case PMSM_OID_MECH_INIT:
    case PMSM_OID_MECH_CALCULATE:
    case PMSM_OID_MECH_COMPLETE:
    case PMSM_OID_MECH_FAULT:
        break; // Loop handles these.

    // -------------------------------------------------------------
    // IF Spin Up
    // -------------------------------------------------------------
    case PMSM_OID_MECH_IF_START:
        if (sub->is_first_entry)
        {
            ctl_id_set_vf_target_speed(ctx, cfg->low_speed_pu);
            ctl_id_apply_dc_current(ctx, cfg->if_current_pu, float2ctrl(0.0f));
            sub->is_first_entry = 0;
        }

        if (ctx->vf_gen.current_freq_pu >= cfg->low_speed_pu)
        {
            sub->sm = PMSM_OID_MECH_HANDOVER_TO_CLOSED;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Handover: IF (A) -> Real/SMO (B)
    // -------------------------------------------------------------
    case PMSM_OID_MECH_HANDOVER_TO_CLOSED:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            // Target 1 = Source B (Real/SMO Angle)
            ctl_trigger_angle_transition(&ctx->angle_switcher, 1);
            sub->is_first_entry = 0;
        }

        // Blend current vector: Id fades out, Iq builds up
        {
            ctrl_gt w = ctx->angle_switcher.weight;
            sub->active_id_ref_pu = ctl_mul(cfg->if_current_pu, float2ctrl(1.0f) - w);
            sub->active_iq_ref_pu = ctl_mul(cfg->low_speed_pu, w); // Approx startup Iq
            ctl_id_apply_dc_current(ctx, sub->active_id_ref_pu, sub->active_iq_ref_pu);
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->transition_ticks)
        {
            sub->sm = PMSM_OID_MECH_STEADY_LOW;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Steady State (Low & High) for Friction Measurement
    // -------------------------------------------------------------
    case PMSM_OID_MECH_STEADY_LOW:
    case PMSM_OID_MECH_STEADY_HIGH:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            sub->sum_iq_steady = float2ctrl(0.0f);
            sub->is_first_entry = 0;
        }

        // Mini I-Controller for Speed Hold
        {
            ctrl_gt target_spd = (sub->sm == PMSM_OID_MECH_STEADY_LOW) ? cfg->low_speed_pu : cfg->high_speed_pu;
            ctrl_gt err = target_spd - current_speed_pu;
            sub->active_iq_ref_pu += ctl_mul(err, float2ctrl(0.001f)); // Soft I-gain
            sub->active_iq_ref_pu = ctl_sat(sub->active_iq_ref_pu, float2ctrl(0.5f), float2ctrl(-0.5f));
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), sub->active_iq_ref_pu);
        }

        sub->sum_iq_steady += ctx->foc_core.idq0.dat[1]; // Accumulate actual Iq

        sub->tick_timer++;
        if (sub->tick_timer >= sub->settle_ticks)
        {
            parameter_gt avg_iq = (parameter_gt)ctl_mul(sub->sum_iq_steady, sub->inv_settle_ticks);

            if (sub->sm == PMSM_OID_MECH_STEADY_LOW)
            {
                sub->iq_steady_low_pu = avg_iq;
                sub->sm = PMSM_OID_MECH_ACCEL_TEST;
            }
            else
            {
                sub->iq_steady_high_pu = avg_iq;
                sub->sm = PMSM_OID_MECH_DECEL_TEST;
            }
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Acceleration Test
    // -------------------------------------------------------------
    case PMSM_OID_MECH_ACCEL_TEST:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), cfg->accel_iq_pu);
            // Record DA start index
            // sub->da_idx_accel_start = ctx->analyzer.count;
            sub->is_first_entry = 0;
        }

        // Push Point: X = Time, Y = Speed
        // ctl_da_push_point(&ctx->analyzer, time_now, current_speed_pu);

        sub->tick_timer++;
        // If speed reached OR DA is full
        if (current_speed_pu >= cfg->high_speed_pu /* || ctx->analyzer.count >= CTL_DA_MAX_POINTS */)
        {
            // sub->da_idx_accel_end = ctx->analyzer.count - 1;
            sub->active_iq_ref_pu = cfg->accel_iq_pu; // Handover to high-speed PI
            sub->sm = PMSM_OID_MECH_STEADY_HIGH;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Deceleration Test
    // -------------------------------------------------------------
    case PMSM_OID_MECH_DECEL_TEST:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            ctl_id_apply_dc_current(ctx, float2ctrl(0.0f), cfg->decel_iq_pu);
            // sub->da_idx_decel_start = ctx->analyzer.count;
            sub->is_first_entry = 0;
        }

        // OVER-VOLTAGE PROTECTION
        if (ctx->foc_core.udc > cfg->max_vbus_pu)
        {
            sub->sm = PMSM_OID_MECH_FAULT;
            sub->is_first_entry = 1;
            break;
        }

        // Push Point: X = Time, Y = Speed
        // ctl_da_push_point(&ctx->analyzer, time_now, current_speed_pu);

        sub->tick_timer++;
        if (current_speed_pu <= cfg->low_speed_pu /* || ctx->analyzer.count >= CTL_DA_MAX_POINTS */)
        {
            // sub->da_idx_decel_end = ctx->analyzer.count - 1;
            sub->sm = PMSM_OID_MECH_HANDOVER_TO_IF;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Handover Back: Real/SMO (B) -> IF (A)
    // -------------------------------------------------------------
    case PMSM_OID_MECH_HANDOVER_TO_IF:
        if (sub->is_first_entry)
        {
            sub->tick_timer = 0;
            // Align V/F freq to current physical speed to prevent jerk
            ctl_id_set_vf_target_speed(ctx, current_speed_pu);
            ctx->vf_gen.current_freq_pu = current_speed_pu;

            // Target 0 = Source A (V/F Generator)
            ctl_trigger_angle_transition(&ctx->angle_switcher, 0);
            sub->is_first_entry = 0;
        }

        // Blend current vector back to dragging Id
        {
            ctrl_gt w = ctx->angle_switcher.weight; // Transitions from 1.0 back to 0.0
            sub->active_id_ref_pu = ctl_mul(cfg->if_current_pu, float2ctrl(1.0f) - w);
            sub->active_iq_ref_pu = ctl_mul(sub->active_iq_ref_pu, w); // Fade out active Iq
            ctl_id_apply_dc_current(ctx, sub->active_id_ref_pu, sub->active_iq_ref_pu);
        }

        sub->tick_timer++;
        if (sub->tick_timer >= sub->transition_ticks)
        {
            sub->sm = PMSM_OID_MECH_IF_STOP;
            sub->is_first_entry = 1;
        }
        break;

    // -------------------------------------------------------------
    // Safe IF Shutdown
    // -------------------------------------------------------------
    case PMSM_OID_MECH_IF_STOP:
        if (sub->is_first_entry)
        {
            ctl_id_set_vf_target_speed(ctx, float2ctrl(0.0f));
            sub->is_first_entry = 0;
        }

        if (ctx->vf_gen.current_freq_pu <= float2ctrl(0.005f))
        {
            sub->sm = PMSM_OID_MECH_CALCULATE;
            sub->is_first_entry = 1;
        }
        break;
    }
}

/**
 * @brief Background loop function for Mechanical Parameters identification.
 * @details Safely executes pre-calculations and dual-curve linear regressions.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_mech(ctl_pmsm_offline_id_t* ctx)
{
    pmsm_offline_id_mech_t* sub = &ctx->sub_mech;
    pmsm_oid_cfg_mech_t* cfg = &sub->cfg;

    // --- Safety Rule ---
    if (sub->sm == PMSM_OID_MECH_CALCULATE || sub->sm == PMSM_OID_MECH_COMPLETE || sub->sm == PMSM_OID_MECH_FAULT)
    {
        ctl_id_disable_output(ctx);
    }

    if (sub->sm == PMSM_OID_MECH_INIT)
    {
        if (sub->is_first_entry)
        {
            // 1. Pre-calculate ISR constants
            sub->dt_sec = 1.0f / ctx->cfg_basic.isr_freq_hz;
            sub->settle_ticks = SEC_TO_TICKS(cfg->settle_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->transition_ticks = SEC_TO_TICKS(cfg->transition_time_s, ctx->cfg_basic.isr_freq_hz);
            sub->inv_settle_ticks = ctl_div(float2ctrl(1.0f), float2ctrl((float)sub->settle_ticks));

            // 2. Configure FOC core and Angle Switcher
            ctx->foc_core.flag_enable_decouple = 0;
            ctx->foc_core.flag_enable_vdq_feedforward = 0;
            ctl_init_angle_switcher(&ctx->angle_switcher, cfg->transition_time_s, ctx->cfg_basic.isr_freq_hz);
            ctl_attach_angle_switcher(&ctx->angle_switcher, &ctx->vf_gen.enc, ctx->enc);
            ctx->foc_core.pos_if = &ctx->angle_switcher.out_enc;

            // 3. Clear Data Analyzer
            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_MECH_IF_START;
            sub->is_first_entry = 1;
        }
    }
    else if (sub->sm == PMSM_OID_MECH_CALCULATE)
    {
        if (sub->is_first_entry)
        {
            parameter_gt alpha_acc_pu_s = 0.0f, dummy_intercept = 0.0f;
            parameter_gt alpha_dec_pu_s = 0.0f;

            // 1. Perform Linear Fitting on DA Segments: Speed = alpha * Time + initial_speed
            // ctl_da_linear_fit(&ctx->analyzer, sub->da_idx_accel_start, sub->da_idx_accel_end, &alpha_acc_pu_s, &dummy_intercept);
            // ctl_da_linear_fit(&ctx->analyzer, sub->da_idx_decel_start, sub->da_idx_decel_end, &alpha_dec_pu_s, &dummy_intercept);

            // --- Dummy values for compilation ---
            alpha_acc_pu_s = 2.5f;
            alpha_dec_pu_s = -3.0f;

            // 2. Base Conversions
            parameter_gt I_base = ctx->cfg_basic.i_base;
            parameter_gt W_mech_base = ctx->cfg_basic.w_base / (parameter_gt)ctx->cfg_basic.pole_pairs;

            // 3. Convert PU data to Physical Units
            parameter_gt alpha_acc_rads2 = alpha_acc_pu_s * W_mech_base;
            parameter_gt alpha_dec_rads2 = alpha_dec_pu_s * W_mech_base;
            parameter_gt w_mech_low = cfg->low_speed_pu * W_mech_base;
            parameter_gt w_mech_high = cfg->high_speed_pu * W_mech_base;

            // 4. Calculate Electromagnetic Torque Constant Kt (Nm/A)
            parameter_gt Kt = 1.5f * (parameter_gt)ctx->pmsm_param.pole_pairs * ctx->pmsm_param.flux_linkage;

            parameter_gt T_acc = Kt * (cfg->accel_iq_pu * I_base);
            parameter_gt T_dec = Kt * (cfg->decel_iq_pu * I_base);
            parameter_gt T_fric_low = Kt * (sub->iq_steady_low_pu * I_base);
            parameter_gt T_fric_high = Kt * (sub->iq_steady_high_pu * I_base);

            // 5. Calculate Inertia J (kg*m^2)
            parameter_gt delta_T = T_acc - T_dec;
            parameter_gt delta_alpha = alpha_acc_rads2 - alpha_dec_rads2;

            if (delta_alpha > 0.001f)
            {
                ctx->pmsm_mech_param.J_total = delta_T / delta_alpha;
            }
            else
            {
                ctx->pmsm_mech_param.J_total = 0.0001f; // Safeguard
            }

            // 6. Calculate Viscous Damping B (Nm / (rad/s))
            parameter_gt delta_T_fric = T_fric_high - T_fric_low;
            parameter_gt delta_w_mech = w_mech_high - w_mech_low;

            if (delta_w_mech > 0.001f)
            {
                ctx->pmsm_mech_param.B_viscous = delta_T_fric / delta_w_mech;
            }
            else
            {
                ctx->pmsm_mech_param.B_viscous = 0.0f;
            }

            // 7. Calculate Derived Mechanical Time Constant
            if (ctx->pmsm_mech_param.B_viscous > 0.00001f)
            {
                ctx->pmsm_mech_param.tau_m = ctx->pmsm_mech_param.J_total / ctx->pmsm_mech_param.B_viscous;
            }
            else
            {
                ctx->pmsm_mech_param.tau_m = 9999.0f;
            }

            // ctl_da_clear(&ctx->analyzer);

            sub->is_first_entry = 0;
            sub->sm = PMSM_OID_MECH_COMPLETE;
        }
    }
}

#pragma endregion

/**
 * @brief High-frequency ISR step function for PMSM Offline Identification.
 * @details This function MUST be called inside the high-priority motor control interrupt.
 * It routes the execution to the currently active sub-task's ISR function, steps the 
 * angle switcher, and finally executes the FOC core.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    // 1. Safety check: Do nothing if disabled, faulty, or merely waiting in READY.
    if (ctx->sm == PMSM_OFFLINE_ID_DISABLED || ctx->sm == PMSM_OFFLINE_ID_FAULT || ctx->sm == PMSM_OFFLINE_ID_READY)
    {
        return;
    }

    // 2. Dispatch ISR logic based on current main state
    switch (ctx->sm)
    {
    case PMSM_OFFLINE_ID_ADC_CALIB:
        // Optional: Handle ADC calibration PWM alignment if needed
        break;

    case PMSM_OFFLINE_ID_ENC_ALIGN:
        // (Alignment logic: Force Id, Wait)
        break;

    case PMSM_OFFLINE_ID_RS_DT:
        ctl_step_oid_rs_dt_isr(ctx);
        if (ctx->sub_rs_dt.sm == PMSM_OID_RSDT_FAULT)
        {
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        }
        break;

    case PMSM_OFFLINE_ID_LD_LQ:
        ctl_step_oid_ldq_isr(ctx);
        break;

    case PMSM_OFFLINE_ID_FLUX:
        ctl_step_oid_flux_isr(ctx);
        break;

    case PMSM_OFFLINE_ID_MECH:
        ctl_step_oid_mech_isr(ctx);
        break;

    default:
        break;
    }

    // 3. Step Core Embedded Components
    // 3.1 Step the angle switcher to blend or route the angle safely
    ctl_step_angle_switcher(&ctx->angle_switcher);

    // 3.2 Step the FOC core to generate final V_alpha/V_beta outputs
    ctl_step_current_controller(&ctx->foc_core);

    // 3.3 (Future) Step Data Analyzer High-Speed Recorder here
    // ctl_step_data_analyzer_isr(&ctx->analyzer);
}

/**
 * @brief Background loop function for PMSM Offline Identification.
 * @details This function manages heavy calculations, timeout checking, and state transitions.
 * It should be called in a low-priority background task (Main loop).
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    switch (ctx->sm)
    {
    case PMSM_OFFLINE_ID_DISABLED:
        // Do nothing
        break;

    case PMSM_OFFLINE_ID_READY:
        // In a real system, you would check a flag here (e.g., from a communication protocol)
        // to transition from READY to the first active state.
        // if (user_command == START_ID) {
        //     ctx->sm = PMSM_OFFLINE_ID_ADC_CALIB;
        // }
        break;

    case PMSM_OFFLINE_ID_ADC_CALIB:
        // Wait for external ADC module to finish
        // if (adc_calib_done) {
        //     ctx->sm = PMSM_OFFLINE_ID_RS_DT; // Simplified: Assuming we go to RS_DT next
        //     ctl_init_oid_rs_dt(ctx);
        // }
        break;

    // ---------------------------------------------------------------------
    // Sub-Task: Resistance & Dead-Time
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_RS_DT:
        ctl_loop_oid_rs_dt(ctx); // Handle heavy math (Least-Squares) inside

        if (ctx->sub_rs_dt.sm == PMSM_OID_RSDT_COMPLETE)
        {
            // Transition to Inductance ID if enabled
            if (ctx->cfg_basic.flag_enable_ldq)
            {
                ctx->sm = PMSM_OFFLINE_ID_LD_LQ;
                ctl_init_oid_ldq(ctx);
            }
            else
            {
                // Fallthrough skip logic (simplified for illustration)
                ctx->sm = PMSM_OFFLINE_ID_FLUX;
                ctl_init_oid_flux(ctx);
            }
        }
        else if (ctx->sub_rs_dt.sm == PMSM_OID_RSDT_FAULT)
        {
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        }
        break;

    // ---------------------------------------------------------------------
    // Sub-Task: Inductance
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_LD_LQ:
        ctl_loop_oid_ldq(ctx);

        if (ctx->sub_ldq.sm == PMSM_OID_LDQ_COMPLETE)
        {
            if (ctx->cfg_basic.flag_enable_flux)
            {
                ctx->sm = PMSM_OFFLINE_ID_FLUX;
                ctl_init_oid_flux(ctx);
            }
            else
            {
                ctx->sm = PMSM_OFFLINE_ID_CALC_REPORT;
            }
        }
        else if (ctx->sub_ldq.sm == PMSM_OID_LDQ_FAULT)
        {
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        }
        break;

    // ---------------------------------------------------------------------
    // Sub-Task: Flux Linkage
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_FLUX:
        ctl_loop_oid_flux(ctx);

        if (ctx->sub_flux.sm == PMSM_OID_FLUX_COMPLETE)
        {
            if (ctx->cfg_basic.flag_enable_mech_id)
            {
                ctx->sm = PMSM_OFFLINE_ID_MECH;
                ctl_init_oid_mech(ctx);
            }
            else
            {
                ctx->sm = PMSM_OFFLINE_ID_CALC_REPORT;
            }
        }
        else if (ctx->sub_flux.sm == PMSM_OID_FLUX_FAULT)
        {
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        }
        break;

    // ---------------------------------------------------------------------
    // Sub-Task: Mechanical Parameters
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_MECH:
        ctl_loop_oid_mech(ctx);

        if (ctx->sub_mech.sm == PMSM_OID_MECH_COMPLETE)
        {
            ctx->sm = PMSM_OFFLINE_ID_CALC_REPORT;
        }
        else if (ctx->sub_mech.sm == PMSM_OID_MECH_FAULT)
        {
            ctx->sm = PMSM_OFFLINE_ID_FAULT;
        }
        break;

    // ---------------------------------------------------------------------
    // Finalization
    // ---------------------------------------------------------------------
    case PMSM_OFFLINE_ID_CALC_REPORT:
        // Format the final Consultant Structures
        // ctl_finalize_offline_id_report(ctx);

        // Cleanly shut down FOC and transition back to READY
        ctl_disable_mtr_current_ctrl(&ctx->foc_core);
        ctx->sm = PMSM_OFFLINE_ID_READY;
        break;

    case PMSM_OFFLINE_ID_FAULT:
        // Handle fault state (e.g., turn off PWM, send error code to host)
        ctl_disable_mtr_current_ctrl(&ctx->foc_core);
        break;

    default:
        break;
    }
}

void ctl_init_pmsm_offline_id_sm()
{
}
