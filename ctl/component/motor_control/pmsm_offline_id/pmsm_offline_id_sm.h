
#ifndef _FILE_PMSM_OFFLINE_ID_SM_H_
#define _FILE_PMSM_OFFLINE_ID_SM_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

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
    PMSM_ID_RSDT_DISABLED = 0,

    /** * @brief 1: Initialization.
     * @details Triggers the background loop to pre-calculate tick delays and current arrays.
     * @note Debug: Resets `angle_idx` = 0, `step_idx` = 0, `tick_timer` = 0. 
     */
    PMSM_ID_RSDT_INIT,

    /** * @brief 2: Mechanical & Electrical Alignment.
     * @details Forces the FOC static angle to `angle_pu_array[angle_idx]` and injects maximum 
     * current to pull the rotor to the target position.
     * @note Debug: `tick_timer` increments until it reaches `align_ticks` (derived from `cfg.align_time_s`). 
     */
    PMSM_ID_RSDT_ALIGN_SETTLE,

    /** * @brief 3: Step Current Transient Delay.
     * @details Applies the specific target current `current_step_array[step_idx]` and waits for 
     * the L/R inductive transient to fully decay into a steady DC state.
     * @note Debug: `tick_timer` increments until it reaches `measure_delay_ticks` (derived from `cfg.measure_delay_s`). 
     */
    PMSM_ID_RSDT_STEP_DELAY,

    /** * @brief 4: Data Measurement & Accumulation.
     * @details Integrates voltage commands (`vdq_ref`) and actual currents (`idq0`) over a fixed number of ISR cycles.
     * @note Debug: `tick_timer` increments until it reaches `cfg.measure_points`. 
     * Variables `sum_u` and `sum_i` will actively accumulate during this state.
     */
    PMSM_ID_RSDT_MEASURE,

    /** * @brief 5: Loop Evaluation & Routing.
     * @details A 0-tick transient state. Evaluates if all steps for the current angle are done, 
     * and if all 6 angles are completed.
     * @note Debug: Watch `step_idx` and `angle_idx` increment here. Routes back to `STEP_DELAY`, 
     * `ALIGN_SETTLE`, or moves forward to `CALCULATE`.
     */
    PMSM_ID_RSDT_STEP_EVALUATE,

    /** * @brief 6: Least-Squares Calculation.
     * @details Hands over execution to the background loop to perform linear regression on the 
     * accumulated Data Analyzer points. Output is safely clamped to 0V/0A.
     * @note Debug: Watch for `rs_mean` and `vcomp_mean` to be populated in the main context.
     */
    PMSM_ID_RSDT_CALCULATE,

    /** * @brief 7: Identification Complete.
     * @details Signals the master state machine that Rs and DT parameters have been successfully 
     * updated in the global consultant structure. Output is disabled.
     */
    PMSM_ID_RSDT_COMPLETE,

    /** * @brief 8: Fault / Exception.
     * @details Entered upon over-current, timeout, or mathematical singularities during fitting.
     * Output is immediately disabled.
     */
    PMSM_ID_RSDT_FAULT

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
    ctrl_gt angle_pu_array[6];    /*!< NEW: Pre-calculated electrical angles to avoid float math in ISR. */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;       /*!< Internal timer for delays and settling. */
    uint16_t angle_idx;        /*!< Current electrical angle index (0 to 5). */
    ctrl_gt angle_pu;          /*!< Current active electrical angle. */
    uint16_t step_idx;         /*!< Current injected current step index. */
    ctrl_gt current_ref_pu;    /*!< The active DC current reference being applied. */
    fast_gt is_first_entry;    /*!< Flag to distinguish between step (first entry) and hold logic. */
    //ctrl_gt current_increment; /*!< Current increment for each step*/

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
    PMSM_ID_LDQ_DISABLED = 0, /*!< 0: Disabled/Bypass. Allows the main SM to skip this step. */

    PMSM_ID_LDQ_INIT, /*!< 1: Initialize logic. Set target axis to D-axis (theta = alignment_offset).
                                            Reset step counters and clear Data Analyzer buffers. */

    PMSM_ID_LDQ_BIAS_SETTLE, /*!< 2: Apply DC Bias Current (Id_bias or Iq_bias) using PI controllers.
                                            Wait for the L/R transient to settle. 
                                            If testing unsaturated L (bias=0A), just wait for I=0. */

    PMSM_ID_LDQ_PULSE_MEASURE, /*!< 3: Open-loop Voltage Pulse Injection.
                                            Suspend PI controllers. Inject a fixed voltage vector (e.g., U_test) 
                                            on the active axis for a VERY SHORT duration (e.g., 500us ~ 2ms).
                                            Trigger Data Analyzer to record high-speed current slope (di/dt). */

    PMSM_ID_LDQ_COOLDOWN, /*!< 4: Flux/Energy Reset.
                                            Apply 0V (zero vector) or re-enable PI to drive current back to the 
                                            bias level or 0A. Wait until current is fully discharged to prevent 
                                            current runaway on the next pulse. */

    PMSM_ID_LDQ_STEP_EVALUATE, /*!< 5: Loop Controller.
                                            - If bias steps remain: Update bias current, go to BIAS_SETTLE.
                                            - If axis is D and D is done: Switch to Q-axis (theta += 90 deg), 
                                              reset steps, go to BIAS_SETTLE.
                                            - If both D and Q are mapped: Go to CALCULATE. */

    PMSM_ID_LDQ_CALCULATE, /*!< 6: Trigger Math library. 
                                            Calculate L = (U_ref - Rs*I - V_comp) / (di/dt) for each point.
                                            Fit the L-I saturation polynomial curves for D and Q axes. */

    PMSM_ID_LDQ_COMPLETE, /*!< 7: Update the ctl_consultant_pu_pmsm_t structure.
                                            Signal the main state machine to proceed. */

    PMSM_ID_LDQ_FAULT /*!< 8: Exception handling. Triggered by over-current during pulse, 
                                            unintended rotor movement (if encoder detects delta_theta > threshold), 
                                            or DA timeout. */

} pmsm_offline_id_ld_lq_sm_t;

/**
 * @brief Configuration for Inductance (Ld, Lq) Identification.
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

    // --- DSA Slicing Indices ---
    uint32_t d_start_idx[16]; /*!< DA start index for D-axis pulses. */
    uint32_t d_end_idx[16];   /*!< DA end index for D-axis pulses. */
    uint32_t q_start_idx[16]; /*!< DA start index for Q-axis pulses. */
    uint32_t q_end_idx[16];   /*!< DA end index for Q-axis pulses. */

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
    PMSM_ID_FLUX_DISABLED = 0,

    /** @brief 1: Initialization. 
     * @details Pre-calculates step sizes and timing ticks in the background loop. 
     * @note Debug: Resets `step_idx` = 0, `is_first_entry` = 1. 
     */
    PMSM_ID_FLUX_INIT,

    /** @brief 2: Speed Trajectory Generator.
     * @details Sets the target speed for the V/F generator and waits for the ramp to finish.
     * @note Debug: Watch `ctx->vf_gen.current_freq_pu` approach `target_w_pu`.
     */
    PMSM_ID_FLUX_RAMP_SPEED,

    /** @brief 3: Mechanical Stabilization.
     * @details Waits for the "spring-pendulum" oscillation of the rotor to dampen.
     * @note Debug: `tick_timer` increments until `settle_ticks`.
     */
    PMSM_ID_FLUX_SETTLE,

    /** @brief 4: Data Collection.
     * @details Accumulates Ud, Uq, Id, Iq, and W_ref over a fixed number of ISR ticks.
     * @note Debug: `tick_timer` increments until `cfg.measure_points`.
     */
    PMSM_ID_FLUX_MEASURE,

    /** @brief 5: Loop Controller.
     * @details Evaluates if all speed steps are completed to route to STOP or next SPEED.
     */
    PMSM_ID_FLUX_STEP_EVALUATE,

    /** @brief 6: Safe Shutdown.
     * @details Ramps the speed down to 0 smoothly to avoid regenerative over-voltage.
     */
    PMSM_ID_FLUX_RAMP_STOP,

    /** @brief 7: Mathematical Fitting.
     * @details Hands over to the background loop. Calculates |E| and uses Data Analyzer to find Psi_m.
     */
    PMSM_ID_FLUX_CALCULATE,

    /** @brief 8: Update ctl_consultant_pu_pmsm_t structure. Signal Main SM. */
    PMSM_ID_FLUX_COMPLETE,

    /** @brief 9: Exception handling. (Over-current, Loss of Sync, OVP). */
    PMSM_ID_FLUX_FAULT

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

    // --- Pre-calculated Context (Computed in Loop to save ISR time) ---
    uint32_t settle_ticks;      /*!< ISR ticks corresponding to settle_time_s. */
    ctrl_gt step_size_pu;       /*!< Speed increment per step. */
    ctrl_gt inv_measure_points; /*!< 1.0f / measure_points, to avoid division in ISR. */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;    /*!< Internal timer for steady-state measuring. */
    uint16_t step_idx;      /*!< Current speed step index. */
    fast_gt is_first_entry; /*!< Flag to distinguish between step and hold logic. */
    ctrl_gt target_w_pu;    /*!< Target speed for the current ramp. */

    // --- Measurement Accumulators (ISR) ---
    ctrl_gt sum_ud; /*!< D-axis voltage accumulator. */
    ctrl_gt sum_uq; /*!< Q-axis voltage accumulator. */
    ctrl_gt sum_id; /*!< D-axis current accumulator. */
    ctrl_gt sum_iq; /*!< Q-axis current accumulator. */
    ctrl_gt sum_w;  /*!< Speed accumulator. */

} pmsm_offline_id_flux_t;

// ============================================================================

/**
 * @brief State enumeration for the Mechanical Parameters (J, B) sub-state machine.
 * @details Execution flow:
 * INIT -> IF_START -> HANDOVER_TO_CLOSED -> STEADY_LOW -> ACCEL_TEST -> STEADY_HIGH -> DECEL_TEST -> HANDOVER_TO_IF -> IF_STOP -> CALCULATE -> COMPLETE
 */
typedef enum _tag_pmsm_offline_id_mech_sm
{
    PMSM_ID_MECH_DISABLED = 0, /*!< 0: Disabled/Bypass. Safe state. */

    PMSM_ID_MECH_INIT, /*!< 1: Initialize. Pre-calculate ticks and limits in the background loop. */

    // --- Stage 1: Spin up & Closed-loop Transition ---
    PMSM_ID_MECH_IF_START,           /*!< 2: Open-loop start. Drive motor in I/F mode up to W_low. */
    PMSM_ID_MECH_HANDOVER_TO_CLOSED, /*!< 3: Bumpless Transfer to Closed-Loop. Transition angle from VF to Real/SMO. */
    PMSM_ID_MECH_STEADY_LOW,         /*!< 4: Stabilize at W_low. Accumulate Iq to calculate low-speed friction. */

    // --- Stage 2: Acceleration Test ---
    PMSM_ID_MECH_ACCEL_TEST,  /*!< 5: Inject constant +Iq. Record (Time, Speed) into Data Analyzer until W >= W_high. */
    PMSM_ID_MECH_STEADY_HIGH, /*!< 6: Stabilize at W_high. Accumulate Iq to calculate high-speed friction. */

    // --- Stage 3: Deceleration Test & Safe Shutdown ---
    PMSM_ID_MECH_DECEL_TEST, /*!< 7: Inject constant -Iq. Record (Time, Speed) into DA until W <= W_low. Monitors OVP. */
    PMSM_ID_MECH_HANDOVER_TO_IF, /*!< 8: Transition back to I/F mode to ensure safe shutdown as SMO fails at low speed. */
    PMSM_ID_MECH_IF_STOP,        /*!< 9: Ramp V/F target to 0 and gracefully stop the motor. */

    // --- Stage 4: Calculation ---
    PMSM_ID_MECH_CALCULATE, /*!< 10: Trigger Math library. Fit J and B from DA buffers in the background loop. */
    PMSM_ID_MECH_COMPLETE,  /*!< 11: Update ctl_consultant_mech1_t structure. Signal Main SM. */
    PMSM_ID_MECH_FAULT      /*!< 12: Fault (OVP, Timeout, DA overflow). */

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

    // --- Pre-calculated Context (Computed in Loop to save ISR time) ---
    uint32_t settle_ticks;     /*!< ISR ticks corresponding to settle_time_s. */
    uint32_t transition_ticks; /*!< ISR ticks corresponding to transition_time_s. */
    ctrl_gt inv_settle_ticks;  /*!< 1.0f / settle_ticks for fast averaging. */
    parameter_gt dt_sec;       /*!< Time step per ISR tick (1.0 / isr_freq_hz). */

    // --- Runtime Context (ISR) ---
    uint32_t tick_timer;      /*!< Internal timer for handover and settling. */
    fast_gt is_first_entry;   /*!< Flag to distinguish between step (first entry) and hold logic. */
    ctrl_gt active_iq_ref_pu; /*!< The active torque current applied during accel/decel. */
    ctrl_gt active_id_ref_pu; /*!< The active dragging current applied. */

    // --- Measurement Accumulators (ISR) ---
    ctrl_gt sum_iq_steady;          /*!< Accumulator for steady-state friction current. */
    parameter_gt iq_steady_low_pu;  /*!< Resulting average Iq at low speed. */
    parameter_gt iq_steady_high_pu; /*!< Resulting average Iq at high speed. */

    // --- DSA Slicing Indices ---
    uint32_t da_idx_accel_start; /*!< DA start index for acceleration phase. */
    uint32_t da_idx_accel_end;   /*!< DA end index for acceleration phase. */
    uint32_t da_idx_decel_start; /*!< DA start index for deceleration phase. */
    uint32_t da_idx_decel_end;   /*!< DA end index for deceleration phase. */

} pmsm_offline_id_mech_t;

//================================================================================
// 2. Identification Configuration Structures
//================================================================================

/**
 * @brief State machine for PMSM offline parameter identification.
 * Follows a strict top-down sequence: Manual Verification -> Prepare -> Electrical -> Mechanical -> Complete.
 */
typedef enum _tag_pmsm_offline_id_sm
{
    PMSM_OFFLINE_ID_DISABLED = 0, /*!< 0: Idle. PWM is OFF. Identification system is inactive. */

    // --- Phase 0: Manual Verification & Readiness ---
    PMSM_OFFLINE_ID_TEST_RUN, /*!< 1: Test Run Mode. User manually drives motor in I/F mode. 
                                           Purpose: Verify phase sequence, current sensor integrity, etc. */

    PMSM_OFFLINE_ID_READY, /*!< 2: Ready State. Acts as the staging ground for the automated sequence.
                                           Waits for the master "Start ID" command. */

    // --- Phase 1: Sensor & Hardware Preparation ---
    PMSM_OFFLINE_ID_PREPARE, /*!< 3: Hardware Preparation Phase (Merged ADC & Encoder Calib).
                                           User executes ADC zero-offset calibration and/or Encoder alignment here.
                                           Waits until user software signals completion. */

    // --- Phase 2: Electrical Parameter ID (Static/Quasi-static) ---
    PMSM_OFFLINE_ID_RS_DT, /*!< 4: Stator Resistance (Rs) & Dead-time (DT) compensation. */

    PMSM_OFFLINE_ID_LD_LQ, /*!< 5: Inductance (Ld, Lq) and saturation profiling. */

    // --- Phase 3: Electromechanical Parameter ID (Dynamic) ---
    PMSM_OFFLINE_ID_FLUX, /*!< 6: PM Flux Linkage (Psi_m). */

    PMSM_OFFLINE_ID_MECH, /*!< 7: Mechanical Parameters (J, B). (Optional) */

    // --- Phase 4: Finalization ---
    PMSM_OFFLINE_ID_COMPLETE, /*!< 8: Identification Complete. Motor is safely powered off.
                                           System holds in this state for the user to extract data/results. */

    PMSM_OFFLINE_ID_FAULT /*!< 9: Fault. Triggered by external protection or math singularity. */

} pmsm_offline_id_sm_t;

/**
 * @brief Basic configuration and feature flags for the identification process.
 */
typedef struct _tag_pmsm_oid_cfg_basic
{
    parameter_gt isr_freq_hz; /*!< Execution frequency of the control loop (Hz). */
    parameter_gt pole_pairs;  /*!< Motor pole pairs. */

    // --- Feature Options (1 to enable, 0 to skip) ---
    fast_gt flag_enable_prepare; /*!< Enables the PREPARE stage for custom calibration. */
    fast_gt flag_enable_rs_dt;   /*!< Enables Rs & DT identification. */
    fast_gt flag_enable_ldq;     /*!< Enables Ld & Lq identification. */
    fast_gt flag_enable_flux;    /*!< Enables Flux Linkage identification. */
    fast_gt flag_enable_mech_id; /*!< Enables Mechanical (J, B) identification. */

    fast_gt is_sensorless; /*!< Flag: 1 if no physical encoder is present. */

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

    // --- FOC Core Initial Parameters ---
    parameter_gt kp;        /*!< Proportional gain for FOC PI controllers. */
    parameter_gt ki;        /*!< Integral gain for FOC PI controllers. */
    parameter_gt max_vs_pu; /*!< Maximum output voltage magnitude (PU). */

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
    //mtr_current_ctrl_t foc_core;         /*!< Dedicated FOC current controller core. */
    ctl_slope_f_pu_controller vf_gen;    /*!< V/F slope frequency generator for I/F mode. */
    ctl_angle_switcher_t angle_switcher; /*!< Smooth transition router for angles. */
    //ctl_pmsm_esmo_t esmo;                /*!< Extended Sliding Mode Observer. */
    ctl_dsa_scope_t analyzer; /*!< Data recording and fitting engine (WIP). */
    //ctl_mtr_protect_t protect;           /*!< Protection Module*/

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
// --- Resistance & Dead-Time (RS_DT) ---
//

/**
 * @brief Initializes the Rs & DT identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_rs_dt(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief ISR step function for Rs & DT identification.
 * @details Executes step-current injection. Leverages DSA Scope to safely log U-I data.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_rs_dt_isr(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief Background loop function for Rs & DT identification.
 * @details Executes setup configurations and invokes the DSA Scope for linear regression.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_rs_dt(ctl_pmsm_offline_id_t* ctx);

//
// --- Inductance (LD_LQ) ---
//

/**
 * @brief Initializes the Inductance (Ld, Lq) identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_ldq(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief ISR step function for Ld & Lq identification.
 * @details Executes DC bias settling, PI freezing, pulse injection, and DA recording.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_ldq_isr(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief Background loop function for Ld & Lq identification.
 * @details Computes pre-requisites and performs the di/dt linear regression using the DA.
 * MUST be called from a low-priority task or main loop.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_ldq(ctl_pmsm_offline_id_t* ctx);

//
// --- Flux Linkage (FLUX) ---
//

/**
 * @brief Initializes the Flux Linkage identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_flux(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief ISR step function for Flux Linkage identification.
 * @details Executes V/F ramping, I/F dragging, and pushes averaged data to the DSA Scope.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_flux_isr(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief Background loop function for Flux Linkage identification.
 * @details Computes pre-requisites and performs the linear regression (|E| vs W) using the DA.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_flux(ctl_pmsm_offline_id_t* ctx);

//
// --- Mechanical Parameters (MECH) ---
//

/**
 * @brief Initializes the Mechanical Parameters identification sub-task.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_init_oid_mech(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief ISR step function for Mechanical Parameters identification.
 * @details Highly optimized state machine managing V/F start, closed-loop handover,
 * localized speed control, and dual-curve high-speed recording via DSA Scope.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_oid_mech_isr(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief Background loop function for Mechanical Parameters identification.
 * @details Safely executes pre-calculations and dual-curve linear regressions.
 * MUST be called from a low-priority task or main loop.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_oid_mech(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief High-frequency ISR step function for PMSM Offline Identification.
 * @details Routes execution to the active sub-task's ISR, steps angle switcher, and executes FOC.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_step_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief Background loop function for PMSM Offline Identification.
 * @details Manages heavy calculations, timeout checking, and state transitions using the Move Next router.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
void ctl_loop_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx);

/**
 * @brief Initializes the complete PMSM Offline Identification Master State Machine.
 * @param[out] ctx          Pointer to the master offline ID context.
 * @param[in]  init_cfg     Pointer to the configuration "checkup form".
 * @param[in]  dsa_buffer   Pointer to the memory pool for the DSA Scope.
 * @param[in]  dsa_capacity Total capacity of the DSA Scope memory buffer.
 */
void ctl_init_pmsm_offline_id_sm(ctl_pmsm_offline_id_t* ctx, const ctl_pmsm_offline_id_init_t* init_cfg,
                                 ctrl_gt* dsa_buffer, uint32_t dsa_capacity);

//
// Service function
//

/**
 * @brief Angle source routing options for the offline identification context.
 */
typedef enum _tag_pmsm_oid_angle_src
{
    PMSM_ID_ANGLE_SRC_STATIC = 0, /*!< Route FOC to the internal static dummy angle. */
    PMSM_ID_ANGLE_SRC_VF_GEN,     /*!< Route FOC to the V/F slope generator. */
    PMSM_ID_ANGLE_SRC_REAL_ENC    /*!< Route FOC to the real physical encoder or SMO. */
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
    case PMSM_ID_ANGLE_SRC_STATIC:
        mtr_ctrl.pos_if = &ctx->static_angle;
        break;
    case PMSM_ID_ANGLE_SRC_VF_GEN:
        mtr_ctrl.pos_if = &ctx->vf_gen.enc;
        break;
    case PMSM_ID_ANGLE_SRC_REAL_ENC:
        mtr_ctrl.pos_if = ctx->enc;
        break;
    }
}

typedef enum _tag_pmsm_id_foc_state
{
    PMSM_ID_VOLTAGE_OPENLOOP = 0,
    PMSM_ID_CURRENT_CLOSELOOP = 1
} pmsm_id_foc_state_e;

GMP_STATIC_INLINE void ctl_id_set_foc_state(ctl_pmsm_offline_id_t* ctx, pmsm_id_foc_state_e state)
{
    switch (src)
    {
    case PMSM_ID_VOLTAGE_OPENLOOP:
        ctl_disable_foc_core_current_ctrl(&mtr_ctrl);
        ctl_disable_foc_core_decouple(&mtr_ctrl);
        ctl_disable_foc_core_vdq_ff(&mtr_ctrl);
        break;
    case PMSM_ID_CURRENT_CLOSELOOP:
        ctl_enable_mtr_current_ctrl(&mtr_ctrl);
        ctl_disable_foc_core_decouple(&mtr_ctrl);
        ctl_disable_foc_core_vdq_ff(&mtr_ctrl);
        break;
    default:
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
    ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_STATIC);
    ctx->static_angle.elec_position = angle_pu;
}

/**
 * @brief Safely shuts down the FOC output (Zero current/voltage injection).
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_id_disable_output(ctl_pmsm_offline_id_t* ctx)
{
    ctl_disable_mtr_current_ctrl(&mtr_ctrl);
    ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, float2ctrl(0.0f), float2ctrl(0.0f));
    ctl_set_mtr_current_ctrl_vdq_ref(&mtr_ctrl, float2ctrl(0.0f), float2ctrl(0.0f));
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
    ctl_enable_mtr_current_ctrl(&mtr_ctrl);
    ctl_set_mtr_current_ctrl_ref(&mtr_ctrl, id_pu, iq_pu);
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
    ctl_disable_mtr_current_ctrl(&mtr_ctrl); // Disable PI regulation
    ctl_set_mtr_current_ctrl_vdq_ref(&mtr_ctrl, vd_pu, vq_pu);
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
    ctl_id_route_foc_angle(ctx, PMSM_ID_ANGLE_SRC_VF_GEN);
    ctl_step_slope_f_pu(&ctx->vf_gen);
}

/**
 * @brief Helper function to navigate to the next enabled state based on configuration.
 * @param[in] ctx Pointer to the master offline ID context.
 * @param[in] current_state The state that just finished executing.
 * @return pmsm_offline_id_sm_t The next state to transition to.
 */
GMP_STATIC_INLINE pmsm_offline_id_sm_t ctl_oid_get_next_state(ctl_pmsm_offline_id_t* ctx,
                                                              pmsm_offline_id_sm_t current_state)
{
    // The fallthrough design perfectly matches sequential identification steps.
    switch (current_state)
    {
    case PMSM_OFFLINE_ID_READY:
        if (ctx->cfg_basic.flag_enable_prepare)
            return PMSM_OFFLINE_ID_PREPARE;
        // fallthrough
    case PMSM_OFFLINE_ID_PREPARE:
        if (ctx->cfg_basic.flag_enable_rs_dt)
            return PMSM_OFFLINE_ID_RS_DT;
        // fallthrough
    case PMSM_OFFLINE_ID_RS_DT:
        if (ctx->cfg_basic.flag_enable_ldq)
            return PMSM_OFFLINE_ID_LD_LQ;
        // fallthrough
    case PMSM_OFFLINE_ID_LD_LQ:
        if (ctx->cfg_basic.flag_enable_flux)
            return PMSM_OFFLINE_ID_FLUX;
        // fallthrough
    case PMSM_OFFLINE_ID_FLUX:
        if (ctx->cfg_basic.flag_enable_mech_id)
            return PMSM_OFFLINE_ID_MECH;
        // fallthrough
    case PMSM_OFFLINE_ID_MECH:
        return PMSM_OFFLINE_ID_COMPLETE; // End of the road

    default:
        return PMSM_OFFLINE_ID_COMPLETE;
    }
}

/**
 * @brief Helper function to initialize the target state.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
static void ctl_oid_init_target_state(ctl_pmsm_offline_id_t* ctx)
{
    switch (ctx->sm)
    {
    case PMSM_OFFLINE_ID_PREPARE:
        // Placeholder: Call user's external prepare init if needed
        // ctl_init_oid_prepare(ctx);
        break;
    case PMSM_OFFLINE_ID_RS_DT:
        ctl_init_oid_rs_dt(ctx);
        break;
    case PMSM_OFFLINE_ID_LD_LQ:
        ctl_init_oid_ldq(ctx);
        break;
    case PMSM_OFFLINE_ID_FLUX:
        ctl_init_oid_flux(ctx);
        break;
    case PMSM_OFFLINE_ID_MECH:
        ctl_init_oid_mech(ctx);
        break;
    default:
        break;
    }
}

/**
 * @brief Enables the Offline Identification process.
 * @details Commands the state machine to transition from READY to the first 
 * active test stage (PREPARE or otherwise). If the system is not in READY, this is ignored.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_enable_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    if (ctx->sm == PMSM_OFFLINE_ID_READY)
    {
        // Leverage the Move Next router to automatically jump to the first enabled step
        ctx->sm = ctl_oid_get_next_state(ctx, PMSM_OFFLINE_ID_READY);
        ctl_oid_init_target_state(ctx);
    }
}

/**
 * @brief Safely disables the Offline Identification process.
 * @details Immediately turns off the FOC PWM outputs and forces the master state 
 * machine into the DISABLED state. Can be used as a soft E-Stop.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_disable_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    ctl_disable_mtr_current_ctrl(&mtr_ctrl);
    ctx->sm = PMSM_OFFLINE_ID_DISABLED;
}

/**
 * @brief Clears the operational state and faults of the Offline Identification module.
 * @details Safely stops the motor, wipes the data analyzer memory, resets protection faults,
 * clears FOC internal integrals, and puts the state machine back into the READY state.
 * @param[in,out] ctx Pointer to the master offline ID context.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_offline_id(ctl_pmsm_offline_id_t* ctx)
{
    // 1. Safe Hardware Shutdown
    //ctl_disable_mtr_current_ctrl(&mtr_ctrl);
    //ctl_clear_mtr_current_ctrl(&mtr_ctrl);

    // 2. Reset Core Components
    //ctl_clear_mtr_protect(&ctx->protect);
    ctl_wipe_dsa_scope_memory(&ctx->analyzer);
    ctl_clear_slope_f_pu(&ctx->vf_gen);

    // 3. Reset Sub-state machines
    ctx->sub_rs_dt.sm = PMSM_ID_RSDT_DISABLED;
    ctx->sub_ldq.sm = PMSM_ID_LDQ_DISABLED;
    ctx->sub_flux.sm = PMSM_ID_FLUX_DISABLED;
    ctx->sub_mech.sm = PMSM_ID_MECH_DISABLED;

    // 4. Return to Staging Ground
    ctx->sm = PMSM_OFFLINE_ID_READY;
}

// 将物理时间(秒)转换为 ISR 滴答数
#define SEC_TO_TICKS(sec, freq) ((uint32_t)((sec) * (freq)))

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_OFFLINE_ID_SM_H_
