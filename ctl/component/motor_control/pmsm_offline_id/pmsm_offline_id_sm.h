
#ifndef _FILE_PMSM_OFFLINE_ID_SM_H_
#define _FILE_PMSM_OFFLINE_ID_SM_H_

/**
 * @brief State machine for PMSM offline parameter identification.
 * Follows a strict top-down sequence: Manual Verification -> Sensor Calibration -> Electrical -> Mechanical.
 */
typedef enum _tag_pmsm_offline_id_sm
{
    PMSM_OFFLINE_ID_DISABLED = 0, /*!< 0: Idle. PWM is OFF. Identification system is inactive. */

    // --- Phase 0: Manual Verification & Readiness ---
    PMSM_OFFLINE_ID_TEST_RUN, /*!< 1: Test Run Mode. User manually drives motor in I/F mode. 
                                           Purpose: Verify phase sequence, current sensor integrity, and encoder direction.
                                           Requires user intervention to exit this state. (Optional) */

    PMSM_OFFLINE_ID_READY, /*!< 2: Ready State. Acts as the staging ground for the automated sequence.
                                           Entered directly from DISABLED if TEST_RUN is skipped, or from 
                                           TEST_RUN upon user confirmation. Waits for "Start ID" command. */

    // --- Phase 1: Sensor Calibration (Static) ---
    PMSM_OFFLINE_ID_ADC_CALIB, /*!< 3: ADC Zero-Offset Calibration. 
                                           Handshakes with the existing external ADC calibration module.
                                           Waits until ADC offset is calculated and applied. */

    PMSM_OFFLINE_ID_ENC_ALIGN, /*!< 4: Absolute Encoder Zero-Position Alignment.
                                           Injects DC to align rotor to d-axis, records mechanical offset.
                                           Skipped if running in sensorless mode. */

    // --- Phase 2: Electrical Parameter ID (Static/Quasi-static) ---
    PMSM_OFFLINE_ID_RS_DT, /*!< 5: Stator Resistance (Rs) & Dead-time (DT) compensation.
                                           Multi-position DC injection with Data Analyzer fitting. */

    PMSM_OFFLINE_ID_LD_LQ, /*!< 6: Inductance (Ld, Lq) and saturation profiling.
                                           Voltage pulse / HFI sequence based on aligned d-axis. */

    // --- Phase 3: Electromechanical Parameter ID (Dynamic) ---
    PMSM_OFFLINE_ID_FLUX, /*!< 7: PM Flux Linkage (Psi_m).
                                           Spins motor to target speed using I/F, measures back-EMF. */

    PMSM_OFFLINE_ID_MECH, /*!< 8: Mechanical Parameters (J, B). (Optional) 
                                           Uses dual-curve (accel/decel) closed-loop Iq method to find 
                                           inertia and viscous damping simultaneously. */

    // --- Phase 4: Finalization ---
    PMSM_OFFLINE_ID_CALC_REPORT, /*!< 9: Finalize Data Analyzer buffers, calculate PU bases, update FOC core. */

    PMSM_OFFLINE_ID_FAULT /*!< 10: Fault. Triggered by external protection (OVP, OCP, LOS). */

} pmsm_offline_id_sm_t;

/**
 * @brief Sub-state machine for Stator Resistance (Rs) and Dead-Time (DT) identification.
 * Execution flow: INIT -> [Loop: 6 Angles -> [Loop: N Current Steps -> SETTLE -> MEASURE] ] -> CALC -> COMPLETE.
 */
typedef enum _tag_pmsm_offline_id_rs_dt_sm
{
    PMSM_OID_RSDT_DISABLED = 0, /*!< 0: Disabled/Bypass. Allows the main state machine to skip this step. */

    PMSM_OID_RSDT_INIT, /*!< 1: Initialize logic. Reset angle index (0~5) and current step index. 
                                          Set the first electrical angle vector (e.g., 0 degrees). */

    PMSM_OID_RSDT_ALIGN_SETTLE, /*!< 2: Apply the target DC current vector. 
                                          Wait for a defined timer (e.g., 500ms) to ensure the rotor is physically 
                                          attracted/locked to the position AND the L/R electrical transient has decayed. */

    PMSM_OID_RSDT_MEASURE, /*!< 3: Steady state. Trigger the Data Analyzer to start recording I_actual and U_ref. 
                                          Wait in this state until the Data Analyzer flags "Buffer_Full". */

    PMSM_OID_RSDT_STEP_EVALUATE, /*!< 4: Loop Controller. 
                                          - If current steps remain: Update current ref, go to ALIGN_SETTLE.
                                          - If angle steps remain: Reset current ref, update angle, go to ALIGN_SETTLE.
                                          - If all done (6 angles * N steps): Go to CALCULATE. */

    PMSM_OID_RSDT_CALCULATE, /*!< 5: Trigger Math library to perform Least-Squares fitting on the collected data.
                                          Extract Rs (slope) and V_dead (intercept) for all 6 positions.
                                          Compute variances and final average values. */

    PMSM_OID_RSDT_COMPLETE, /*!< 6: Update the ctl_consultant_pu_pmsm_t structure and write the "Medical Report".
                                          Signal the main state machine to proceed. */

    PMSM_OID_RSDT_FAULT /*!< 7: Exception handling. Entered if Data Analyzer times out, or current regulation fails. */

} pmsm_offline_id_rs_dt_sm_t;

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
 * @brief Sub-state machine for PM Flux Linkage (Psi_m) identification using I/F open-loop control.
 * Execution flow: INIT -> [Loop: N Speed Steps -> RAMP -> SETTLE -> MEASURE] -> RAMP_STOP -> CALC -> COMPLETE.
 */
typedef enum _tag_pmsm_offline_id_flux_sm
{
    PMSM_OID_FLUX_DISABLED = 0, /*!< 0: Disabled/Bypass. */

    PMSM_OID_FLUX_INIT, /*!< 1: Initialize sequence. 
                                            Set constant I/F current magnitude (e.g., 0.2pu I_base).
                                            Reset speed step index. Prepare the speed target array 
                                            (e.g., [0.15pu, 0.2pu, 0.25pu, 0.3pu, 0.35pu]).
                                            Clear Data Analyzer buffers. */

    PMSM_OID_FLUX_RAMP_SPEED, /*!< 2: Speed Trajectory Generator.
                                            Linearly ramp the I/F synchronous frequency (W_ref) to the 
                                            next target speed step in the array. 
                                            Wait until W_ref == Target_Speed[index]. */

    PMSM_OID_FLUX_SETTLE, /*!< 3: Mechanical Stabilization.
                                            Maintain W_ref. Wait for a timer (e.g., 1~2 seconds).
                                            In I/F mode, the rotor will oscillate around the synchronous 
                                            frame before locking into a steady load angle. This wait is crucial. */

    PMSM_OID_FLUX_MEASURE, /*!< 4: Data Collection.
                                            Trigger Data Analyzer to record:
                                            - U_alpha_ref, U_beta_ref (from duty cycles)
                                            - I_alpha, I_beta (actual sampled)
                                            - W_ref (the current synchronous speed step)
                                            Wait for "Buffer_Full" flag. */

    PMSM_OID_FLUX_STEP_EVALUATE, /*!< 5: Loop Controller.
                                            - If speed steps remain: increment index, go to RAMP_SPEED.
                                            - If all speeds measured: go to RAMP_STOP. */

    PMSM_OID_FLUX_RAMP_STOP, /*!< 6: Safe Shutdown.
                                            Linearly ramp W_ref down to 0.
                                            Monitor V_bus to prevent regenerative over-voltage.
                                            Once W_ref == 0, turn off PWM/current output. */

    PMSM_OID_FLUX_CALCULATE, /*!< 7: Mathematical Fitting.
                                            Calculate Back-EMF magnitude |E| for each speed step.
                                            Perform Least-Squares linear fit: |E| vs W_ref.
                                            The slope of the line is the Flux Linkage (Psi_m). */

    PMSM_OID_FLUX_COMPLETE, /*!< 8: Update ctl_consultant_pu_pmsm_t structure. Signal Main SM. */

    PMSM_OID_FLUX_FAULT /*!< 9: Exception handling. (Over-current, Loss of Sync, OVP). */

} pmsm_offline_id_flux_sm_t;

/**
 * @brief Sub-state machine for Mechanical Parameters (J, B) identification.
 * Execution flow: INIT -> IF_START -> HANDOVER -> STEADY_LOW -> ACCEL -> STEADY_HIGH -> DECEL -> CALC -> COMPLETE.
 */
typedef enum _tag_pmsm_offline_id_mech_sm
{
    PMSM_OID_MECH_DISABLED = 0,

    PMSM_OID_MECH_INIT, /*!< 1: Initialize. Set test thresholds (e.g., W_low = 0.2pu, W_high = 0.8pu).
                                        Set test currents (e.g., Iq_acc = 0.5pu, Iq_dec = -0.5pu). */

    // --- Stage 1: Spin up & Closed-loop Transition ---
    PMSM_OID_MECH_IF_START, /*!< 2: Open-loop start. Drive motor in I/F mode up to W_low (0.2pu). 
                                        Wait for SMO to converge (SMO back-EMF magnitude > threshold). */

    PMSM_OID_MECH_HANDOVER, /*!< 3: Bumpless Transfer. 
                                        Phase-lock the FOC coordinate system from I/F angle to SMO angle.
                                        Switch from I/F current vector control to Speed-Loop/Iq closed-loop control. */

    PMSM_OID_MECH_STEADY_LOW, /*!< 4: Stabilize at W_low (0.2pu) in CLOSED-LOOP.
                                        Measure average Iq to calculate low-speed friction torque. */

    // --- Stage 2: Acceleration Test ---
    PMSM_OID_MECH_ACCEL_TEST, /*!< 5: Inject constant Iq_acc (+). 
                                        Trigger Data Analyzer to record W_actual and time until W_actual >= W_high (0.8pu). */

    PMSM_OID_MECH_STEADY_HIGH, /*!< 6: Stabilize at W_high (0.8pu).
                                        Measure average Iq to calculate high-speed friction torque. */

    // --- Stage 3: Deceleration Test ---
    PMSM_OID_MECH_DECEL_TEST, /*!< 7: Inject constant Iq_dec (-). 
                                        Trigger Data Analyzer to record W_actual and time until W_actual <= W_low (0.2pu). 
                                        CRITICAL: Monitor V_bus to prevent over-voltage during regen braking. */

    // --- Stage 4: Calculation ---
    PMSM_OID_MECH_CALCULATE, /*!< 8: Trigger Math library.
                                        Calculate alpha_acc and alpha_dec using linear regression on Data Analyzer buffers.
                                        Calculate J using the dual-curve differential formula.
                                        Calculate B using (T_high - T_low) / (W_high - W_low) from the STEADY states. */

    PMSM_OID_MECH_COMPLETE, /*!< 9: Update ctl_consultant_pu_pmsm_t structure. Signal Main SM. */

    PMSM_OID_MECH_FAULT /*!< 10: Fault (SMO divergence, Over-voltage during decel, Timeout). */

} pmsm_offline_id_mech_sm_t;

#endif // _FILE_PMSM_OFFLINE_ID_SM_H_
