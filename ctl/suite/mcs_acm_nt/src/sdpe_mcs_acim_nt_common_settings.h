/**
 * @file sdpe_mcs_acim_nt_common_settings.h
 * @brief SDPE project bindings for MCS ACIM NT Common Controller Settings.
 * @note Platform-independent AC induction-motor FOC settings.
 */

#ifndef _PROJECT_SDPE_MCS_ACIM_NT_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_MCS_ACIM_NT_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#define MCS_ACIM_FEEDBACK_SENSORED (1)
#define MCS_ACIM_FEEDBACK_SENSORLESS (2)
#define MCS_FO_VOLTAGE_FROM_COMMAND (1)
#define MCS_FO_VOLTAGE_FROM_MEASUREMENT (2)

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_ACIM_NT_COMMON_SDPE_PROJECT_ID "mcs_acim_nt_common"
#define MCS_ACIM_NT_COMMON_SDPE_PROJECT_SUITE "mcs_acim_nt"
#define MCS_ACIM_NT_COMMON_SDPE_PROJECT_VERSION "0.1.0"
#define MCS_ACIM_NT_COMMON_SDPE_PROJECT_UPDATED_AT "2026-08-09"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Apply ACIM-specific cross-coupling through the generic FOC feedforward port.
 */
#define ENABLE_ACIM_DECOUPLING

/**
 * @brief Enable current-polarity PWM compare compensation for physical inverter dead time. Keep disabled for an ideal SIL bridge; identify the required count and current polarity on hardware.
 */
// #ifndef ENABLE_PWM_DEADTIME_COMPENSATION
// #define ENABLE_PWM_DEADTIME_COMPENSATION
// #endif // ENABLE_PWM_DEADTIME_COMPENSATION

//=================================================================================================
/**
 * @brief Protection.
 */

/**
 * @brief Enable motor fault protection processing.
 */
// #define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning gate: 1 open-angle V/f and PWM/ADC polarity; 2 I/F dq-current loop with synthetic field angle; 3 real field-angle current loop using external ACIM_POS_CALC or ACIM_FO; 4 mechanical speed loop. Do not advance until the previous level passes the commissioning record.
 *        Options: (1), (2), (3), (4)
 */
#ifndef BUILD_LEVEL
#define BUILD_LEVEL (1)
#endif // BUILD_LEVEL

/**
 * @brief Select speed-feedback IFOC or the voltage/current flux observer.
 *        Options: MCS_ACIM_FEEDBACK_SENSORED, MCS_ACIM_FEEDBACK_SENSORLESS
 */
#ifndef MCS_ACIM_FEEDBACK_MODE
#define MCS_ACIM_FEEDBACK_MODE MCS_ACIM_FEEDBACK_SENSORED
#endif // MCS_ACIM_FEEDBACK_MODE

/**
 * @brief Select pre-dead-time controller command reconstruction or sampled phase voltage for the ACIM voltage model.
 *        Options: MCS_FO_VOLTAGE_FROM_COMMAND, MCS_FO_VOLTAGE_FROM_MEASUREMENT
 */
#ifndef MCS_FO_VOLTAGE_SOURCE
#define MCS_FO_VOLTAGE_SOURCE MCS_FO_VOLTAGE_FROM_COMMAND
#endif // MCS_FO_VOLTAGE_SOURCE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Fast-loop frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Motor Rs
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS (0.329f)
#endif // MOTOR_PARAM_RS

/**
 * @brief Motor Rr
 */
#ifndef MOTOR_PARAM_RR
#define MOTOR_PARAM_RR (0.44f)
#endif // MOTOR_PARAM_RR

/**
 * @brief Motor L1s
 */
#ifndef MOTOR_PARAM_L1S
#define MOTOR_PARAM_L1S (0.0003f)
#endif // MOTOR_PARAM_L1S

/**
 * @brief Motor L1r
 */
#ifndef MOTOR_PARAM_L1R
#define MOTOR_PARAM_L1R (0.0005f)
#endif // MOTOR_PARAM_L1R

/**
 * @brief Motor Lm
 */
#ifndef MOTOR_PARAM_LM
#define MOTOR_PARAM_LM (0.0012f)
#endif // MOTOR_PARAM_LM

/**
 * @brief Motor Pole Pairs
 */
#ifndef MOTOR_PARAM_POLE_PAIRS
#define MOTOR_PARAM_POLE_PAIRS (2)
#endif // MOTOR_PARAM_POLE_PAIRS

/**
 * @brief Motor Rated Frequency
 */
#ifndef MOTOR_PARAM_RATED_FREQUENCY
#define MOTOR_PARAM_RATED_FREQUENCY (50.0f)
#endif // MOTOR_PARAM_RATED_FREQUENCY

/**
 * @brief Motor Maximum Speed
 */
#ifndef MOTOR_PARAM_MAX_SPEED
#define MOTOR_PARAM_MAX_SPEED (1450.0f)
#endif // MOTOR_PARAM_MAX_SPEED

/**
 * @brief Synchronous Speed Base
 */
#define CTRL_SPEED_RPM_BASE (1500.0f)

/**
 * @brief Nominal DC Bus Voltage
 */
#define MCS_NOMINAL_DC_BUS_VOLTAGE_V (48.0f)

/**
 * @brief Rectangular Voltage Limit
 */
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V (24.0f)

/**
 * @brief Circular Voltage Limit
 */
#define MCS_MAX_CIR_SATURATION_VOLTAGE_V (24.0f)

/**
 * @brief Deadtime Current Deadband
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)

/**
 * @brief Deadtime Current Hysteresis
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)

/**
 * @brief Open Loop Frequency
 */
#define MCS_OPEN_LOOP_FREQ_HZ (30.0f)

/**
 * @brief Open Loop Frequency Slope
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S (20.0f)

/**
 * @brief Optional constant d-axis voltage boost for BUILD_LEVEL 1, in volts.
 */
#define MCS_OPEN_LOOP_VD_REF_V (0.0f)

/**
 * @brief BUILD_LEVEL 1 q-axis voltage at MCS_OPEN_LOOP_FREQ_HZ. Runtime command is ramped proportionally with frequency (V/f).
 */
#define MCS_OPEN_LOOP_VQ_REF_V (6.0f)

/**
 * @brief Closed-loop ACIM magnetizing-current reference; unlike PMSM this normally must remain nonzero.
 */
#define MCS_COMMISSIONING_ID_REF_A (3.0f)

/**
 * @brief High I/F startup excitation current before the observer is sufficiently observable.
 */
#define MCS_SENSORLESS_STARTUP_ID_REF_A (4.0f)

/**
 * @brief I/F excitation reached at the angle-handover speed.
 */
#define MCS_SENSORLESS_HANDOVER_ID_REF_A (3.0f)

/**
 * @brief Absolute I/F electrical frequency where startup Id begins to decrease.
 */
#define MCS_STARTUP_ID_FADE_START_HZ (5.0f)

/**
 * @brief Absolute I/F electrical frequency where startup Id reaches the handover value.
 */
#define MCS_STARTUP_ID_FADE_END_HZ (25.0f)

/**
 * @brief Commissioning Iq
 */
#define MCS_COMMISSIONING_IQ_REF_A (1.0f)

/**
 * @brief Commissioning Speed
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM (300.0f)

/**
 * @brief Magnetizing Time
 */
#define MCS_MAGNETIZING_TIME_MS (300.0f)

/**
 * @brief Mech Position Kp
 */
#define MCS_MECH_POSITION_KP_PU (5.0f)

/**
 * @brief Mech Position Ki
 */
#define MCS_MECH_POSITION_KI_PU_S (1.0f)

/**
 * @brief Mech Velocity Kp
 */
#define MCS_MECH_VELOCITY_KP_PU (1.0f)

/**
 * @brief Mech Velocity Ki
 */
#define MCS_MECH_VELOCITY_KI_PU_S (10.0f)

/**
 * @brief Mech Speed Limit
 */
#define MCS_MECH_SPEED_LIMIT_RPM (1400.0f)

/**
 * @brief Mech Speed Slope
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S (1000.0f)

/**
 * @brief Mech Current Limit
 */
#define MCS_MECH_CURRENT_LIMIT_A (3.0f)

/**
 * @brief Encoder Speed Filter
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)

/**
 * @brief Current-model correction bandwidth while the known I/F field frame owns the current loop and initializes the voltage model.
 */
#define MCS_FO_COMP_BW_HZ (20.0f)

/**
 * @brief Post-handover correction bandwidth used only to reject voltage-integrator DC drift; keep it far below the sensorless operating frequency.
 */
#define MCS_FO_RUN_COMP_BW_HZ (0.5f)

/**
 * @brief Voltage-model leaky-integrator cutoff used to block DC voltage and current offsets. Keep it below the handover frequency and account for its phase lag during tuning.
 */
#define MCS_FO_VM_LEAK_HZ (5.0f)

/**
 * @brief Rotor-flux PLL bandwidth. A modest value avoids speed saturation from low-speed voltage-model phase error.
 */
#define MCS_FO_ATO_BW_HZ (10.0f)

/**
 * @brief Flux Observer Fault Time
 */
#define MCS_FO_FAULT_TIME_MS (20.0f)

/**
 * @brief Multiply DC-bus PU by this target-specific modulator gain when reconstructing applied phase voltage from alpha-beta command.
 */
#ifndef MCS_FO_COMMAND_VOLTAGE_SCALE
#define MCS_FO_COMMAND_VOLTAGE_SCALE (0.5f)
#endif // MCS_FO_COMMAND_VOLTAGE_SCALE

/**
 * @brief I/F electrical frequency at which the flux PLL is released to acquire the voltage-model angle while the current loop remains on the I/F field angle.
 */
#define MCS_FO_ACQUIRE_FREQ_HZ (10.0f)

/**
 * @brief Minimum I/F electrical frequency before a qualified flux observer may take ownership of field angle. Keep this well above MCS_FO_COMP_BW_HZ so the voltage model has directional authority.
 */
#define MCS_FO_HANDOVER_FREQ_HZ (25.0f)

/**
 * @brief Maximum wrapped angle mismatch in electrical-turn PU (1 pu = 360 degrees).
 */
#define MCS_FO_HANDOVER_ANGLE_ERR_PU (0.08f)

/**
 * @brief Maximum observer-versus-I/F synchronous-speed mismatch in PU before the enforced startup synchronization is applied.
 */
#define MCS_FO_HANDOVER_SPEED_ERR_PU (0.10f)

/**
 * @brief Larger speed mismatch that resets handover qualification; the gap above the enter tolerance is noise hysteresis.
 */
#define MCS_FO_HANDOVER_SPEED_EXIT_ERR_PU (0.15f)

/**
 * @brief Continuous qualification time before the sensorless field-angle handover.
 */
#define MCS_FO_HANDOVER_DEBOUNCE_MS (100.0f)

/**
 * @brief Time used to decay captured angle offset and synchronously blend startup Id to closed-loop Id.
 */
#define MCS_FO_HANDOVER_TRANSITION_MS (100.0f)

/**
 * @brief Maximum post-handover synchronous-speed deviation from the continuing I/F reference before fallback qualification.
 */
#define MCS_FO_LOSS_SPEED_ERR_PU (0.15f)

/**
 * @brief Post-handover observer-loss debounce time. A confirmed loss latches I/F fallback until controllers are cleared.
 */
#define MCS_FO_LOSS_DEBOUNCE_MS (20.0f)

/**
 * @brief Maximum DC Bus Voltage
 */
#define MCS_MAX_DC_BUS_VOLTAGE_V (MCS_NOMINAL_DC_BUS_VOLTAGE_V * 1.2f)

/**
 * @brief Minimum DC Bus Voltage
 */
#define MCS_MIN_DC_BUS_VOLTAGE_V (MCS_NOMINAL_DC_BUS_VOLTAGE_V * 0.2f)

/**
 * @brief Maximum Shutdown Current
 */
#define MCS_MAX_SHUTDOWN_CURRENT_A (10.0f)

/**
 * @brief CiA402 Enable Delay
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (0)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be in the range 1..4."
#endif
#if (MCS_ACIM_FEEDBACK_MODE != MCS_ACIM_FEEDBACK_SENSORED) && (MCS_ACIM_FEEDBACK_MODE != MCS_ACIM_FEEDBACK_SENSORLESS)
#error "MCS_ACIM_FEEDBACK_MODE must select sensored or sensorless feedback."
#endif
#if (MCS_FO_VOLTAGE_SOURCE != MCS_FO_VOLTAGE_FROM_COMMAND) && (MCS_FO_VOLTAGE_SOURCE != MCS_FO_VOLTAGE_FROM_MEASUREMENT)
#error "MCS_FO_VOLTAGE_SOURCE must select command reconstruction or phase-voltage measurement."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_ACIM_NT_COMMON_SETTINGS_H_
