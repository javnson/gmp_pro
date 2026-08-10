/**
 * @file sdpe_mcs_pmsm_id_common_settings.h
 * @brief SDPE project bindings for MCS PMSM Identification Common Control.
 * @note Platform-independent motor-control, protection, commissioning and runtime defaults shared by mcs_pmsm_id targets.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_ID_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_ID_COMMON_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Platform-independent settings only. Project hardware is supplied by the including project header. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_ID_COMMON_SDPE_PROJECT_ID "mcs_pmsm_id_common"
#define MCS_PMSM_ID_COMMON_SDPE_PROJECT_SUITE "mcs_pmsm_id"
#define MCS_PMSM_ID_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define MCS_PMSM_ID_COMMON_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Use the discrete controller implementation instead of the default continuous controller path.
 */
// #ifndef PMSM_CTRL_USING_DISCRETE_CTRL
// #define PMSM_CTRL_USING_DISCRETE_CTRL
// #endif // PMSM_CTRL_USING_DISCRETE_CTRL

/**
 * @brief Enable the existing discrete-PID anti-saturation debug path.
 */
// #ifndef _USE_DEBUG_DISCRETE_PID
// #define _USE_DEBUG_DISCRETE_PID
// #endif // _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable the sliding-mode observer path. Disabled to preserve the current sensored-control build.
 */
// #ifndef ENABLE_SMO
// #define ENABLE_SMO
// #endif // ENABLE_SMO

/**
 * @brief Enable motor fault protection processing.
 */
// #ifndef ENABLE_MOTOR_FAULT_PROTECTION
// #define ENABLE_MOTOR_FAULT_PROTECTION
// #endif // ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Runtime.
 */

/**
 * @brief Calibrate ADC offsets before enabling normal control.
 */
// #ifndef SPECIFY_ENABLE_ADC_CALIBRATE
// #define SPECIFY_ENABLE_ADC_CALIBRATE
// #endif // SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #ifndef GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO
// #endif // GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use active-low PWM modulation for the selected inverter gate path.
 */
// #ifndef PWM_MODULATOR_USING_NEGATIVE_LOGIC
// #define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)
// #endif // PWM_MODULATOR_USING_NEGATIVE_LOGIC

/**
 * @brief Use the three-level NPC modulator. Disabled for the two-level three-phase inverter.
 */
// #ifndef USING_NPC_MODULATOR
// #define USING_NPC_MODULATOR
// #endif // USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning level. 1: V/f voltage open loop; 2: current loop with synthetic electrical angle; 3: current loop with encoder angle; 4: speed loop with encoder feedback.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#ifndef BUILD_LEVEL
#define BUILD_LEVEL (2)
#endif // BUILD_LEVEL

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Enable the seven ADC slots used by the PMSM SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (127)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable three PWM slots and six monitor slots used by the PMSM SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (4128775)
#endif // GMP_PIL_TX_MASK

/**
 * @brief Main motor-control ISR frequency in hertz.
 */
#ifndef CONTROLLER_FREQUENCY
#define CONTROLLER_FREQUENCY (20e3f)
#endif // CONTROLLER_FREQUENCY

/**
 * @brief Minimum absolute phase current in amperes before PWM dead-time compensation selects a current direction. Converted to PU with CTRL_CURRENT_BASE at initialization.
 */
#ifndef MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)
#endif // MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A

/**
 * @brief Phase-current hysteresis band in amperes used to prevent dead-time compensation direction chatter around zero current.
 */
#ifndef MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)
#endif // MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A

/**
 * @brief Cutoff frequency in hertz of the low-pass filter applied to encoder-derived mechanical speed.
 */
#ifndef MCS_ENCODER_SPEED_FILTER_FC_HZ
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)
#endif // MCS_ENCODER_SPEED_FILTER_FC_HZ

/**
 * @brief Position-loop proportional gain. Input is mechanical position error in PU revolutions and output is speed reference in PU, so the gain is speed_pu/position_pu.
 */
#ifndef MCS_MECH_POSITION_KP_PU
#define MCS_MECH_POSITION_KP_PU (5.0f)
#endif // MCS_MECH_POSITION_KP_PU

/**
 * @brief Position-loop integral gain in speed_pu/(position_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#ifndef MCS_MECH_POSITION_KI_PU_S
#define MCS_MECH_POSITION_KI_PU_S (1.0f)
#endif // MCS_MECH_POSITION_KI_PU_S

/**
 * @brief Velocity-loop proportional gain. Input is speed error in PU and output is q-axis current/torque reference in PU, so the gain is current_pu/speed_pu.
 */
#ifndef MCS_MECH_VELOCITY_KP_PU
#define MCS_MECH_VELOCITY_KP_PU (5.0f)
#endif // MCS_MECH_VELOCITY_KP_PU

/**
 * @brief Velocity-loop integral gain in current_pu/(speed_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#ifndef MCS_MECH_VELOCITY_KI_PU_S
#define MCS_MECH_VELOCITY_KI_PU_S (1.0f)
#endif // MCS_MECH_VELOCITY_KI_PU_S

/**
 * @brief D-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#ifndef MCS_COMMISSIONING_ID_REF_A
#define MCS_COMMISSIONING_ID_REF_A (1.0f)
#endif // MCS_COMMISSIONING_ID_REF_A

/**
 * @brief Q-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#ifndef MCS_COMMISSIONING_IQ_REF_A
#define MCS_COMMISSIONING_IQ_REF_A (1.0f)
#endif // MCS_COMMISSIONING_IQ_REF_A

/**
 * @brief Mechanical speed reference in rpm used by BUILD_LEVEL 4 commissioning. Converted to PU using MOTOR_PARAM_MAX_SPEED.
 */
#ifndef MCS_COMMISSIONING_SPEED_REF_RPM
#define MCS_COMMISSIONING_SPEED_REF_RPM (300.0f)
#endif // MCS_COMMISSIONING_SPEED_REF_RPM

/**
 * @brief Absolute mechanical speed-command limit in rpm. It is divided by MOTOR_PARAM_MAX_SPEED to obtain the controller PU limit.
 */
#ifndef MCS_MECH_SPEED_LIMIT_RPM
#define MCS_MECH_SPEED_LIMIT_RPM (3000.0f)
#endif // MCS_MECH_SPEED_LIMIT_RPM

/**
 * @brief Maximum mechanical speed-command slew rate in rpm/s. The configured value corresponds to 1 PU/s for the selected 3000 rpm motor.
 */
#ifndef MCS_MECH_SPEED_SLOPE_RPM_S
#define MCS_MECH_SPEED_SLOPE_RPM_S (3000.0f)
#endif // MCS_MECH_SPEED_SLOPE_RPM_S

/**
 * @brief Absolute q-axis current/torque command limit in amperes. It is divided by CTRL_CURRENT_BASE to obtain the controller PU limit; 3 A corresponds to the previous 0.3 PU setting.
 */
#ifndef MCS_MECH_CURRENT_LIMIT_A
#define MCS_MECH_CURRENT_LIMIT_A (3.0f)
#endif // MCS_MECH_CURRENT_LIMIT_A

/**
 * @brief Rectangular saturation limit for d/q axes in V.
 */
#ifndef MCS_MAX_RECT_SATURATION_VOLTAGE_V
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V (10.0f)
#endif // MCS_MAX_RECT_SATURATION_VOLTAGE_V

/**
 * @brief
 */
#ifndef MCS_MAX_DC_BUS_VOLTAGE_V
#define MCS_MAX_DC_BUS_VOLTAGE_V (CTRL_DCBUS_VOLTAGE*1.2f)
#endif // MCS_MAX_DC_BUS_VOLTAGE_V

/**
 * @brief
 */
#ifndef MCS_MIN_DC_BUS_VOLTAGE_V
#define MCS_MIN_DC_BUS_VOLTAGE_V (CTRL_DCBUS_VOLTAGE*0.2f)
#endif // MCS_MIN_DC_BUS_VOLTAGE_V

/**
 * @brief
 */
#ifndef MOTOR_PARAM_INERTIA
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA
#endif // MOTOR_PARAM_INERTIA

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FLUX
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX
#endif // MOTOR_PARAM_FLUX

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LD
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD
#endif // MOTOR_PARAM_LD

/**
 * @brief
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS
#endif // MOTOR_PARAM_RS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LQ
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ
#endif // MOTOR_PARAM_LQ

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LS
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD
#endif // MOTOR_PARAM_LS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FRICTION
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION
#endif // MOTOR_PARAM_FRICTION

/**
 * @brief
 */
#ifndef MOTOR_PARAM_POLE_PAIRS
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS
#endif // MOTOR_PARAM_POLE_PAIRS

/**
 * @brief Controller startup delay in milliseconds.
 */
#ifndef CTRL_STARTUP_DELAY
#define CTRL_STARTUP_DELAY (100)
#endif // CTRL_STARTUP_DELAY

/**
 * @brief Minimum delay in milliseconds before the CiA402 state machine enters Operation Enabled. The project control tick is expressed in milliseconds.
 */
#ifndef MCS_CIA402_OPERATION_ENABLE_DELAY_MS
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)
#endif // MCS_CIA402_OPERATION_ENABLE_DELAY_MS

/**
 * @brief Cutoff frequency in hertz of the second-order low-pass filter used while estimating ADC zero offsets.
 */
#ifndef MCS_ADC_CALIBRATOR_FC_HZ
#define MCS_ADC_CALIBRATOR_FC_HZ (20.0f)
#endif // MCS_ADC_CALIBRATOR_FC_HZ

/**
 * @brief Quality factor of the ADC calibration low-pass filter; 0.707 gives an approximately Butterworth second-order response.
 */
#ifndef MCS_ADC_CALIBRATOR_Q
#define MCS_ADC_CALIBRATOR_Q (0.707f)
#endif // MCS_ADC_CALIBRATOR_Q

/**
 * @brief Enable the target ADC-calibration preparation handshake.
 */
#ifndef MCS_PMSM_ID_ENABLE_PREPARE
#define MCS_PMSM_ID_ENABLE_PREPARE (1)
#endif // MCS_PMSM_ID_ENABLE_PREPARE

/**
 * @brief Enable stator-resistance and inverter dead-time voltage identification.
 */
#ifndef MCS_PMSM_ID_ENABLE_RS_DT
#define MCS_PMSM_ID_ENABLE_RS_DT (1)
#endif // MCS_PMSM_ID_ENABLE_RS_DT

/**
 * @brief Enable d/q-axis inductance pulse identification.
 */
#ifndef MCS_PMSM_ID_ENABLE_LDQ
#define MCS_PMSM_ID_ENABLE_LDQ (1)
#endif // MCS_PMSM_ID_ENABLE_LDQ

/**
 * @brief Enable PM flux-linkage identification.
 */
#ifndef MCS_PMSM_ID_ENABLE_FLUX
#define MCS_PMSM_ID_ENABLE_FLUX (1)
#endif // MCS_PMSM_ID_ENABLE_FLUX

/**
 * @brief Maximum closed-loop current in amperes used by the Rs/dead-time sweep.
 */
#ifndef MCS_PMSM_ID_RSDT_MAX_CURRENT_A
#define MCS_PMSM_ID_RSDT_MAX_CURRENT_A (5.0f)
#endif // MCS_PMSM_ID_RSDT_MAX_CURRENT_A

/**
 * @brief Minimum closed-loop current in amperes used by the Rs/dead-time sweep.
 */
#ifndef MCS_PMSM_ID_RSDT_MIN_CURRENT_A
#define MCS_PMSM_ID_RSDT_MIN_CURRENT_A (1.0f)
#endif // MCS_PMSM_ID_RSDT_MIN_CURRENT_A

/**
 * @brief Number of positive current points in the Rs/dead-time regression.
 */
#ifndef MCS_PMSM_ID_RSDT_STEPS
#define MCS_PMSM_ID_RSDT_STEPS (5)
#endif // MCS_PMSM_ID_RSDT_STEPS

/**
 * @brief Rotor alignment dwell in seconds before the Rs/dead-time sweep.
 */
#ifndef MCS_PMSM_ID_RSDT_ALIGN_TIME_S
#define MCS_PMSM_ID_RSDT_ALIGN_TIME_S (1.0f)
#endif // MCS_PMSM_ID_RSDT_ALIGN_TIME_S

/**
 * @brief Settling delay in seconds after changing each Rs/dead-time current point.
 */
#ifndef MCS_PMSM_ID_RSDT_MEASURE_DELAY_S
#define MCS_PMSM_ID_RSDT_MEASURE_DELAY_S (0.2f)
#endif // MCS_PMSM_ID_RSDT_MEASURE_DELAY_S

/**
 * @brief Number of ISR samples averaged at each Rs/dead-time current point.
 */
#ifndef MCS_PMSM_ID_RSDT_MEASURE_POINTS
#define MCS_PMSM_ID_RSDT_MEASURE_POINTS (100)
#endif // MCS_PMSM_ID_RSDT_MEASURE_POINTS

/**
 * @brief Physical d/q pulse voltage in volts used for inductance identification.
 */
#ifndef MCS_PMSM_ID_LDQ_PULSE_VOLTAGE_V
#define MCS_PMSM_ID_LDQ_PULSE_VOLTAGE_V (0.277128f)
#endif // MCS_PMSM_ID_LDQ_PULSE_VOLTAGE_V

/**
 * @brief Maximum current bias in amperes used for the inductance profile.
 */
#ifndef MCS_PMSM_ID_LDQ_MAX_BIAS_CURRENT_A
#define MCS_PMSM_ID_LDQ_MAX_BIAS_CURRENT_A (5.0f)
#endif // MCS_PMSM_ID_LDQ_MAX_BIAS_CURRENT_A

/**
 * @brief Number of bias points recorded for each inductance axis.
 */
#ifndef MCS_PMSM_ID_LDQ_BIAS_STEPS
#define MCS_PMSM_ID_LDQ_BIAS_STEPS (12)
#endif // MCS_PMSM_ID_LDQ_BIAS_STEPS

/**
 * @brief D-axis alignment current in amperes used before inductance pulses.
 */
#ifndef MCS_PMSM_ID_LDQ_ALIGN_CURRENT_A
#define MCS_PMSM_ID_LDQ_ALIGN_CURRENT_A (5.0f)
#endif // MCS_PMSM_ID_LDQ_ALIGN_CURRENT_A

/**
 * @brief Bias-current settling time in seconds before each inductance pulse.
 */
#ifndef MCS_PMSM_ID_LDQ_SETTLE_TIME_S
#define MCS_PMSM_ID_LDQ_SETTLE_TIME_S (0.2f)
#endif // MCS_PMSM_ID_LDQ_SETTLE_TIME_S

/**
 * @brief Inductance voltage-pulse duration in seconds.
 */
#ifndef MCS_PMSM_ID_LDQ_PULSE_TIME_S
#define MCS_PMSM_ID_LDQ_PULSE_TIME_S (0.002f)
#endif // MCS_PMSM_ID_LDQ_PULSE_TIME_S

/**
 * @brief Zero-voltage cooldown time in seconds between inductance pulses.
 */
#ifndef MCS_PMSM_ID_LDQ_COOLDOWN_TIME_S
#define MCS_PMSM_ID_LDQ_COOLDOWN_TIME_S (0.05f)
#endif // MCS_PMSM_ID_LDQ_COOLDOWN_TIME_S

/**
 * @brief Minimum mechanical speed in rpm used by the flux-linkage regression.
 */
#ifndef MCS_PMSM_ID_FLUX_MIN_SPEED_RPM
#define MCS_PMSM_ID_FLUX_MIN_SPEED_RPM (300.0f)
#endif // MCS_PMSM_ID_FLUX_MIN_SPEED_RPM

/**
 * @brief Maximum mechanical speed in rpm used by the flux-linkage regression.
 */
#ifndef MCS_PMSM_ID_FLUX_MAX_SPEED_RPM
#define MCS_PMSM_ID_FLUX_MAX_SPEED_RPM (1800.0f)
#endif // MCS_PMSM_ID_FLUX_MAX_SPEED_RPM

/**
 * @brief Number of speed points used by the flux-linkage regression.
 */
#ifndef MCS_PMSM_ID_FLUX_STEPS
#define MCS_PMSM_ID_FLUX_STEPS (6)
#endif // MCS_PMSM_ID_FLUX_STEPS

/**
 * @brief Current magnitude in amperes used for I/F flux-linkage rotation.
 */
#ifndef MCS_PMSM_ID_FLUX_IF_CURRENT_A
#define MCS_PMSM_ID_FLUX_IF_CURRENT_A (4.0f)
#endif // MCS_PMSM_ID_FLUX_IF_CURRENT_A

/**
 * @brief Settling time in seconds at each flux-identification speed.
 */
#ifndef MCS_PMSM_ID_FLUX_SETTLE_TIME_S
#define MCS_PMSM_ID_FLUX_SETTLE_TIME_S (2.0f)
#endif // MCS_PMSM_ID_FLUX_SETTLE_TIME_S

/**
 * @brief Number of ISR samples averaged at each flux-identification speed.
 */
#ifndef MCS_PMSM_ID_FLUX_MEASURE_POINTS
#define MCS_PMSM_ID_FLUX_MEASURE_POINTS (2000)
#endif // MCS_PMSM_ID_FLUX_MEASURE_POINTS

/**
 * @brief Enable sensored phase-A alignment, pole-pair detection and encoder-offset calibration before electrical/mechanical identification.
 */
#ifndef MCS_PMSM_ID_ENABLE_ENCODER_CALIBRATION
#define MCS_PMSM_ID_ENABLE_ENCODER_CALIBRATION (1)
#endif // MCS_PMSM_ID_ENABLE_ENCODER_CALIBRATION

/**
 * @brief Enable the sensored constant-Iq acceleration and PWM-off coast-down mechanical identification stage.
 */
#ifndef MCS_PMSM_ID_ENABLE_MECHANICAL_ID
#define MCS_PMSM_ID_ENABLE_MECHANICAL_ID (1)
#endif // MCS_PMSM_ID_ENABLE_MECHANICAL_ID

/**
 * @brief Closed-loop d-axis current in amperes used to lock the rotor to phase A during encoder calibration.
 */
#ifndef MCS_PMSM_ID_ENCODER_ALIGN_CURRENT_A
#define MCS_PMSM_ID_ENCODER_ALIGN_CURRENT_A (1.0f)
#endif // MCS_PMSM_ID_ENCODER_ALIGN_CURRENT_A

/**
 * @brief PWM-off observation time in seconds used to detect a randomly jumping encoder.
 */
#ifndef MCS_PMSM_ID_ENCODER_NOISE_CHECK_TIME_S
#define MCS_PMSM_ID_ENCODER_NOISE_CHECK_TIME_S (0.10f)
#endif // MCS_PMSM_ID_ENCODER_NOISE_CHECK_TIME_S

/**
 * @brief Time in seconds allowed for the rotor to settle at the phase-A electrical zero.
 */
#ifndef MCS_PMSM_ID_ENCODER_ALIGN_SETTLE_TIME_S
#define MCS_PMSM_ID_ENCODER_ALIGN_SETTLE_TIME_S (0.30f)
#endif // MCS_PMSM_ID_ENCODER_ALIGN_SETTLE_TIME_S

/**
 * @brief Electrical revolutions per second used while rotating the alignment current vector.
 */
#ifndef MCS_PMSM_ID_ENCODER_SWEEP_ELEC_HZ
#define MCS_PMSM_ID_ENCODER_SWEEP_ELEC_HZ (1.0f)
#endif // MCS_PMSM_ID_ENCODER_SWEEP_ELEC_HZ

/**
 * @brief Dwell time in seconds at phase A after each complete electrical revolution.
 */
#ifndef MCS_PMSM_ID_ENCODER_ANCHOR_SETTLE_TIME_S
#define MCS_PMSM_ID_ENCODER_ANCHOR_SETTLE_TIME_S (0.10f)
#endif // MCS_PMSM_ID_ENCODER_ANCHOR_SETTLE_TIME_S

/**
 * @brief Maximum wrapped mechanical-position change accepted in one control ISR sample.
 */
#ifndef MCS_PMSM_ID_ENCODER_MAX_SAMPLE_JUMP_PU
#define MCS_PMSM_ID_ENCODER_MAX_SAMPLE_JUMP_PU (0.02f)
#endif // MCS_PMSM_ID_ENCODER_MAX_SAMPLE_JUMP_PU

/**
 * @brief Maximum unwrapped position span accepted during the PWM-off encoder noise test.
 */
#ifndef MCS_PMSM_ID_ENCODER_MAX_STATIONARY_SPAN_PU
#define MCS_PMSM_ID_ENCODER_MAX_STATIONARY_SPAN_PU (0.002f)
#endif // MCS_PMSM_ID_ENCODER_MAX_STATIONARY_SPAN_PU

/**
 * @brief Minimum net mechanical motion required during one electrical revolution; below this reports an uncoupled/stuck encoder.
 */
#ifndef MCS_PMSM_ID_ENCODER_MIN_CYCLE_MOTION_PU
#define MCS_PMSM_ID_ENCODER_MIN_CYCLE_MOTION_PU (0.02f)
#endif // MCS_PMSM_ID_ENCODER_MIN_CYCLE_MOTION_PU

/**
 * @brief Maximum absolute deviation of each electrical-cycle mechanical motion from the measured mean.
 */
#ifndef MCS_PMSM_ID_ENCODER_MAX_CYCLE_DEVIATION_PU
#define MCS_PMSM_ID_ENCODER_MAX_CYCLE_DEVIATION_PU (0.03f)
#endif // MCS_PMSM_ID_ENCODER_MAX_CYCLE_DEVIATION_PU

/**
 * @brief Maximum wrapped mechanical-position error accepted when returning to the first phase-A anchor.
 */
#ifndef MCS_PMSM_ID_ENCODER_ZERO_RETURN_TOLERANCE_PU
#define MCS_PMSM_ID_ENCODER_ZERO_RETURN_TOLERANCE_PU (0.02f)
#endif // MCS_PMSM_ID_ENCODER_ZERO_RETURN_TOLERANCE_PU

/**
 * @brief Maximum pole-pair count searched before encoder calibration stops with a zero-return fault.
 */
#ifndef MCS_PMSM_ID_ENCODER_MAX_POLE_PAIRS
#define MCS_PMSM_ID_ENCODER_MAX_POLE_PAIRS (16)
#endif // MCS_PMSM_ID_ENCODER_MAX_POLE_PAIRS

/**
 * @brief User-selected mechanical target speed in rpm for the identification test.
 */
#ifndef MCS_PMSM_ID_MECH_TARGET_SPEED_RPM
#define MCS_PMSM_ID_MECH_TARGET_SPEED_RPM (1000.0f)
#endif // MCS_PMSM_ID_MECH_TARGET_SPEED_RPM

/**
 * @brief User-selected q-axis current in amperes applied by the real current loop during acceleration.
 */
#ifndef MCS_PMSM_ID_MECH_ACCEL_CURRENT_A
#define MCS_PMSM_ID_MECH_ACCEL_CURRENT_A (1.0f)
#endif // MCS_PMSM_ID_MECH_ACCEL_CURRENT_A

/**
 * @brief Lower acceleration/coast fitting boundary relative to the selected target speed.
 */
#ifndef MCS_PMSM_ID_MECH_FIT_LOW_RATIO
#define MCS_PMSM_ID_MECH_FIT_LOW_RATIO (0.30f)
#endif // MCS_PMSM_ID_MECH_FIT_LOW_RATIO

/**
 * @brief Upper acceleration/coast fitting boundary relative to the selected target speed.
 */
#ifndef MCS_PMSM_ID_MECH_FIT_HIGH_RATIO
#define MCS_PMSM_ID_MECH_FIT_HIGH_RATIO (0.70f)
#endif // MCS_PMSM_ID_MECH_FIT_HIGH_RATIO

/**
 * @brief Target-speed ratio at which PWM is physically disabled to start free coast-down.
 */
#ifndef MCS_PMSM_ID_MECH_PWM_OFF_RATIO
#define MCS_PMSM_ID_MECH_PWM_OFF_RATIO (0.75f)
#endif // MCS_PMSM_ID_MECH_PWM_OFF_RATIO

/**
 * @brief Safety timeout in seconds covering acceleration and free coast-down.
 */
#ifndef MCS_PMSM_ID_MECH_MAX_TEST_TIME_S
#define MCS_PMSM_ID_MECH_MAX_TEST_TIME_S (20.0f)
#endif // MCS_PMSM_ID_MECH_MAX_TEST_TIME_S

/**
 * @brief Expected maximum combined duration of the two recorded 30%-70% speed curves.
 */
#ifndef MCS_PMSM_ID_MECH_RECORD_TIME_S
#define MCS_PMSM_ID_MECH_RECORD_TIME_S (10.0f)
#endif // MCS_PMSM_ID_MECH_RECORD_TIME_S

/**
 * @brief Minimum coefficient of determination accepted for acceleration-versus-speed regression.
 */
#ifndef MCS_PMSM_ID_MECH_MIN_FIT_R2
#define MCS_PMSM_ID_MECH_MIN_FIT_R2 (0.80f)
#endif // MCS_PMSM_ID_MECH_MIN_FIT_R2

/**
 * @brief Minimum differentiated samples required in each acceleration and coast curve.
 */
#ifndef MCS_PMSM_ID_MECH_MIN_FIT_SAMPLES
#define MCS_PMSM_ID_MECH_MIN_FIT_SAMPLES (30)
#endif // MCS_PMSM_ID_MECH_MIN_FIT_SAMPLES

// User project tail code
/* Accept the historical misspelling while all source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Reject unsupported incremental build levels at preprocessing time. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_ID_COMMON_SETTINGS_H_
