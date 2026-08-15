/**
 * @file sdpe_mcs_pmsm_nt_common_settings.h
 * @brief SDPE project bindings for MCS PMSM NT Common Controller Settings.
 * @note Platform-independent controller contract shared by every mcs_pmsm_nt implementation. Hardware timing, sensing and peripheral bindings remain in each project SDPE requirement.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_COMMON_SETTINGS_H_

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

#define MCS_PMSM_NT_COMMON_SDPE_PROJECT_ID "mcs_pmsm_nt_common"
#define MCS_PMSM_NT_COMMON_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_COMMON_SDPE_PROJECT_VERSION "1.1.0"
#define MCS_PMSM_NT_COMMON_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Use the discrete controller implementation instead of the default continuous controller path.
 */
// #define PMSM_CTRL_USING_DISCRETE_CTRL

/**
 * @brief Enable the existing discrete-PID anti-saturation debug path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable the sliding-mode observer path. Disabled to preserve the current sensored-control build.
 */
// #define ENABLE_SMO

/**
 * @brief Enable motor fault protection processing.
 */
// #define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Runtime.
 */

/**
 * @brief Calibrate ADC offsets before enabling normal control.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use active-low PWM modulation for the selected inverter gate path.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

/**
 * @brief Use the three-level NPC modulator. Disabled for the two-level three-phase inverter.
 */
// #define USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning level. 1: V/f voltage open loop; 2: current loop with synthetic electrical angle; 3: current loop with encoder angle; 4: speed loop with encoder feedback.
 *        Options: (1), (2), (3), (4)
 */
#ifndef BUILD_LEVEL
#define BUILD_LEVEL (1)
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
#define CONTROLLER_FREQUENCY real2param(20e3)

/**
 * @brief Minimum absolute phase current in amperes before PWM dead-time compensation selects a current direction. Converted to PU with CTRL_CURRENT_BASE at initialization.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A real2param(0.2)

/**
 * @brief Phase-current hysteresis band in amperes used to prevent dead-time compensation direction chatter around zero current.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A real2param(0.05)

/**
 * @brief
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS real2param(0.165)
#endif // MOTOR_PARAM_RS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LS
#define MOTOR_PARAM_LS real2param(0.45e-3)
#endif // MOTOR_PARAM_LS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LD
#define MOTOR_PARAM_LD real2param(0.45e-3)
#endif // MOTOR_PARAM_LD

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LQ
#define MOTOR_PARAM_LQ real2param(0.45e-3)
#endif // MOTOR_PARAM_LQ

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FLUX
#define MOTOR_PARAM_FLUX real2param(0.0066843949493427743)
#endif // MOTOR_PARAM_FLUX

/**
 * @brief
 */
#ifndef MOTOR_PARAM_POLE_PAIRS
#define MOTOR_PARAM_POLE_PAIRS (4)
#endif // MOTOR_PARAM_POLE_PAIRS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_INERTIA
#define MOTOR_PARAM_INERTIA real2param(497.0)
#endif // MOTOR_PARAM_INERTIA

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FRICTION
#define MOTOR_PARAM_FRICTION real2param(755.0)
#endif // MOTOR_PARAM_FRICTION

/**
 * @brief Maximum mechanical speed of the common reference motor in rpm.
 */
#ifndef MOTOR_PARAM_MAX_SPEED
#define MOTOR_PARAM_MAX_SPEED real2param(3000.0)
#endif // MOTOR_PARAM_MAX_SPEED

/**
 * @brief Mechanical speed base in rpm.
 */
#ifndef CTRL_SPEED_RPM_BASE
#define CTRL_SPEED_RPM_BASE MOTOR_PARAM_MAX_SPEED
#endif // CTRL_SPEED_RPM_BASE

/**
 * @brief Absolute mechanical speed-command limit in rpm. It is divided by MOTOR_PARAM_MAX_SPEED to obtain the controller PU limit.
 */
#define MCS_MECH_SPEED_LIMIT_RPM real2param(3000.0)

/**
 * @brief Maximum mechanical speed-command slew rate in rpm/s. The configured value corresponds to 1 PU/s for the selected 3000 rpm motor.
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S real2param(3000.0)

/**
 * @brief Absolute q-axis current/torque command limit in amperes. It is divided by CTRL_CURRENT_BASE to obtain the controller PU limit; 3 A corresponds to the previous 0.3 PU setting.
 */
#define MCS_MECH_CURRENT_LIMIT_A real2param(3.0)

/**
 * @brief Rectangular saturation limit for d/q axes in V.
 */
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V real2param(10.0)

/**
 * @brief Nominal DC-bus voltage used by common protection thresholds and target per-unit bases.
 */
#define MCS_NOMINAL_DC_BUS_VOLTAGE_V real2param(80.0)

/**
 * @brief
 */
#define MCS_MAX_DC_BUS_VOLTAGE_V (MCS_NOMINAL_DC_BUS_VOLTAGE_V*1.2f)

/**
 * @brief The current limit value at which the machine must be shut down.
 */
#ifndef MCS_MAX_SHUTDOWN_CURRENT_A
#define MCS_MAX_SHUTDOWN_CURRENT_A real2param(10.0)
#endif // MCS_MAX_SHUTDOWN_CURRENT_A

/**
 * @brief Circular saturation limit for voltage vector magnitude in V.
 */
#define MCS_MAX_CIR_SATURATION_VOLTAGE_V real2param(10.0)

/**
 * @brief Cutoff frequency in hertz of the low-pass filter applied to encoder-derived mechanical speed.
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ real2param(20.0)

/**
 * @brief Cutoff frequency in hertz of the second-order low-pass filter used while estimating ADC zero offsets.
 */
#define MCS_ADC_CALIBRATOR_FC_HZ real2param(20.0)

/**
 * @brief Quality factor of the ADC calibration low-pass filter; 0.707 gives an approximately Butterworth second-order response.
 */
#define MCS_ADC_CALIBRATOR_Q real2param(0.707)

/**
 * @brief D-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#define MCS_COMMISSIONING_ID_REF_A real2param(1.0)

/**
 * @brief Q-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#define MCS_COMMISSIONING_IQ_REF_A real2param(1.0)

/**
 * @brief Mechanical speed reference in rpm used by BUILD_LEVEL 4 commissioning. Converted to PU using MOTOR_PARAM_MAX_SPEED.
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM real2param(300.0)

/**
 * @brief Electrical frequency command in hertz used by the BUILD_LEVEL 1 V/f path and the BUILD_LEVEL 2 synthetic-angle current-loop path.
 */
#define MCS_OPEN_LOOP_FREQ_HZ real2param(20.0)

/**
 * @brief Maximum electrical-frequency slew rate in hertz per second for the synthetic angle generator.
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S real2param(20.0)

/**
 * @brief Position-loop proportional gain. Input is mechanical position error in PU revolutions and output is speed reference in PU, so the gain is speed_pu/position_pu.
 */
#define MCS_MECH_POSITION_KP_PU real2param(5.0)

/**
 * @brief Position-loop integral gain in speed_pu/(position_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#define MCS_MECH_POSITION_KI_PU_S real2param(1.0)

/**
 * @brief Velocity-loop proportional gain. Input is speed error in PU and output is q-axis current/torque reference in PU, so the gain is current_pu/speed_pu.
 */
#define MCS_MECH_VELOCITY_KP_PU real2param(5.0)

/**
 * @brief Velocity-loop integral gain in current_pu/(speed_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#define MCS_MECH_VELOCITY_KI_PU_S real2param(1.0)

/**
 * @brief Controller startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Minimum delay in milliseconds before the CiA402 state machine enters Operation Enabled. The project control tick is expressed in milliseconds.
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief
 */
#define MCS_MIN_DC_BUS_VOLTAGE_V (MCS_NOMINAL_DC_BUS_VOLTAGE_V*0.2f)

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

#endif // _PROJECT_SDPE_MCS_PMSM_NT_COMMON_SETTINGS_H_
