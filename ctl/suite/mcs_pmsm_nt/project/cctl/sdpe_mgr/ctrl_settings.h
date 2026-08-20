/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for MCS PMSM NT Direct CCTL Simulation.
 * @note Direct host bindings for the controller, TI-style peripherals, generated MNA inverter, PMSM plant, and the CCTL host CSP.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/ti_boostxl_3phganinv.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Peripheral and simulation constants are generated from this target requirement. */

// Common prefix code: MCS PMSM NT Common Controller Settings
/* Platform-independent settings only. Project hardware is supplied by the including project header. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_NT_CCTL_SDPE_PROJECT_ID "mcs_pmsm_nt_cctl"
#define MCS_PMSM_NT_CCTL_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_CCTL_SDPE_PROJECT_VERSION "1.0.0"
#define MCS_PMSM_NT_CCTL_SDPE_PROJECT_UPDATED_AT "2026-08-20"

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief MC_CURRENT_SAMPLE_PHASE_MODE
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief BUILD_LEVEL
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (4)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief PWM Compare Max
 */
#define CTRL_PWM_CMP_MAX (2499)

/**
 * @brief PWM Deadband
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief System Frequency
 */
#define CTRL_SYS_FREQUENCY real2param(100e6)

/**
 * @brief ADC Voltage Reference
 */
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief DC Bus Voltage Base
 */
#define CTRL_DCBUS_VOLTAGE real2param(48.0)

/**
 * @brief Phase Voltage Base
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Current Base
 */
#define CTRL_CURRENT_BASE real2param(10.0)

/**
 * @brief Encoder Full Scale
 */
#define CTRL_POS_ENC_FS (16384)

/**
 * @brief Encoder Bias
 */
#define CTRL_POS_ENC_BIAS real2param(0.0)

/**
 * @brief Mechanical Division
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Phase Current Sensitivity
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (0.005f * 11.0f)

/**
 * @brief Phase Current Bias
 */
#define CTRL_INVERTER_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_PH_CSA_BIAS_V)

/**
 * @brief Phase Voltage Sensitivity
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (1.0f / 48.0f)

/**
 * @brief Phase Voltage Bias
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC Current Sensitivity
 */
#define CTRL_DC_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC Current Bias
 */
#define CTRL_DC_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC Voltage Sensitivity
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (1.0f / 48.0f)

/**
 * @brief DC Voltage Bias
 */
#define CTRL_DC_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Simulation ADC Resolution
 */
#define CCTL_SIM_ADC_RESOLUTION_BITS (12)

/**
 * @brief Simulation ADC Reference
 */
#define CCTL_SIM_ADC_REFERENCE_V 3.3

/**
 * @brief Simulation eQEP Counts
 */
#define CCTL_SIM_EQEP_COUNTS_PER_REV (16384)

/**
 * @brief Simulation ePWM Clock
 */
#define CCTL_SIM_EPWM_TBCLK_HZ 100.0e6

/**
 * @brief Simulation ePWM Period
 */
#define CCTL_SIM_EPWM_PERIOD_COUNT (2499)

/**
 * @brief Simulation ePWM Rising Deadband
 */
#define CCTL_SIM_EPWM_DBRED_COUNT (100)

/**
 * @brief Simulation ePWM Falling Deadband
 */
#define CCTL_SIM_EPWM_DBFED_COUNT (100)

/**
 * @brief Simulation ADC Trigger Compare
 */
#define CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT (250)

/**
 * @brief Plant Step
 */
#define CCTL_SIM_PLANT_STEP_S 1.0e-7

/**
 * @brief Control Frequency
 */
#define CCTL_SIM_CONTROL_FREQUENCY_HZ 20.0e3

/**
 * @brief Simulation Duration
 */
#define CCTL_SIM_DURATION_S 4.0

/**
 * @brief Simulation DC Bus
 */
#define CCTL_SIM_DC_BUS_V 48.0

/**
 * @brief Load Torque Start
 */
#define CCTL_SIM_LOAD_TORQUE_START_S 0.5

/**
 * @brief Load Torque
 */
#define CCTL_SIM_LOAD_TORQUE_NM 0.02

/**
 * @brief Startup Short Steps
 */
#define CCTL_SIM_STARTUP_SHORT_STEPS (200)

/**
 * @brief PMSM Integration Order
 */
#define CCTL_SIM_PMSM_INTEGRATION_ORDER (1)

/**
 * @brief Output Ring Bytes
 */
#define CCTL_SIM_OUTPUT_RING_BYTES (33554432)

/**
 * @brief Output Batch Bytes
 */
#define CCTL_SIM_OUTPUT_BATCH_BYTES (1048576)

/**
 * @brief Progress Interval
 */
#define CCTL_SIM_PROGRESS_INTERVAL_MS (1000)

/**
 * @brief Simulation Chunk Steps
 */
#define CCTL_SIM_STEP_CHUNK_STEPS (4096)

/**
 * @brief Realtime Priority
 */
#define CCTL_SIM_REALTIME_PRIORITY (1)

/**
 * @brief Pause On Exit
 */
#define CCTL_SIM_PAUSE_ON_EXIT (1)

/**
 * @brief Output Filename
 */
#define CCTL_SIM_OUTPUT_FILENAME "mcs_pmsm_nt_cctl.csv"

/**
 * @brief Motor Type
 */
#define MOTOR_TYPE SM060R20B30MNAD_MOTOR_TYPE

/**
 * @brief Motor Stator Resistance
 */
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS

/**
 * @brief Motor Stator Inductance
 */
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD

/**
 * @brief Motor D-axis Inductance
 */
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD

/**
 * @brief Motor Q-axis Inductance
 */
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ

/**
 * @brief Motor Flux Linkage
 */
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX

/**
 * @brief Motor Pole Pairs
 */
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS

/**
 * @brief Motor Inertia
 */
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA

/**
 * @brief Motor Friction
 */
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION

/**
 * @brief Motor Velocity Constant
 */
#define MOTOR_PARAM_KV SM060R20B30MNAD_KV

/**
 * @brief Motor Back EMF
 */
#define MOTOR_PARAM_EMF SM060R20B30MNAD_EMF

/**
 * @brief Motor Rated Voltage
 */
#define MOTOR_PARAM_RATED_VOLTAGE SM060R20B30MNAD_RATED_VOLTAGE

/**
 * @brief Motor Rated Current
 */
#define MOTOR_PARAM_RATED_CURRENT SM060R20B30MNAD_RATED_CURRENT

/**
 * @brief Motor No-load Current
 */
#define MOTOR_PARAM_NO_LOAD_CURRENT SM060R20B30MNAD_NO_LOAD_CURRENT

/**
 * @brief Motor Rated Frequency
 */
#define MOTOR_PARAM_RATED_FREQUENCY SM060R20B30MNAD_RATED_FREQUENCY

/**
 * @brief Motor Maximum Speed
 */
#define MOTOR_PARAM_MAX_SPEED SM060R20B30MNAD_MAX_SPEED

/**
 * @brief Motor Maximum Torque
 */
#define MOTOR_PARAM_MAX_TORQUE SM060R20B30MNAD_MAX_TORQUE

/**
 * @brief Motor Maximum DC Voltage
 */
#define MOTOR_PARAM_MAX_DC_VOLTAGE SM060R20B30MNAD_MAX_DC_VOLTAGE

/**
 * @brief Motor Maximum Phase Current
 */
#define MOTOR_PARAM_MAX_PH_CURRENT SM060R20B30MNAD_MAX_PH_CURRENT

//=================================================================================================
/**
 * @brief Common fallbacks: MCS PMSM NT Common Controller Settings.
 */

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

// Common tail code: MCS PMSM NT Common Controller Settings
/* Accept the historical misspelling while all source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Reject unsupported incremental build levels at preprocessing time. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

// User project tail code
/* The direct plant already supplies physical sensor biases. */
#undef SPECIFY_ENABLE_ADC_CALIBRATE

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
