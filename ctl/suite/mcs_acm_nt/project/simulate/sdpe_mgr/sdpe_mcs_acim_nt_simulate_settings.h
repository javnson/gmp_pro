/**
 * @file sdpe_mcs_acim_nt_simulate_settings.h
 * @brief SDPE project bindings for MCS ACIM NT Windows/Simulink SIL.
 * @note Windows SIL bindings for MCS_STD_ACM_MODEL.slx.
 */

#ifndef _PROJECT_SDPE_MCS_ACIM_NT_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_MCS_ACIM_NT_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/acm_motor/acm_4p24v.h>
#include <ctl/hardware_preset/inverter_3ph/ti_boostxl_3phganinv.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Target-specific controller definitions are emitted from SDPE options. */

// Common prefix code: MCS ACIM NT Common Controller Settings
#define MCS_ACIM_FEEDBACK_SENSORED (1)
#define MCS_ACIM_FEEDBACK_SENSORLESS (2)
#define MCS_FO_VOLTAGE_FROM_COMMAND (1)
#define MCS_FO_VOLTAGE_FROM_MEASUREMENT (2)

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_ACIM_NT_SIMULATE_SDPE_PROJECT_ID "mcs_acim_nt_simulate"
#define MCS_ACIM_NT_SIMULATE_SDPE_PROJECT_SUITE "mcs_acim_nt"
#define MCS_ACIM_NT_SIMULATE_SDPE_PROJECT_VERSION "0.1.0"
#define MCS_ACIM_NT_SIMULATE_SDPE_PROJECT_UPDATED_AT "2026-08-09"

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief Directly sampled phase-current count.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)

//=================================================================================================
/**
 * @brief PWM.
 */

/**
 * @brief PWM compare polarity. MCS_STD_ACM_MODEL uses positive compare logic; hardware targets must select their own gate-driver polarity.
 *        Options: (0), (1)
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (0)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Pwm Compare Max
 */
#define CTRL_PWM_CMP_MAX (2500 - 1)

/**
 * @brief Pwm Deadband
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief System Frequency
 */
#define CTRL_SYS_FREQUENCY (CONTROLLER_FREQUENCY)

/**
 * @brief Adc Voltage Reference
 */
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief Dc Bus Voltage Base
 */
#define CTRL_DCBUS_VOLTAGE (MCS_NOMINAL_DC_BUS_VOLTAGE_V)

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
 * @brief Center-aligned bridge reconstruction: phase voltage equals one half of DC-bus voltage times the centered alpha-beta command.
 */
#define MCS_FO_COMMAND_VOLTAGE_SCALE real2param(0.5)

/**
 * @brief Phase Current Sensitivity
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM * TI_BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V)

/**
 * @brief Phase Current Bias
 */
#define CTRL_INVERTER_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_PH_CSA_BIAS_V)

/**
 * @brief Phase Voltage Sensitivity
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase Voltage Bias
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Dc Current Sensitivity
 */
#define CTRL_DC_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief Dc Current Bias
 */
#define CTRL_DC_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief Dc Voltage Sensitivity
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief Dc Voltage Bias
 */
#define CTRL_DC_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Motor Rs
 */
#define MOTOR_PARAM_RS ACM_4P24V_RS

/**
 * @brief Motor Rr
 */
#define MOTOR_PARAM_RR ACM_4P24V_RR

/**
 * @brief Motor L1s
 */
#define MOTOR_PARAM_L1S ACM_4P24V_L1S

/**
 * @brief Motor L1r
 */
#define MOTOR_PARAM_L1R ACM_4P24V_L1R

/**
 * @brief Motor Lm
 */
#define MOTOR_PARAM_LM ACM_4P24V_LM

/**
 * @brief Motor Pole Pairs
 */
#define MOTOR_PARAM_POLE_PAIRS ACM_4P24V_POLE_PAIRS

/**
 * @brief Motor Rated Frequency
 */
#define MOTOR_PARAM_RATED_FREQUENCY ACM_4P24V_RATED_FREQUENCY

/**
 * @brief Motor Maximum Speed
 */
#define MOTOR_PARAM_MAX_SPEED ACM_4P24V_MAX_SPEED

//=================================================================================================
/**
 * @brief Common fallbacks: MCS ACIM NT Common Controller Settings.
 */

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
#define CONTROLLER_FREQUENCY real2param(20e3)

/**
 * @brief Motor Rs
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS real2param(0.329)
#endif // MOTOR_PARAM_RS

/**
 * @brief Motor Rr
 */
#ifndef MOTOR_PARAM_RR
#define MOTOR_PARAM_RR real2param(0.44)
#endif // MOTOR_PARAM_RR

/**
 * @brief Motor L1s
 */
#ifndef MOTOR_PARAM_L1S
#define MOTOR_PARAM_L1S real2param(0.0003)
#endif // MOTOR_PARAM_L1S

/**
 * @brief Motor L1r
 */
#ifndef MOTOR_PARAM_L1R
#define MOTOR_PARAM_L1R real2param(0.0005)
#endif // MOTOR_PARAM_L1R

/**
 * @brief Motor Lm
 */
#ifndef MOTOR_PARAM_LM
#define MOTOR_PARAM_LM real2param(0.0012)
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
#define MOTOR_PARAM_RATED_FREQUENCY real2param(50.0)
#endif // MOTOR_PARAM_RATED_FREQUENCY

/**
 * @brief Motor Maximum Speed
 */
#ifndef MOTOR_PARAM_MAX_SPEED
#define MOTOR_PARAM_MAX_SPEED real2param(1450.0)
#endif // MOTOR_PARAM_MAX_SPEED

/**
 * @brief Synchronous Speed Base
 */
#define CTRL_SPEED_RPM_BASE real2param(1500.0)

/**
 * @brief Nominal DC Bus Voltage
 */
#define MCS_NOMINAL_DC_BUS_VOLTAGE_V real2param(48.0)

/**
 * @brief Rectangular Voltage Limit
 */
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V real2param(24.0)

/**
 * @brief Circular Voltage Limit
 */
#define MCS_MAX_CIR_SATURATION_VOLTAGE_V real2param(24.0)

/**
 * @brief Deadtime Current Deadband
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A real2param(0.2)

/**
 * @brief Deadtime Current Hysteresis
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A real2param(0.05)

/**
 * @brief Open Loop Frequency
 */
#define MCS_OPEN_LOOP_FREQ_HZ real2param(30.0)

/**
 * @brief Open Loop Frequency Slope
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S real2param(20.0)

/**
 * @brief Optional constant d-axis voltage boost for BUILD_LEVEL 1, in volts.
 */
#define MCS_OPEN_LOOP_VD_REF_V real2param(0.0)

/**
 * @brief BUILD_LEVEL 1 q-axis voltage at MCS_OPEN_LOOP_FREQ_HZ. Runtime command is ramped proportionally with frequency (V/f).
 */
#define MCS_OPEN_LOOP_VQ_REF_V real2param(6.0)

/**
 * @brief Closed-loop ACIM magnetizing-current reference; unlike PMSM this normally must remain nonzero.
 */
#define MCS_COMMISSIONING_ID_REF_A real2param(3.0)

/**
 * @brief High I/F startup excitation current before the observer is sufficiently observable.
 */
#define MCS_SENSORLESS_STARTUP_ID_REF_A real2param(4.0)

/**
 * @brief I/F excitation reached at the angle-handover speed.
 */
#define MCS_SENSORLESS_HANDOVER_ID_REF_A real2param(3.0)

/**
 * @brief Absolute I/F electrical frequency where startup Id begins to decrease.
 */
#define MCS_STARTUP_ID_FADE_START_HZ real2param(5.0)

/**
 * @brief Absolute I/F electrical frequency where startup Id reaches the handover value.
 */
#define MCS_STARTUP_ID_FADE_END_HZ real2param(25.0)

/**
 * @brief Commissioning Iq
 */
#define MCS_COMMISSIONING_IQ_REF_A real2param(1.0)

/**
 * @brief Commissioning Speed
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM real2param(300.0)

/**
 * @brief Magnetizing Time
 */
#define MCS_MAGNETIZING_TIME_MS real2param(300.0)

/**
 * @brief Mech Position Kp
 */
#define MCS_MECH_POSITION_KP_PU real2param(5.0)

/**
 * @brief Mech Position Ki
 */
#define MCS_MECH_POSITION_KI_PU_S real2param(1.0)

/**
 * @brief Mech Velocity Kp
 */
#define MCS_MECH_VELOCITY_KP_PU real2param(1.0)

/**
 * @brief Mech Velocity Ki
 */
#define MCS_MECH_VELOCITY_KI_PU_S real2param(10.0)

/**
 * @brief Mech Speed Limit
 */
#define MCS_MECH_SPEED_LIMIT_RPM real2param(1400.0)

/**
 * @brief Mech Speed Slope
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S real2param(1000.0)

/**
 * @brief Mech Current Limit
 */
#define MCS_MECH_CURRENT_LIMIT_A real2param(3.0)

/**
 * @brief Encoder Speed Filter
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ real2param(20.0)

/**
 * @brief Current-model correction bandwidth while the known I/F field frame owns the current loop and initializes the voltage model.
 */
#define MCS_FO_COMP_BW_HZ real2param(20.0)

/**
 * @brief Post-handover correction bandwidth used only to reject voltage-integrator DC drift; keep it far below the sensorless operating frequency.
 */
#define MCS_FO_RUN_COMP_BW_HZ real2param(0.5)

/**
 * @brief Voltage-model leaky-integrator cutoff used to block DC voltage and current offsets. Keep it below the handover frequency and account for its phase lag during tuning.
 */
#define MCS_FO_VM_LEAK_HZ real2param(5.0)

/**
 * @brief Rotor-flux PLL bandwidth. A modest value avoids speed saturation from low-speed voltage-model phase error.
 */
#define MCS_FO_ATO_BW_HZ real2param(10.0)

/**
 * @brief Flux Observer Fault Time
 */
#define MCS_FO_FAULT_TIME_MS real2param(20.0)

/**
 * @brief Multiply DC-bus PU by this target-specific modulator gain when reconstructing applied phase voltage from alpha-beta command.
 */
#ifndef MCS_FO_COMMAND_VOLTAGE_SCALE
#define MCS_FO_COMMAND_VOLTAGE_SCALE real2param(0.5)
#endif // MCS_FO_COMMAND_VOLTAGE_SCALE

/**
 * @brief I/F electrical frequency at which the flux PLL is released to acquire the voltage-model angle while the current loop remains on the I/F field angle.
 */
#define MCS_FO_ACQUIRE_FREQ_HZ real2param(10.0)

/**
 * @brief Minimum I/F electrical frequency before a qualified flux observer may take ownership of field angle. Keep this well above MCS_FO_COMP_BW_HZ so the voltage model has directional authority.
 */
#define MCS_FO_HANDOVER_FREQ_HZ real2param(25.0)

/**
 * @brief Maximum wrapped angle mismatch in electrical-turn PU (1 pu = 360 degrees).
 */
#define MCS_FO_HANDOVER_ANGLE_ERR_PU real2param(0.08)

/**
 * @brief Maximum observer-versus-I/F synchronous-speed mismatch in PU before the enforced startup synchronization is applied.
 */
#define MCS_FO_HANDOVER_SPEED_ERR_PU real2param(0.10)

/**
 * @brief Larger speed mismatch that resets handover qualification; the gap above the enter tolerance is noise hysteresis.
 */
#define MCS_FO_HANDOVER_SPEED_EXIT_ERR_PU real2param(0.15)

/**
 * @brief Continuous qualification time before the sensorless field-angle handover.
 */
#define MCS_FO_HANDOVER_DEBOUNCE_MS real2param(100.0)

/**
 * @brief Time used to decay captured angle offset and synchronously blend startup Id to closed-loop Id.
 */
#define MCS_FO_HANDOVER_TRANSITION_MS real2param(100.0)

/**
 * @brief Maximum post-handover synchronous-speed deviation from the continuing I/F reference before fallback qualification.
 */
#define MCS_FO_LOSS_SPEED_ERR_PU real2param(0.15)

/**
 * @brief Post-handover observer-loss debounce time. A confirmed loss latches I/F fallback until controllers are cleared.
 */
#define MCS_FO_LOSS_DEBOUNCE_MS real2param(20.0)

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
#define MCS_MAX_SHUTDOWN_CURRENT_A real2param(10.0)

/**
 * @brief CiA402 Enable Delay
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (0)

// Common tail code: MCS ACIM NT Common Controller Settings
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be in the range 1..4."
#endif
#if (MCS_ACIM_FEEDBACK_MODE != MCS_ACIM_FEEDBACK_SENSORED) && (MCS_ACIM_FEEDBACK_MODE != MCS_ACIM_FEEDBACK_SENSORLESS)
#error "MCS_ACIM_FEEDBACK_MODE must select sensored or sensorless feedback."
#endif
#if (MCS_FO_VOLTAGE_SOURCE != MCS_FO_VOLTAGE_FROM_COMMAND) && (MCS_FO_VOLTAGE_SOURCE != MCS_FO_VOLTAGE_FROM_MEASUREMENT)
#error "MCS_FO_VOLTAGE_SOURCE must select command reconstruction or phase-voltage measurement."
#endif

// User project tail code
/* No additional simulation-specific tail definitions. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_ACIM_NT_SIMULATE_SETTINGS_H_
