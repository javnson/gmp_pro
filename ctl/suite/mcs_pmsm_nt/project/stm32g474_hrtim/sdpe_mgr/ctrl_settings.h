/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for MCS PMSM NT NUCLEO-G474RE HRTIM.
 * @note NUCLEO-G474RE PMSM settings with independently selectable HRTIM A-E resources for phases U/V/W and USART2 circular-DMA Datalink.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/st_x_nucleo_ihm08m1.h>
#include <ctl/hardware_preset/mcu_board/nucleo_g474re_hrtim_motor_board.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

// Common prefix code: MCS PMSM NT Common Controller Settings
/* Platform-independent settings only. Project hardware is supplied by the including project header. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_ID "mcs_pmsm_nt_stm32g474_hrtim"
#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_VERSION "1.2.0"
#define MCS_PMSM_NT_STM32G474_HRTIM_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief ADC.
 */

/**
 * @brief Directly sampled phase-current count.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (2)

//=================================================================================================
/**
 * @brief HRTIM Phase Mapping.
 */

/**
 * @brief U-phase HRTIM channel: 0=A, 1=B, 2=C, 3=D, 4=E.
 *        Options: 0, 1, 2, 3, 4
 */
#define MCS_HRTIM_PHASE_U_CHANNEL 0

/**
 * @brief V-phase HRTIM channel: 0=A, 1=B, 2=C, 3=D, 4=E.
 *        Options: 0, 1, 2, 3, 4
 */
#define MCS_HRTIM_PHASE_V_CHANNEL 1

/**
 * @brief W-phase HRTIM channel: 0=A, 1=B, 2=C, 3=D, 4=E.
 *        Options: 0, 1, 2, 3, 4
 */
#define MCS_HRTIM_PHASE_W_CHANNEL 2

//=================================================================================================
/**
 * @brief Board.
 */

/**
 * @brief Datalink UART handle.
 *        Options: (&huart2)
 */
#define MCS_UART_HANDLE (&huart2)

/**
 * @brief Datalink UART instance.
 *        Options: USART2
 */
#define MCS_UART_INSTANCE USART2

/**
 * @brief Encoder timer handle.
 *        Options: (&htim3)
 */
#define MCS_ENCODER_TIMER_HANDLE (&htim3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief Maximum compare count of the platform PWM peripheral at the configured controller switching frequency.
 */
#define CTRL_PWM_CMP_MAX (34000)

/**
 * @brief Dead-time count interpreted in the selected PWM peripheral clock domain.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief Platform CPU or system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (170e6)

/**
 * @brief ADC reference voltage used by all sensor conversions.
 */
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief Configured DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE real2param(80.0)

/**
 * @brief Phase-voltage per-unit base derived from the DC-bus base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE real2param(10.0)

/**
 * @brief Encoder counts per mechanical revolution.
 */
#define CTRL_POS_ENC_FS (10000)

/**
 * @brief Mechanical encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS real2param(0.0207000002)

/**
 * @brief Mechanical speed and position division factor.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Phase-current sensor sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (ST_X_NUCLEO_IHM08M1_PH_SHUNT_RESISTANCE_OHM * ST_X_NUCLEO_IHM08M1_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (ST_X_NUCLEO_IHM08M1_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (ST_X_NUCLEO_IHM08M1_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (ST_X_NUCLEO_IHM08M1_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-link current scaling is not consumed in the selected phase-current sampling mode.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (ST_X_NUCLEO_IHM08M1_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (ST_X_NUCLEO_IHM08M1_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (ST_X_NUCLEO_IHM08M1_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (ST_X_NUCLEO_IHM08M1_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Circular UART RX DMA buffer size.
 */
#define MCS_UART_RX_BUFFER_SIZE (64)

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_TYPE SM060R20B30MNAD_MOTOR_TYPE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_KV SM060R20B30MNAD_KV

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_EMF SM060R20B30MNAD_EMF

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_VOLTAGE SM060R20B30MNAD_RATED_VOLTAGE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_CURRENT SM060R20B30MNAD_RATED_CURRENT

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_NO_LOAD_CURRENT SM060R20B30MNAD_NO_LOAD_CURRENT

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_FREQUENCY SM060R20B30MNAD_RATED_FREQUENCY

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_SPEED SM060R20B30MNAD_MAX_SPEED

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_TORQUE SM060R20B30MNAD_MAX_TORQUE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_DC_VOLTAGE SM060R20B30MNAD_MAX_DC_VOLTAGE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
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

//=================================================================================================
/**
 * @brief Common fallbacks: GMP Suite PIL Common Transport.
 */

//=================================================================================================
/**
 * @brief PIL Runtime.
 */

/**
 * @brief Run control steps only from Data Link PIL transactions while physical control dispatch and power-stage enable remain isolated.
 */
// #ifndef ENABLE_GMP_DL_PIL_SIM
// #define ENABLE_GMP_DL_PIL_SIM
// #endif // ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Base command followed by mask, step, status, and abort subcommands.
 */
#ifndef GMP_PIL_DL_BASE_COMMAND
#define GMP_PIL_DL_BASE_COMMAND (16)
#endif // GMP_PIL_DL_BASE_COMMAND

/**
 * @brief Portable commissioning rate used unless a hardware project overrides it.
 */
#ifndef GMP_DL_UART_BAUDRATE
#define GMP_DL_UART_BAUDRATE (115200)
#endif // GMP_DL_UART_BAUDRATE

/**
 * @brief Host running Simulink and the GMP PIL bridge.
 */
#ifndef GMP_PIL_UDP_HOST
#define GMP_PIL_UDP_HOST "127.0.0.1"
#endif // GMP_PIL_UDP_HOST

/**
 * @brief Bridge port receiving plant samples from Simulink.
 */
#ifndef GMP_PIL_BRIDGE_UDP_LISTEN_PORT
#define GMP_PIL_BRIDGE_UDP_LISTEN_PORT (12501)
#endif // GMP_PIL_BRIDGE_UDP_LISTEN_PORT

/**
 * @brief Simulink port receiving controller results from the bridge.
 */
#ifndef GMP_PIL_MATLAB_UDP_LISTEN_PORT
#define GMP_PIL_MATLAB_UDP_LISTEN_PORT (12500)
#endif // GMP_PIL_MATLAB_UDP_LISTEN_PORT

/**
 * @brief Bridge port receiving out-of-band Simulink commands.
 */
#ifndef GMP_PIL_MATLAB_COMMAND_TX_PORT
#define GMP_PIL_MATLAB_COMMAND_TX_PORT (12502)
#endif // GMP_PIL_MATLAB_COMMAND_TX_PORT

/**
 * @brief Simulink port receiving command acknowledgements.
 */
#ifndef GMP_PIL_MATLAB_COMMAND_RX_PORT
#define GMP_PIL_MATLAB_COMMAND_RX_PORT (12503)
#endif // GMP_PIL_MATLAB_COMMAND_RX_PORT

/**
 * @brief Maximum wait for one target Data Link response.
 */
#ifndef GMP_PIL_MCU_TIMEOUT_MS
#define GMP_PIL_MCU_TIMEOUT_MS (200)
#endif // GMP_PIL_MCU_TIMEOUT_MS

/**
 * @brief Maximum wait for the next Simulink plant sample.
 */
#ifndef GMP_PIL_MATLAB_TIMEOUT_MS
#define GMP_PIL_MATLAB_TIMEOUT_MS (5000)
#endif // GMP_PIL_MATLAB_TIMEOUT_MS

/**
 * @brief Digital input slot packed into the standard Data Link PIL request.
 */
#ifndef GMP_PIL_UDP_DIGITAL_INDEX
#define GMP_PIL_UDP_DIGITAL_INDEX (0)
#endif // GMP_PIL_UDP_DIGITAL_INDEX

// Common tail code: MCS PMSM NT Common Controller Settings
/* Accept the historical misspelling while all source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Reject unsupported incremental build levels at preprocessing time. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

// Common tail code: GMP Suite PIL Common Transport
/** Validate the shared PIL command allocation. */
#if (GMP_PIL_DL_BASE_COMMAND > 251U)
#error "GMP_PIL_DL_BASE_COMMAND must leave room for four PIL subcommands."
#endif
#if (GMP_PIL_UDP_DIGITAL_INDEX >= 8U)
#error "GMP_PIL_UDP_DIGITAL_INDEX must be in the range [0, 7]."
#endif

// User project tail code
/* Platform-specific peripheral selection and validation. */
#if (MCS_HRTIM_PHASE_U_CHANNEL == MCS_HRTIM_PHASE_V_CHANNEL) || (MCS_HRTIM_PHASE_U_CHANNEL == MCS_HRTIM_PHASE_W_CHANNEL) || (MCS_HRTIM_PHASE_V_CHANNEL == MCS_HRTIM_PHASE_W_CHANNEL)
#error "HRTIM phase U/V/W channels must be unique."
#endif
#if (MCS_HRTIM_PHASE_U_CHANNEL > 4) || (MCS_HRTIM_PHASE_V_CHANNEL > 4) || (MCS_HRTIM_PHASE_W_CHANNEL > 4)
#error "HRTIM phase channel must be in range 0 (A) through 4 (E)."
#endif

#define MCS_HRTIM_CAT_RAW(a, b) a##b
#define MCS_HRTIM_CAT(a, b) MCS_HRTIM_CAT_RAW(a, b)
#define MCS_HRTIM_SELECT(prefix, channel) MCS_HRTIM_CAT(prefix, channel)
#define MCS_HRTIM_TIMER_INDEX_0 HRTIM_TIMERINDEX_TIMER_A
#define MCS_HRTIM_TIMER_INDEX_1 HRTIM_TIMERINDEX_TIMER_B
#define MCS_HRTIM_TIMER_INDEX_2 HRTIM_TIMERINDEX_TIMER_C
#define MCS_HRTIM_TIMER_INDEX_3 HRTIM_TIMERINDEX_TIMER_D
#define MCS_HRTIM_TIMER_INDEX_4 HRTIM_TIMERINDEX_TIMER_E
#define MCS_HRTIM_TIMER_ID_0 HRTIM_TIMERID_TIMER_A
#define MCS_HRTIM_TIMER_ID_1 HRTIM_TIMERID_TIMER_B
#define MCS_HRTIM_TIMER_ID_2 HRTIM_TIMERID_TIMER_C
#define MCS_HRTIM_TIMER_ID_3 HRTIM_TIMERID_TIMER_D
#define MCS_HRTIM_TIMER_ID_4 HRTIM_TIMERID_TIMER_E
#define MCS_HRTIM_OUTPUT1_0 HRTIM_OUTPUT_TA1
#define MCS_HRTIM_OUTPUT1_1 HRTIM_OUTPUT_TB1
#define MCS_HRTIM_OUTPUT1_2 HRTIM_OUTPUT_TC1
#define MCS_HRTIM_OUTPUT1_3 HRTIM_OUTPUT_TD1
#define MCS_HRTIM_OUTPUT1_4 HRTIM_OUTPUT_TE1
#define MCS_HRTIM_OUTPUT2_0 HRTIM_OUTPUT_TA2
#define MCS_HRTIM_OUTPUT2_1 HRTIM_OUTPUT_TB2
#define MCS_HRTIM_OUTPUT2_2 HRTIM_OUTPUT_TC2
#define MCS_HRTIM_OUTPUT2_3 HRTIM_OUTPUT_TD2
#define MCS_HRTIM_OUTPUT2_4 HRTIM_OUTPUT_TE2
#define MCS_HRTIM_ADC_UPDATE_0 HRTIM_ADCTRIGGERUPDATE_TIMER_A
#define MCS_HRTIM_ADC_UPDATE_1 HRTIM_ADCTRIGGERUPDATE_TIMER_B
#define MCS_HRTIM_ADC_UPDATE_2 HRTIM_ADCTRIGGERUPDATE_TIMER_C
#define MCS_HRTIM_ADC_UPDATE_3 HRTIM_ADCTRIGGERUPDATE_TIMER_D
#define MCS_HRTIM_ADC_UPDATE_4 HRTIM_ADCTRIGGERUPDATE_TIMER_E
#define MCS_HRTIM_ADC_EVENT_0 HRTIM_ADCTRIGGEREVENT24_TIMERA_PERIOD
#define MCS_HRTIM_ADC_EVENT_1 HRTIM_ADCTRIGGEREVENT24_TIMERB_PERIOD
#define MCS_HRTIM_ADC_EVENT_2 HRTIM_ADCTRIGGEREVENT24_TIMERC_PERIOD
#define MCS_HRTIM_ADC_EVENT_3 HRTIM_ADCTRIGGEREVENT24_TIMERD_PERIOD
#define MCS_HRTIM_ADC_EVENT_4 HRTIM_ADCTRIGGEREVENT24_TIMERE_RESET
#define MCS_HRTIM_PHASE_U_TIMER_INDEX MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_INDEX_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_TIMER_INDEX MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_INDEX_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_TIMER_INDEX MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_INDEX_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_TIMER_ID MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_ID_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_TIMER_ID MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_ID_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_TIMER_ID MCS_HRTIM_SELECT(MCS_HRTIM_TIMER_ID_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_OUTPUT1 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT1_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_OUTPUT1 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT1_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_OUTPUT1 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT1_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_OUTPUT2 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT2_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_PHASE_V_OUTPUT2 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT2_, MCS_HRTIM_PHASE_V_CHANNEL)
#define MCS_HRTIM_PHASE_W_OUTPUT2 MCS_HRTIM_SELECT(MCS_HRTIM_OUTPUT2_, MCS_HRTIM_PHASE_W_CHANNEL)
#define MCS_HRTIM_PHASE_U_OUTPUTS (MCS_HRTIM_PHASE_U_OUTPUT1 | MCS_HRTIM_PHASE_U_OUTPUT2)
#define MCS_HRTIM_PHASE_V_OUTPUTS (MCS_HRTIM_PHASE_V_OUTPUT1 | MCS_HRTIM_PHASE_V_OUTPUT2)
#define MCS_HRTIM_PHASE_W_OUTPUTS (MCS_HRTIM_PHASE_W_OUTPUT1 | MCS_HRTIM_PHASE_W_OUTPUT2)
#define MCS_HRTIM_ADC_UPDATE_SOURCE MCS_HRTIM_SELECT(MCS_HRTIM_ADC_UPDATE_, MCS_HRTIM_PHASE_U_CHANNEL)
#define MCS_HRTIM_ADC_TRIGGER_EVENT MCS_HRTIM_SELECT(MCS_HRTIM_ADC_EVENT_, MCS_HRTIM_PHASE_U_CHANNEL)

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
