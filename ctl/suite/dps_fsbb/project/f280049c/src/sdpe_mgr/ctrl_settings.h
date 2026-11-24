/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for DPS FSBB LaunchXL-F280049C.
 * @note FSBB binding for LVFB_100_2PH and LaunchXL-F280049C.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

// Common prefix code: DPS FSBB Common Control
/* Shared FSBB control contract. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_FSBB_F280049C_SDPE_PROJECT_ID "dps_fsbb_f280049c"
#define DPS_FSBB_F280049C_SDPE_PROJECT_SUITE "dps_fsbb"
#define DPS_FSBB_F280049C_SDPE_PROJECT_VERSION "1.0.0"

//=================================================================================================
/**
 * @brief PIL Runtime.
 */

/**
 * @brief Execute control only on PIL STEP requests and hold both physical bridges in Trip.
 */
// #define ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Controller Settings.
 */

/**
 * @brief Use explicit CiA402 commands from the FSBB CAN mailbox.
 */
#define CIA402_CONFIG_DISABLE_CONTROL_WORD_DEFAULT

/**
 * @brief Advance through the full startup sequence on an explicit Enable Operation command.
 */
#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Enable optional isolated Vin feedback on BoosterPack pin 24 (ADCINB0).
 */
// #define FSBB_ENABLE_VIN_SAMPLE

//=================================================================================================
/**
 * @brief Controller Settings.
 */

/**
 * @brief 1: open loop. 2: current loop. 3: cascaded voltage and current.
 *        Options: (1), (2), (3)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Samples per scope channel.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief ePWM TBPRD at 100 MHz in up/down mode.
 */
#define CTRL_PWM_CMP_MAX (2500)

/**
 * @brief 600 ns of MCU deadband at 100 MHz.
 */
#define CTRL_PWM_DEADBAND_CMP (60)

/**
 * @brief 20 kHz ADC ISR to 1 kHz scheduler.
 */
#define DSP_C2000_DSP_TIME_DIV (20)

/**
 * @brief Set to the measured bench input when Vin ADC is disabled.
 */
#define FSBB_LAUNCHXL_FIXED_VIN (12.0f)

/**
 * @brief Initial open-loop command in volts.
 */
#define FSBB_OPEN_LOOP_VOLTAGE_COMMAND (3.0f)

/**
 * @brief Commissioning output overvoltage limit in volts.
 */
#define FSBB_OUTPUT_VOLTAGE_MAX (15.0f)

/**
 * @brief Commissioning positive inductor current trip in amps.
 */
#define FSBB_PROTECT_IL_MAX (2.0f)

/**
 * @brief Commissioning negative inductor current trip in amps.
 */
#define FSBB_PROTECT_IL_MIN (-2.0f)

/**
 * @brief Controller current reference ceiling in amps.
 */
#define FSBB_OUTPUT_CURRENT_LIM (1.0f)

/**
 * @brief Level 3 default command in volts.
 */
#define FSBB_DEFAULT_OUTPUT_VOLTAGE (6.0f)

/**
 * @brief Level 2/3 default current command in amps.
 */
#define FSBB_DEFAULT_CURRENT_LIMIT (0.5f)

//=================================================================================================
/**
 * @brief Common fallbacks: DPS FSBB Common Control.
 */

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Enable the four ADC slots used by the FSBB SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (15)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable two PWM slots and eight monitor slots used by the FSBB SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (16711683)
#endif // GMP_PIL_TX_MASK

/**
 * @brief FSBB switching frequency.
 */
#define PWM_FREQ real2param(20e3)

/**
 * @brief SIL controller sample frequency.
 */
#define CONTROLLER_FREQUENCY real2param(20e3)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief Voltage per-unit base.
 */
#define CTRL_VOLTAGE_BASE real2param(34.0)

/**
 * @brief Current per-unit base.
 */
#define CTRL_CURRENT_BASE real2param(14.14)

/**
 * @brief Minimum load resistance.
 */
#define FSBB_PARAM_RLOAD_MIN real2param(20.0)

/**
 * @brief Input capacitance.
 */
#define FSBB_PARAM_CIN real2param(440e-6)

/**
 * @brief Output capacitance.
 */
#define FSBB_PARAM_COUT real2param(440e-6)

/**
 * @brief Output capacitor ESR.
 */
#define FSBB_PARAM_COUT_ESR real2param(0.1)

/**
 * @brief Main FSBB inductance.
 */
#define FSBB_PARAM_L real2param(1.5e-3)

/**
 * @brief Main inductor ESR.
 */
#define FSBB_PARAM_L_ESR real2param(0.05)

/**
 * @brief Input-voltage sensor sensitivity in V/V.
 */
#define CTRL_FSBB_VIN_SENSITIVITY GMP_LVFB_VOLTAGE_SENSITIVITY

/**
 * @brief Input-voltage sensor bias in V.
 */
#define CTRL_FSBB_VIN_BIAS GMP_LVFB_VOLTAGE_BIAS_V

/**
 * @brief Output-voltage sensor sensitivity in V/V.
 */
#define CTRL_FSBB_VOUT_SENSITIVITY GMP_LVFB_VOLTAGE_SENSITIVITY

/**
 * @brief Output-voltage sensor bias in V.
 */
#define CTRL_FSBB_VOUT_BIAS GMP_LVFB_VOLTAGE_BIAS_V

/**
 * @brief Boost-side output-current sensor sensitivity in V/A.
 */
#define CTRL_FSBB_IOUT_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief Boost-side output-current sensor bias in V.
 */
#define CTRL_FSBB_IOUT_BIAS GMP_LVFB_CURRENT_BIAS_V

/**
 * @brief Inductor-current sensor sensitivity in V/A.
 */
#define CTRL_FSBB_IL_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief Inductor-current sensor bias in V.
 */
#define CTRL_FSBB_IL_BIAS GMP_LVFB_CURRENT_BIAS_V

/**
 * @brief Maximum input voltage.
 */
#define FSBB_INPUT_VOLTAGE_MAX real2param(60.0)

/**
 * @brief Minimum input voltage.
 */
#define FSBB_INPUT_VOLTAGE_MIN real2param(12.0)

/**
 * @brief Nominal model source voltage.
 */
#define FSBB_INPUT_VOLTAGE_NOMINAL real2param(24.0)

/**
 * @brief Maximum output voltage.
 */
#ifndef FSBB_OUTPUT_VOLTAGE_MAX
#define FSBB_OUTPUT_VOLTAGE_MAX real2param(72.0)
#endif // FSBB_OUTPUT_VOLTAGE_MAX

/**
 * @brief Minimum output voltage.
 */
#define FSBB_OUTPUT_VOLTAGE_MIN real2param(3.0)

/**
 * @brief Output current limit.
 */
#ifndef FSBB_OUTPUT_CURRENT_LIM
#define FSBB_OUTPUT_CURRENT_LIM real2param(10.0)
#endif // FSBB_OUTPUT_CURRENT_LIM

/**
 * @brief Default voltage-loop command.
 */
#ifndef FSBB_DEFAULT_OUTPUT_VOLTAGE
#define FSBB_DEFAULT_OUTPUT_VOLTAGE real2param(24.0)
#endif // FSBB_DEFAULT_OUTPUT_VOLTAGE

/**
 * @brief Default current limit.
 */
#ifndef FSBB_DEFAULT_CURRENT_LIMIT
#define FSBB_DEFAULT_CURRENT_LIMIT real2param(5.0)
#endif // FSBB_DEFAULT_CURRENT_LIMIT

/**
 * @brief Maximum leg duty.
 */
#define FSBB_DUTY_MAX real2param(0.95)

/**
 * @brief Minimum leg duty.
 */
#define FSBB_DUTY_MIN real2param(0.05)

/**
 * @brief Positive inductor-current protection threshold.
 */
#ifndef FSBB_PROTECT_IL_MAX
#define FSBB_PROTECT_IL_MAX real2param(25.0)
#endif // FSBB_PROTECT_IL_MAX

/**
 * @brief Negative inductor-current protection threshold.
 */
#ifndef FSBB_PROTECT_IL_MIN
#define FSBB_PROTECT_IL_MIN real2param(-2.0)
#endif // FSBB_PROTECT_IL_MIN

/**
 * @brief Open-loop equivalent voltage command.
 */
#ifndef FSBB_OPEN_LOOP_VOLTAGE_COMMAND
#define FSBB_OPEN_LOOP_VOLTAGE_COMMAND real2param(12.0)
#endif // FSBB_OPEN_LOOP_VOLTAGE_COMMAND

/**
 * @brief Current-loop crossover frequency.
 */
#define FSBB_CURRENT_LOOP_BANDWIDTH real2param(800.0)

/**
 * @brief Voltage-loop crossover frequency.
 */
#define FSBB_VOLTAGE_LOOP_BANDWIDTH real2param(40.0)

/**
 * @brief Buck-to-transition boundary.
 */
#define FSBB_TRANSITION_RATIO_LOW real2param(0.90)

/**
 * @brief Transition-to-boost boundary.
 */
#define FSBB_TRANSITION_RATIO_HIGH real2param(1.10)

/**
 * @brief Compatibility setting used by the suite framework.
 */
#define CTRL_SPLL_EPSILON real2param(0.005)

/**
 * @brief ADC calibration timeout.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief Voltage command ramp in pu/s.
 */
#define FSBB_VOLTAGE_RAMP_PU_S real2param(1.0)

/**
 * @brief Current command ramp in pu/s.
 */
#define FSBB_CURRENT_RAMP_PU_S real2param(1.0)

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
 * @brief Digital input slot packed into the standard Data Link PIL request.
 */
#ifndef GMP_PIL_UDP_DIGITAL_INDEX
#define GMP_PIL_UDP_DIGITAL_INDEX (0)
#endif // GMP_PIL_UDP_DIGITAL_INDEX

/**
 * @brief Base command followed by mask, step, status, and abort subcommands.
 */
#ifndef GMP_PIL_DL_BASE_COMMAND
#define GMP_PIL_DL_BASE_COMMAND (16)
#endif // GMP_PIL_DL_BASE_COMMAND

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

// Common tail code: DPS FSBB Common Control
/* FSBB common extension point. */

// Common tail code: GMP Suite PIL Common Transport
/** Validate the shared PIL command allocation. */
#if (GMP_PIL_DL_BASE_COMMAND > 251U)
#error "GMP_PIL_DL_BASE_COMMAND must leave room for four PIL subcommands."
#endif
#if (GMP_PIL_UDP_DIGITAL_INDEX >= 8U)
#error "GMP_PIL_UDP_DIGITAL_INDEX must be in the range [0, 7]."
#endif

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 3)
#error "BUILD_LEVEL must be 1, 2, or 3."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
