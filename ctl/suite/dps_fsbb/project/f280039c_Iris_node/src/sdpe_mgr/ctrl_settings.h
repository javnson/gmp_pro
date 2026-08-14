/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for DPS FSBB F280039C IRIS Node.
 * @note Four-switch buck-boost converter SDPE project requirement prepared from ctl/suite/dps_fsbb/project/f280039c_Iris_node/xplt/ctrl_settings.h.
 *       The requirement introduces the GMP LVFB 150V 2-phase board as the switching stage and sensor source, and IRIS F280039C Node as the peripheral option provider.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Project-specific includes can be added here if the migrated project needs extra headers. */

// Common prefix code: DPS FSBB Common Control
/* Shared FSBB control contract. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_FSBB_IRIS_SDPE_PROJECT_ID "dps_fsbb_f280039c_Iris_node"
#define DPS_FSBB_IRIS_SDPE_PROJECT_SUITE "dps_fsbb"
#define DPS_FSBB_IRIS_SDPE_PROJECT_VERSION "0.1.0"
#define DPS_FSBB_IRIS_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief Controller Settings.
 */

/**
 * @brief Enable ADC calibration before normal control starts.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable PIL simulation mode. This mode disables direct controller output for safe communication-based simulation.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

/**
 * @brief Invert duty-to-CMP mapping because the IRIS ePWM action qualifier sets high on up-count CMPA and low on down-count CMPA.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC 1

/**
 * @brief This macro will disable the fault request of the controller. That's dangerous.
 */
#define DISABLE_FSBB_PROTECTION_FAULT_LOGIC

//=================================================================================================
/**
 * @brief Optional Sampling Channels.
 */

/**
 * @brief Enable output/load current sampling path.
 */
// #define FSBB_ENABLE_IOUT_SAMPLE

/**
 * @brief Enable input voltage sampling path.
 */
// #define FSBB_ENABLE_VIN_SAMPLE

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental debug build level. 1: modulation and hardware check; 2: current loop; 3: voltage loop.
 *        Options: (1), (2), (3)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief Board GPIO.
 */

/**
 * @brief Gate-driver enable GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_ENABLE_PORT IRIS_GPIO1

/**
 * @brief Gate-driver reset GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_RESET_PORT IRIS_GPIO3

/**
 * @brief System status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define SYSTEM_LED IRIS_LED1

//=================================================================================================
/**
 * @brief PWM Channel.
 */

/**
 * @brief Buck phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_BUCK_BASE IRIS_EPWM1_BASE

/**
 * @brief Boost phase ePWM base.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_BOOST_BASE IRIS_EPWM2_BASE

//=================================================================================================
/**
 * @brief ADC Channel.
 */

/**
 * @brief Input voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_VIN_ADC_BASE ADC_CH1_RESULT_BASE

/**
 * @brief Input voltage ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_VIN ADC_CH1

/**
 * @brief Output voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_VOUT_ADC_BASE ADC_CH2_RESULT_BASE

/**
 * @brief Output voltage ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_VOUT ADC_CH2

/**
 * @brief Inductor current ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_IL_ADC_BASE ADC_CH3_RESULT_BASE

/**
 * @brief Inductor current ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IL ADC_CH3

/**
 * @brief Input current ADC result register base, available when input-current sampling is enabled.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_IIN_ADC_BASE ADC_CH4_RESULT_BASE

/**
 * @brief Input current ADC channel index, available when input-current sampling is enabled.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IIN ADC_CH4

/**
 * @brief Output/load current ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define FSBB_IOUT_ADC_BASE ADC_CH5_RESULT_BASE

/**
 * @brief Output/load current ADC channel index.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define FSBB_IOUT ADC_CH5

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief PWM compare maximum value for the configured ePWM time base.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief PWM deadband compare count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief CPU main clock frequency.
 */
#define CTRL_SYS_FREQUENCY (120e6)

/**
 * @brief C2000 system tick divider derived from PWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

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
#define FSBB_OUTPUT_VOLTAGE_MAX real2param(72.0)

/**
 * @brief Minimum output voltage.
 */
#define FSBB_OUTPUT_VOLTAGE_MIN real2param(3.0)

/**
 * @brief Output current limit.
 */
#define FSBB_OUTPUT_CURRENT_LIM real2param(10.0)

/**
 * @brief Default voltage-loop command.
 */
#define FSBB_DEFAULT_OUTPUT_VOLTAGE real2param(24.0)

/**
 * @brief Default current limit.
 */
#define FSBB_DEFAULT_CURRENT_LIMIT real2param(5.0)

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
#define FSBB_PROTECT_IL_MAX real2param(25.0)

/**
 * @brief Negative inductor-current protection threshold.
 */
#define FSBB_PROTECT_IL_MIN real2param(-2.0)

/**
 * @brief Open-loop equivalent voltage command.
 */
#define FSBB_OPEN_LOOP_VOLTAGE_COMMAND real2param(12.0)

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
/* Backward compatibility for the historical misspelled PIL switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Project-specific compile-time validation. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 3)
#error "BUILD_LEVEL must be 1 (open loop), 2 (current loop), or 3 (voltage/current cascade)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
