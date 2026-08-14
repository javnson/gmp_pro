/**
 * @file sdpe_pgs_inv_gfl_stm32g431_settings.h
 * @brief SDPE project bindings for PGS GFL Inverter STM32G431.
 * @note Nucleo-G431 platform timing and sensing settings for the common GFL controller.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFL_STM32G431_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFL_STM32G431_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/inverter_3ph/gmp_helios_3phganinv_lv.h>
#include <ctl/hardware_preset/mcu_board/nucleo_g431rb_motor_board.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

// Common prefix code: PGS Grid-Following Inverter Common Settings
/* Platform-independent GFL controller settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_ID "pgs_inv_gfl_stm32g431"
#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief BUILD_LEVEL descriptor: 1=open-loop standalone voltage/PWM sensing check; 2=standalone d-q current loop with internal RG; 3=grid PLL plus positive/negative-sequence current control; 4=level 3 plus decoupling, active damping and lead compensation; 5=grid P/Q outer loop over level 4; 6=standalone LC capacitor-voltage outer loop using ordinary PI, final vector limiting and clamping correction over the current loop. USING_3D_SVPWM additionally requires a mapped fourth neutral-leg PWM.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Number of directly sampled phase currents.
 *        Options: (2), (3)
 */
#define GFL_CURRENT_SAMPLE_PHASE_MODE (3)

/**
 * @brief Number of directly sampled phase voltages.
 *        Options: (2), (3)
 */
#define GFL_VOLTAGE_SAMPLE_PHASE_MODE (3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief Startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Control interrupt frequency.
 */
#define CONTROLLER_FREQUENCY real2param(10e3)

/**
 * @brief TIM compare range.
 */
#define CTRL_PWM_CMP_MAX (8500 - 1)

/**
 * @brief TIM dead-time count.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief ADC reference.
 */
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE real2param(80.0)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current base.
 */
#define CTRL_CURRENT_BASE real2param(10.0)

/**
 * @brief Harmonia inductance.
 */
#define GFL_GRID_FILTER_INDUCTANCE_H (HARMONIA_3PH_LC_FILTER_INDUCTANCE_H)

/**
 * @brief Harmonia capacitance.
 */
#define GFL_GRID_FILTER_CAPACITANCE_F (HARMONIA_3PH_LC_FILTER_CAPACITANCE_F)

/**
 * @brief Grid-current sensitivity.
 */
#define CTRL_GRID_CURRENT_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_CURRENT_SENSITIVITY_MV_A * 0.001f)

/**
 * @brief Grid-current bias.
 */
#define CTRL_GRID_CURRENT_BIAS (HARMONIA_3PH_LC_FILTER_PH_CURRENT_ZERO_BIAS_V)

/**
 * @brief Grid-voltage gain.
 */
#define CTRL_GRID_VOLTAGE_SENSITIVITY (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Grid-voltage bias.
 */
#define CTRL_GRID_VOLTAGE_BIAS (HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Converter current sensitivity.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY real2param(0.05)

/**
 * @brief Converter current bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS real2param(1.65)

/**
 * @brief Converter voltage gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY real2param(0.02738589)

/**
 * @brief Converter voltage bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS real2param(0.0)

/**
 * @brief DC current sensitivity.
 */
#define CTRL_DC_CURRENT_SENSITIVITY real2param(0.02475)

/**
 * @brief DC current bias.
 */
#define CTRL_DC_CURRENT_BIAS real2param(1.65)

/**
 * @brief DC voltage gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY real2param(0.02738589)

/**
 * @brief DC voltage bias.
 */
#define CTRL_DC_VOLTAGE_BIAS real2param(0.0)

//=================================================================================================
/**
 * @brief Common fallbacks: PGS Grid-Following Inverter Common Settings.
 */

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Enable the existing discrete PID anti-saturation path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Use DSOGI PLL instead of the default SRF PLL.
 */
// #define USING_DSOGI_PLL

/**
 * @brief Use the three-level NPC modulator instead of the two-level SPWM modulator.
 */
// #define USING_NPC_MODULATOR

/**
 * @brief Use four-leg 3D-SVPWM. This enables an independently controlled neutral-leg duty and permits zero-sequence QPR control; the selected board must provide a fourth PWM bridge leg.
 */
// #define USING_3D_SVPWM

/**
 * @brief Enable omega*C cross-coupling feed-forward in the BUILD_LEVEL 6 capacitor-voltage loop. Disable this switch to commission the voltage PI loop without decoupling.
 */
#define GFL_ENABLE_VOLTAGE_DECOUPLE

/**
 * @brief Enable circular magnitude limiting of the complete BUILD_LEVEL 6 d-q current reference.
 */
#define GFL_ENABLE_VOLTAGE_CIRCLE_LIMIT

/**
 * @brief Enable independent symmetric d/q-axis limiting of the complete BUILD_LEVEL 6 current reference.
 */
// #define GFL_ENABLE_VOLTAGE_SQUARE_LIMIT

/**
 * @brief Enable frequency-P and voltage-Q droop reference generation ahead of the BUILD_LEVEL 5 P/Q controller.
 */
#define GFL_ENABLE_PQ_DROOP

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Enable startup ADC offset calibration. Only enable while all calibrated power inputs are in a known zero state.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Capacitor-current source: direct measurement, current difference, or capacitor-voltage derivative.
 *        Options: (1), (2), (3)
 */
#define GFL_CAPACITOR_CURRENT_CALCULATE_MODE (3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Enable the seven ADC slots used by the three-phase inverter SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (127)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable four PWM slots and all sixteen monitor slots used by the GFL SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (4294901775)
#endif // GMP_PIL_TX_MASK

/**
 * @brief ADC offset calibrator filter cutoff frequency.
 */
#define GFL_ADC_CALIBRATOR_FC_HZ real2param(20.0)

/**
 * @brief ADC offset calibrator second-order filter quality factor.
 */
#define GFL_ADC_CALIBRATOR_Q real2param(0.707)

/**
 * @brief Minimum CiA402 delay before Operation Enabled.
 */
#define GFL_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief Nominal grid phase-voltage magnitude in controller per unit.
 */
#define GFL_GRID_VOLTAGE_PU real2param(0.33)

/**
 * @brief P/Q outer-loop execution frequency in hertz.
 */
#define GFL_PQ_LOOP_FREQUENCY_HZ real2param(1000.0)

/**
 * @brief Current-ISR to P/Q-loop execution divider.
 */
#define GFL_PQ_LOOP_DIVIDER ((uint32_t)(CONTROLLER_FREQUENCY / GFL_PQ_LOOP_FREQUENCY_HZ))

/**
 * @brief Active-power loop proportional gain from P error PU to d-axis current PU.
 */
#define GFL_PQ_ACTIVE_KP real2param(0.75)

/**
 * @brief Active-power loop integral gain in inverse seconds.
 */
#define GFL_PQ_ACTIVE_KI real2param(0.001)

/**
 * @brief Reactive-power loop proportional gain from Q error PU to q-axis current PU.
 */
#define GFL_PQ_REACTIVE_KP real2param(0.75)

/**
 * @brief Reactive-power loop integral gain in inverse seconds.
 */
#define GFL_PQ_REACTIVE_KI real2param(0.001)

/**
 * @brief Circular magnitude limit applied to the d/q current reference produced by the P/Q loop.
 */
#define GFL_PQ_CURRENT_LIMIT_PU real2param(1.0)

/**
 * @brief PQ droop frequency and voltage measurement low-pass cutoff.
 */
#define GFL_PQ_DROOP_LPF_HZ real2param(10.0)

/**
 * @brief Additional active-power command per hertz of frequency deficit.
 */
#define GFL_PQ_DROOP_P_GAIN_PU_PER_HZ real2param(0.10)

/**
 * @brief Additional reactive-power command per PU of voltage deficit.
 */
#define GFL_PQ_DROOP_Q_GAIN_PU_PER_V_PU real2param(0.50)

/**
 * @brief Minimum active-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_P_MIN_PU real2param(-0.80)

/**
 * @brief Maximum active-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_P_MAX_PU real2param(0.80)

/**
 * @brief Minimum reactive-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_Q_MIN_PU real2param(-0.80)

/**
 * @brief Maximum reactive-power reference after PQ droop.
 */
#define GFL_PQ_DROOP_Q_MAX_PU real2param(0.80)

/**
 * @brief BUILD_LEVEL 1 d-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VD_PU real2param(0.6)

/**
 * @brief BUILD_LEVEL 1 q-axis open-loop voltage command.
 */
#define GFL_OPEN_LOOP_VQ_PU real2param(0.6)

/**
 * @brief PLL lock-error threshold in controller per unit.
 */
#define CTRL_SPLL_EPSILON real2ctrl(0.005)

/**
 * @brief Default active-power reference. Positive power exports energy to the grid.
 */
#define GFL_ACTIVE_POWER_REF_PU real2param(0.1)

/**
 * @brief Default reactive-power reference using Q = vq*id - vd*iq.
 */
#define GFL_REACTIVE_POWER_REF_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 2 d-axis current command.
 */
#define GFL_CURRENT_LEVEL2_ID_PU real2param(0.1)

/**
 * @brief BUILD_LEVEL 2 q-axis current command.
 */
#define GFL_CURRENT_LEVEL2_IQ_PU real2param(0.1)

/**
 * @brief BUILD_LEVEL 3 grid-connected d-axis current command.
 */
#define GFL_CURRENT_LEVEL3_ID_PU real2param(0.1)

/**
 * @brief BUILD_LEVEL 3 grid-connected q-axis current command.
 */
#define GFL_CURRENT_LEVEL3_IQ_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 4 d-axis current command.
 */
#define GFL_CURRENT_LEVEL4_ID_PU real2param(0.6)

/**
 * @brief BUILD_LEVEL 4 q-axis current command.
 */
#define GFL_CURRENT_LEVEL4_IQ_PU real2param(0.6)

/**
 * @brief BUILD_LEVEL 6 positive-sequence d-axis capacitor phase-voltage reference in per unit.
 */
#define GFL_STANDALONE_VD_PU real2param(0.50)

/**
 * @brief BUILD_LEVEL 6 positive-sequence q-axis capacitor phase-voltage reference in per unit.
 */
#define GFL_STANDALONE_VQ_PU real2param(0.0)

/**
 * @brief BUILD_LEVEL 6 capacitor-voltage outer-loop bandwidth in hertz; keep it well below the inner current-loop bandwidth.
 */
#define GFL_VOLTAGE_LOOP_BW_HZ real2param(100.0)

/**
 * @brief BUILD_LEVEL 6 ordinary PI zero frequency in hertz.
 */
#define GFL_VOLTAGE_LOOP_ZERO_HZ real2param(20.0)

/**
 * @brief Circular magnitude limit for the complete BUILD_LEVEL 6 PI plus feed-forward current reference. The final limited output is returned to the ordinary PID integrators by clamping correction.
 */
#define GFL_VOLTAGE_CIRCLE_LIMIT_PU real2param(0.80)

/**
 * @brief Independent symmetric d/q-axis limit for the complete BUILD_LEVEL 6 PI plus feed-forward current reference.
 */
#define GFL_VOLTAGE_SQUARE_LIMIT_PU real2param(0.80)

/**
 * @brief Zero-sequence current QPR proportional gain used when 3D-SVPWM is enabled.
 */
#define GFL_ZERO_QPR_KP real2param(0.10)

/**
 * @brief Zero-sequence current QPR resonant gain used when 3D-SVPWM is enabled.
 */
#define GFL_ZERO_QPR_KR real2param(50.0)

/**
 * @brief Zero-sequence QPR resonant bandwidth/cutoff frequency in hertz.
 */
#define GFL_ZERO_QPR_CUTOFF_HZ real2param(5.0)

/**
 * @brief Symmetric zero-axis voltage-command limit for four-wire operation.
 */
#define GFL_ZERO_VOLTAGE_LIMIT_PU real2param(0.20)

/**
 * @brief Nominal grid frequency in hertz.
 */
#define GFL_GRID_FREQUENCY_HZ real2param(50.0)

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

// Common tail code: PGS Grid-Following Inverter Common Settings
/* Accept the historical PIL spelling while new projects use the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif
/* Four-leg hardware must explicitly acknowledge that A/B/C/N PWM and protection are mapped. */
#if defined(USING_3D_SVPWM) && !defined(SPECIFY_PC_ENVIRONMENT) && !defined(GFL_3D_SVPWM_PLATFORM_MAPPED)
#error Define_GFL_3D_SVPWM_PLATFORM_MAPPED_only_after_mapping_all_four_PWM_legs
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
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 6)
#error BUILD_LEVEL_must_be_between_1_and_6
#endif
#if defined(USING_3D_SVPWM) && defined(USING_NPC_MODULATOR)
#error USING_3D_SVPWM_and_USING_NPC_MODULATOR_are_mutually_exclusive
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_STM32G431_SETTINGS_H_
