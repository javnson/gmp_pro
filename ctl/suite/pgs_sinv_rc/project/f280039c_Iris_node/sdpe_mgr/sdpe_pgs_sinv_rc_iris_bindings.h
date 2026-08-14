/**
 * @file sdpe_pgs_sinv_rc_iris_bindings.h
 * @brief SDPE project bindings for PGS SINV RC F280039C IRIS Node.
 * @note Single-phase inverter project requirement prepared from xplt/ctrl_settings.h.
 *       This requirement introduces the GMP LVFB 150V 2-phase inverter board as the switching stage and the GMP Harmonia 3-phase LC filter as the grid filter. IRIS F280039C is included as the peripheral option provider.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
// SDPE extension point: add after_extern_open code in the Project Requirement Code page if needed.

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define SDPE_PROJECT_ID "pgs_sinv_rc_iris_node"
#define SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define SDPE_PROJECT_VERSION "0.2.0"
#define SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Controller Features.
 */

/**
 * @brief Enable Discrete PID controller anti-saturation algorithm.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable the dead zone compensation mechanism of the modulator
 */
// #define ENABLE_DEADBAND_COMP

//=================================================================================================
/**
 * @brief Sensing and Calibration.
 */

/**
 * @brief Enable ADC calibration.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

//=================================================================================================
/**
 * @brief Diagnostics and Simulation.
 */

/**
 * @brief Enable PIL simulation function. This macro disables controller output.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable CiA402 debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief Control Mode.
 */

/**
 * @brief Single-phase converter commissioning level.
 *        BUILD_LEVEL 1: open-loop sinusoidal H-bridge voltage on an isolated resistive load; validates ADC polarity, PWM mapping and the power stage without a current loop.
 *        BUILD_LEVEL 2: closed AC-current loop on an isolated resistive load; validates QPR tracking, grid-voltage feedforward and optional FDRC.
 *        BUILD_LEVEL 3: grid-connected signed P/Q command through the current loop; positive P exports power and negative P rectifies.
 *        BUILD_LEVEL 4: grid-connected measured-active-power outer loop feeding the current loop.
 *        BUILD_LEVEL 5: active-front-end rectifier with a DC-bus voltage outer loop; takeover is initialized from the measured passive-rectifier power.
 *        Options: (1), (2), (3), (4), (5)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use negative PWM modulator logic.
 *        Options: (0), (1)
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (0)

//=================================================================================================
/**
 * @brief PWM Channel Mapping.
 */

/**
 * @brief PWM base for inverter phase L.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_L_BASE IRIS_EPWM3_BASE

/**
 * @brief PWM base for inverter phase N.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_N_BASE IRIS_EPWM4_BASE

//=================================================================================================
/**
 * @brief Gate Driver GPIO.
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

//=================================================================================================
/**
 * @brief Status GPIO.
 */

/**
 * @brief System status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define SYSTEM_LED IRIS_LED1

/**
 * @brief Controller status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define CONTROLLER_LED IRIS_LED2

//=================================================================================================
/**
 * @brief AC Current Sensing.
 */

/**
 * @brief AC current ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IAC_RESULT_BASE ADC_CH1_RESULT_BASE

/**
 * @brief AC current ADC channel.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IAC ADC_CH1

//=================================================================================================
/**
 * @brief AC Voltage Sensing.
 */

/**
 * @brief AC voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_VAC_RESULT_BASE ADC_CH2_RESULT_BASE

/**
 * @brief AC voltage ADC channel.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_VAC ADC_CH2

//=================================================================================================
/**
 * @brief DC Bus Sensing.
 */

/**
 * @brief DC bus voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE ADC_CH3_RESULT_BASE

/**
 * @brief DC bus voltage ADC channel.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_VBUS ADC_CH3

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief Controller ISR frequency.
 */
#define CONTROLLER_FREQUENCY (20e3)

/**
 * @brief PWM compare maximum.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief PWM deadband compare value.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief System tick divider derived from PWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (120000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC voltage reference.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Rated DC bus voltage.
 */
#define CTRL_DCBUS_VOLTAGE (60.0f)

/**
 * @brief Rated AC grid/load RMS voltage.
 */
#define CTRL_GRID_VOLTAGE_RMS (24.0f)

/**
 * @brief Rated AC output RMS current.
 */
#define CTRL_RATED_CURRENT_RMS (10.0f)

/**
 * @brief Voltage per-unit base, using peak value.
 */
#define CTRL_VOLTAGE_BASE (34.0f)

/**
 * @brief Current per-unit base, using peak value.
 */
#define CTRL_CURRENT_BASE (14.14f)

/**
 * @brief Nominal AC grid/fundamental frequency in Hz.
 */
#define CTRL_GRID_FREQUENCY (50.0f)

/**
 * @brief Total AC-side filter/grid inductance in H.
 */
#define CTRL_AC_INDUCTANCE (0.0015f)

/**
 * @brief Total AC-side series resistance in Ohm.
 */
#define CTRL_AC_RESISTANCE (0.05f)

/**
 * @brief Minimum PLL voltage magnitude used by P/Q reference division.
 */
#define CTRL_GRID_VMIN_PU (0.1f)

/**
 * @brief Maximum hardware DC bus voltage from the LVFB inverter board.
 */
#define CTRL_MAX_HW_VOLTAGE GMP_LVFB_VBUS_MAX_V

/**
 * @brief Maximum continuous RMS hardware current from the LVFB inverter board.
 */
#define CTRL_MAX_HW_CURRENT GMP_LVFB_CURRENT_MAX_RMS_A

/**
 * @brief Project DC bus over-voltage protection threshold.
 */
#define CTRL_PROT_VBUS_MAX (100.0f)

/**
 * @brief Fast AC peak-current trip threshold in A.
 */
#define CTRL_PROT_IAC_PEAK_MAX (CTRL_MAX_HW_CURRENT * 0.9f * 1.41421356f)

/**
 * @brief Maximum unsaturated modulation command before controller-divergence trip.
 */
#define CTRL_PROT_VCTRL_MAX_PU (1.5f)

/**
 * @brief Minimum physical DC-bus voltage accepted by the startup state machine.
 */
#define CTRL_DCBUS_READY_MIN (CTRL_DCBUS_VOLTAGE * 0.8f)

/**
 * @brief Maximum physical DC-bus voltage accepted by the startup state machine.
 */
#define CTRL_DCBUS_READY_MAX (CTRL_PROT_VBUS_MAX)

/**
 * @brief Single-phase PLL proportional gain.
 */
#define CTRL_PLL_KP (2.0f)

/**
 * @brief Single-phase PLL integral time constant in seconds.
 */
#define CTRL_PLL_TI (0.02f)

/**
 * @brief PLL q-axis error low-pass cutoff in Hz.
 */
#define CTRL_PLL_LPF_FC (20.0f)

/**
 * @brief SPLL close-loop convergence criterion.
 */
#define CTRL_SPLL_EPSILON ((real2ctrl(0.005)))

/**
 * @brief DC-bus outer-loop proportional gain.
 */
#define SINV_DC_BUS_LOOP_KP (0.8f)

/**
 * @brief DC-bus outer-loop integral gain per second.
 */
#define SINV_DC_BUS_LOOP_KI (12.0f)

/**
 * @brief Symmetric outer-loop active-power command limit.
 */
#define SINV_OUTER_LOOP_POWER_LIMIT_PU (0.65f)

/**
 * @brief Power and DC-bus outer-loop execution frequency.
 */
#define SINV_OUTER_LOOP_FREQUENCY_HZ (1000.0f)

/**
 * @brief Current polarity deadband for PWM dead-time compensation.
 */
#define CTRL_CURRENT_DB_PU (0.01f)

/**
 * @brief ADC calibration timeout in ms.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief Active-power command slew limit in PU/s.
 */
#define CTRL_P_SLEW_PU_S (10.0f)

/**
 * @brief Reactive-power command slew limit in PU/s.
 */
#define CTRL_Q_SLEW_PU_S (20.0f)

/**
 * @brief Active-power outer-loop proportional gain.
 */
#define SINV_POWER_LOOP_KP (0.6f)

/**
 * @brief Active-power outer-loop integral gain per second.
 */
#define SINV_POWER_LOOP_KI (8.0f)

/**
 * @brief Peak current-reference limit in per unit.
 */
#define CTRL_CURRENT_LIMIT_PU (1.5f)

/**
 * @brief Measured active/reactive power low-pass cutoff in Hz.
 */
#define CTRL_PQ_LPF_FC (200.0f)

/**
 * @brief Minimum fundamental frequency tracked by the repetitive controller in Hz.
 */
#define CTRL_FDRC_MIN_FREQ (45.0f)

/**
 * @brief Repetitive-control learning gain.
 */
#define SINV_FDRC_LEARNING_GAIN (0.10f)

/**
 * @brief FDRC robustness-filter cutoff frequency.
 */
#define SINV_FDRC_Q_FILTER_HZ (1000.0f)

/**
 * @brief Plant-delay compensation in controller samples.
 */
#define SINV_FDRC_LEAD_STEPS (3.0f)

/**
 * @brief Current-error threshold above which RC learning is frozen.
 */
#define SINV_FDRC_FREEZE_ERROR_PU (0.05f)

/**
 * @brief Settling time before repetitive control starts learning.
 */
#define SINV_FDRC_ENABLE_DELAY_MS (300)

/**
 * @brief Startup delay in ms.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief DC bus voltage sensing gain from the LVFB inverter voltage sensor.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY GMP_LVFB_VOLTAGE_SENSITIVITY

/**
 * @brief DC bus voltage sensing ADC bias from the LVFB inverter voltage sensor.
 */
#define CTRL_DC_VOLTAGE_BIAS GMP_LVFB_VOLTAGE_BIAS_V

/**
 * @brief AC voltage sensing gain from the grid LC filter voltage sense path.
 */
#define CTRL_AC_VOLTAGE_SENSITIVITY HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN

/**
 * @brief AC voltage sensing ADC bias from the grid LC filter.
 */
#define CTRL_AC_VOLTAGE_BIAS HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V

/**
 * @brief AC current sensing sensitivity from the LVFB inverter current sensor.
 */
#define CTRL_AC_CURRENT_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief AC current sensing ADC bias from the LVFB inverter current sensor.
 */
#define CTRL_AC_CURRENT_BIAS GMP_LVFB_CURRENT_BIAS_V

//=================================================================================================
/**
 * @brief Common fallbacks: PGS Single-Phase Inverter Common Control.
 */

//=================================================================================================
/**
 * @brief Control Features.
 */

/**
 * @brief FDRC master switch. Select this switch to enable delayed insertion of the frequency-adaptive repetitive controller; clear it to keep FDRC disabled at every BUILD_LEVEL.
 */
#define SINV_ENABLE_REPETITIVE_CONTROL

/**
 * @brief Enable grid-voltage feedforward for closed-current-loop build levels.
 */
#define SINV_ENABLE_GRID_VOLTAGE_FEEDFORWARD

//=================================================================================================
/**
 * @brief Runtime.
 */

/**
 * @brief Allow ENABLE_OPERATION to advance through the complete CiA402 startup sequence.
 */
#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief DC-bus outer-loop proportional gain.
 */
#ifndef SINV_DC_BUS_LOOP_KP
#define SINV_DC_BUS_LOOP_KP (0.8f)
#endif // SINV_DC_BUS_LOOP_KP

/**
 * @brief DC-bus outer-loop integral gain per second.
 */
#ifndef SINV_DC_BUS_LOOP_KI
#define SINV_DC_BUS_LOOP_KI (12.0f)
#endif // SINV_DC_BUS_LOOP_KI

/**
 * @brief Symmetric outer-loop active-power command limit.
 */
#ifndef SINV_OUTER_LOOP_POWER_LIMIT_PU
#define SINV_OUTER_LOOP_POWER_LIMIT_PU (0.65f)
#endif // SINV_OUTER_LOOP_POWER_LIMIT_PU

/**
 * @brief Power and DC-bus outer-loop execution frequency.
 */
#ifndef SINV_OUTER_LOOP_FREQUENCY_HZ
#define SINV_OUTER_LOOP_FREQUENCY_HZ (1000.0f)
#endif // SINV_OUTER_LOOP_FREQUENCY_HZ

/**
 * @brief Active-power outer-loop proportional gain.
 */
#ifndef SINV_POWER_LOOP_KP
#define SINV_POWER_LOOP_KP (0.6f)
#endif // SINV_POWER_LOOP_KP

/**
 * @brief Active-power outer-loop integral gain per second.
 */
#ifndef SINV_POWER_LOOP_KI
#define SINV_POWER_LOOP_KI (8.0f)
#endif // SINV_POWER_LOOP_KI

/**
 * @brief Repetitive-control learning gain.
 */
#ifndef SINV_FDRC_LEARNING_GAIN
#define SINV_FDRC_LEARNING_GAIN (0.10f)
#endif // SINV_FDRC_LEARNING_GAIN

/**
 * @brief FDRC robustness-filter cutoff frequency.
 */
#ifndef SINV_FDRC_Q_FILTER_HZ
#define SINV_FDRC_Q_FILTER_HZ (1000.0f)
#endif // SINV_FDRC_Q_FILTER_HZ

/**
 * @brief Plant-delay compensation in controller samples.
 */
#ifndef SINV_FDRC_LEAD_STEPS
#define SINV_FDRC_LEAD_STEPS (3.0f)
#endif // SINV_FDRC_LEAD_STEPS

/**
 * @brief Current-error threshold above which RC learning is frozen.
 */
#ifndef SINV_FDRC_FREEZE_ERROR_PU
#define SINV_FDRC_FREEZE_ERROR_PU (0.05f)
#endif // SINV_FDRC_FREEZE_ERROR_PU

/**
 * @brief Settling time before repetitive control starts learning.
 */
#ifndef SINV_FDRC_ENABLE_DELAY_MS
#define SINV_FDRC_ENABLE_DELAY_MS (300)
#endif // SINV_FDRC_ENABLE_DELAY_MS

/**
 * @brief Enable the three sparse ADC slots used by the single-phase inverter SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (21)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable two PWM slots and all sixteen monitor slots used by the single-phase inverter SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (4294901763)
#endif // GMP_PIL_TX_MASK

/**
 * @brief Nominal grid frequency in Hz.
 */
#ifndef CTRL_GRID_FREQUENCY
#define CTRL_GRID_FREQUENCY (50.0f)
#endif // CTRL_GRID_FREQUENCY

/**
 * @brief Minimum voltage magnitude used by the P/Q reference generator.
 */
#ifndef CTRL_GRID_VMIN_PU
#define CTRL_GRID_VMIN_PU (0.1f)
#endif // CTRL_GRID_VMIN_PU

/**
 * @brief SOGI PLL proportional gain.
 */
#ifndef CTRL_PLL_KP
#define CTRL_PLL_KP (10.0f)
#endif // CTRL_PLL_KP

/**
 * @brief SOGI PLL integral time constant in seconds.
 */
#ifndef CTRL_PLL_TI
#define CTRL_PLL_TI (0.02f)
#endif // CTRL_PLL_TI

/**
 * @brief PLL error-filter cutoff frequency in Hz.
 */
#ifndef CTRL_PLL_LPF_FC
#define CTRL_PLL_LPF_FC (20.0f)
#endif // CTRL_PLL_LPF_FC

/**
 * @brief PLL frequency-error lock threshold in PU.
 */
#ifndef CTRL_SPLL_EPSILON
#define CTRL_SPLL_EPSILON (0.005f)
#endif // CTRL_SPLL_EPSILON

/**
 * @brief Current deadband used by PWM dead-time compensation.
 */
#ifndef CTRL_CURRENT_DB_PU
#define CTRL_CURRENT_DB_PU (0.01f)
#endif // CTRL_CURRENT_DB_PU

/**
 * @brief Active-power command slew limit in PU/s.
 */
#ifndef CTRL_P_SLEW_PU_S
#define CTRL_P_SLEW_PU_S (5.0f)
#endif // CTRL_P_SLEW_PU_S

/**
 * @brief Reactive-power command slew limit in PU/s.
 */
#ifndef CTRL_Q_SLEW_PU_S
#define CTRL_Q_SLEW_PU_S (5.0f)
#endif // CTRL_Q_SLEW_PU_S

/**
 * @brief Peak current command limit in PU.
 */
#ifndef CTRL_CURRENT_LIMIT_PU
#define CTRL_CURRENT_LIMIT_PU (0.9f)
#endif // CTRL_CURRENT_LIMIT_PU

/**
 * @brief Power measurement low-pass cutoff frequency in Hz.
 */
#ifndef CTRL_PQ_LPF_FC
#define CTRL_PQ_LPF_FC (200.0f)
#endif // CTRL_PQ_LPF_FC

/**
 * @brief Minimum fundamental tracked by FDRC in Hz.
 */
#ifndef CTRL_FDRC_MIN_FREQ
#define CTRL_FDRC_MIN_FREQ (45.0f)
#endif // CTRL_FDRC_MIN_FREQ

/**
 * @brief QPR current-loop crossover target in Hz.
 */
#define SINV_CURRENT_LOOP_BANDWIDTH_HZ (600.0f)

/**
 * @brief Minimum operation-enabled transition delay.
 */
#define SINV_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief BUILD_LEVEL 1 sinusoidal H-bridge voltage amplitude.
 */
#define SINV_LEVEL1_VOLTAGE_REF_PU (0.35f)

/**
 * @brief BUILD_LEVEL 2 peak current command with a resistive load.
 */
#define SINV_LEVEL2_CURRENT_REF_PEAK_PU (0.20f)

/**
 * @brief BUILD_LEVEL 3 signed grid active-power command; positive exports power.
 */
#define SINV_LEVEL3_ACTIVE_POWER_REF_PU (0.10f)

/**
 * @brief BUILD_LEVEL 3 grid reactive-power command.
 */
#define SINV_LEVEL3_REACTIVE_POWER_REF_PU (0.0f)

/**
 * @brief BUILD_LEVEL 4 measured active-power closed-loop target.
 */
#define SINV_LEVEL4_ACTIVE_POWER_REF_PU (0.15f)

/**
 * @brief BUILD_LEVEL 5 physical DC bus voltage target.
 */
#define SINV_DC_BUS_REF_V (60.0f)

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

// Common tail code: GMP Suite PIL Common Transport
/** Validate the shared PIL command allocation. */
#if (GMP_PIL_DL_BASE_COMMAND > 251U)
#error "GMP_PIL_DL_BASE_COMMAND must leave room for four PIL subcommands."
#endif
#if (GMP_PIL_UDP_DIGITAL_INDEX >= 8U)
#error "GMP_PIL_UDP_DIGITAL_INDEX must be in the range [0, 7]."
#endif

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif
/* Compatibility with framework revisions that use the historical misspelling. */
#if defined(ENABLE_GMP_DL_PIL_SIM) && !defined(ENBALE_GMP_DL_PIL_SIM)
#define ENBALE_GMP_DL_PIL_SIM
#endif
#if defined(ENBALE_GMP_DL_PIL_SIM) && !defined(ENABLE_GMP_DL_PIL_SIM)
#define ENABLE_GMP_DL_PIL_SIM
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_
