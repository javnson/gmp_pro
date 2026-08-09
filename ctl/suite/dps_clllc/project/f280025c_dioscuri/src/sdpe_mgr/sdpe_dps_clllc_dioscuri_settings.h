/**
 * @file sdpe_dps_clllc_dioscuri_settings.h
 * @brief SDPE project bindings for DPS CLLLC / DAB on F280025C Dioscuri.
 * @note F280025C clock, six-pair PWM routing, ADC, UART, scheduler timer and active-low SN74LVC8T245 gate-buffer bindings.
 */

#ifndef _PROJECT_SDPE_DPS_CLLLC_DIOSCURI_SETTINGS_H_
#define _PROJECT_SDPE_DPS_CLLLC_DIOSCURI_SETTINGS_H_

#include <ctl/hardware_preset/current_sensor/tmcs1133_b5a.h>
#include <ctl/hardware_preset/mcu_board/dioscuri_f280025c.h>
#include <ctl/hardware_preset/resonant_tank/dioscuri_clllc_resonant_tank.h>
#include <ctl/hardware_preset/voltage_sensor/dioscuri_voltage_sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Dioscuri platform bindings follow the shared CLLLC/DAB contract. */

// Common prefix code: DPS CLLLC / DAB Common Control
/* Shared CLLLC/DAB control and physical hardware contract. Floating-point physical ranges are constrained by their SDPE parameter definitions rather than non-portable preprocessor arithmetic. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_ID "dps_clllc_f280025c_dioscuri"
#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_SUITE "dps_clllc"
#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_CLLLC_DIOSCURI_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Debug Interfaces.
 */

/**
 * @brief Enable communication-based PIL execution instead of direct hardware control.
 */
// #define ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Controller Mode.
 */

/**
 * @brief 1: open loop; 2: current loop; 3: voltage loop; 4: voltage/current parallel competition loop.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief CLLLC PWM Routing.
 */

/**
 * @brief Primary PRI_P / ControlPort1 bridge leg: EPWM4 on the Dioscuri schematic. Changing bridge order changes the defined positive current direction.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_PRIMARY_LEG_A_BASE DIOSCURI_PWM_PAIR4_BASE

/**
 * @brief Primary PRI_N / ControlPort2 bridge leg: EPWM7 on the Dioscuri schematic.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_PRIMARY_LEG_B_BASE DIOSCURI_PWM_PAIR6_BASE

/**
 * @brief Secondary SEC_P / ControlPort4 bridge leg: EPWM1 on the Dioscuri schematic.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_SECONDARY_LEG_A_BASE DIOSCURI_PWM_PAIR2_BASE

/**
 * @brief Secondary SEC_N / ControlPort3 bridge leg: EPWM2 on the Dioscuri schematic.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_SECONDARY_LEG_B_BASE DIOSCURI_PWM_PAIR1_BASE

//=================================================================================================
/**
 * @brief CLLLC PWM Synchronization.
 */

/**
 * @brief Master time base that emits synchronization at counter zero. It must be one of the four selected CLLLC legs.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_SYNC_MASTER_PWM_BASE DIOSCURI_PWM_PAIR1_BASE

/**
 * @brief Slave sync-input source corresponding to the selected master ePWM peripheral.
 *        Options: EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM2, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM3, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM4, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM5, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM7
 */
#define CLLLC_SYNC_IN_SOURCE EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM1

//=================================================================================================
/**
 * @brief ADC Trigger.
 */

/**
 * @brief PWM that generates ADC SOCA. Keep this consistent with the SOC trigger selected in SysConfig.
 *        Options: DIOSCURI_PWM_PAIR1_BASE, DIOSCURI_PWM_PAIR2_BASE, DIOSCURI_PWM_PAIR3_BASE, DIOSCURI_PWM_PAIR4_BASE, DIOSCURI_PWM_PAIR5_BASE, DIOSCURI_PWM_PAIR6_BASE
 */
#define CLLLC_ADC_TRIGGER_PWM_BASE DIOSCURI_PWM_PAIR1_BASE

//=================================================================================================
/**
 * @brief ADC Voltage Channel Mapping.
 */

/**
 * @brief Primary AMC1311 result register base.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_RESULT_BASE, DIOSCURI_PRIMARY_CURRENT_RESULT_BASE, DIOSCURI_SECONDARY_VOLTAGE_RESULT_BASE, DIOSCURI_SECONDARY_CURRENT_RESULT_BASE
 */
#define CLLLC_PRIMARY_V_ADC_BASE DIOSCURI_PRIMARY_VOLTAGE_RESULT_BASE

/**
 * @brief Primary AMC1311 ADC SOC.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_SOC, DIOSCURI_PRIMARY_CURRENT_SOC, DIOSCURI_SECONDARY_VOLTAGE_SOC, DIOSCURI_SECONDARY_CURRENT_SOC
 */
#define CLLLC_PRIMARY_V_ADC_SOC DIOSCURI_PRIMARY_VOLTAGE_SOC

/**
 * @brief Secondary AMC1311 result register base.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_RESULT_BASE, DIOSCURI_PRIMARY_CURRENT_RESULT_BASE, DIOSCURI_SECONDARY_VOLTAGE_RESULT_BASE, DIOSCURI_SECONDARY_CURRENT_RESULT_BASE
 */
#define CLLLC_SECONDARY_V_ADC_BASE DIOSCURI_SECONDARY_VOLTAGE_RESULT_BASE

/**
 * @brief Secondary AMC1311 ADC SOC.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_SOC, DIOSCURI_PRIMARY_CURRENT_SOC, DIOSCURI_SECONDARY_VOLTAGE_SOC, DIOSCURI_SECONDARY_CURRENT_SOC
 */
#define CLLLC_SECONDARY_V_ADC_SOC DIOSCURI_SECONDARY_VOLTAGE_SOC

//=================================================================================================
/**
 * @brief ADC Current Channel Mapping.
 */

/**
 * @brief Primary TMCS1133B5A result register base.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_RESULT_BASE, DIOSCURI_PRIMARY_CURRENT_RESULT_BASE, DIOSCURI_SECONDARY_VOLTAGE_RESULT_BASE, DIOSCURI_SECONDARY_CURRENT_RESULT_BASE
 */
#define CLLLC_PRIMARY_I_ADC_BASE DIOSCURI_PRIMARY_CURRENT_RESULT_BASE

/**
 * @brief Primary TMCS1133B5A ADC SOC.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_SOC, DIOSCURI_PRIMARY_CURRENT_SOC, DIOSCURI_SECONDARY_VOLTAGE_SOC, DIOSCURI_SECONDARY_CURRENT_SOC
 */
#define CLLLC_PRIMARY_I_ADC_SOC DIOSCURI_PRIMARY_CURRENT_SOC

/**
 * @brief Secondary TMCS1133B5A result register base.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_RESULT_BASE, DIOSCURI_PRIMARY_CURRENT_RESULT_BASE, DIOSCURI_SECONDARY_VOLTAGE_RESULT_BASE, DIOSCURI_SECONDARY_CURRENT_RESULT_BASE
 */
#define CLLLC_SECONDARY_I_ADC_BASE DIOSCURI_SECONDARY_CURRENT_RESULT_BASE

/**
 * @brief Secondary TMCS1133B5A ADC SOC.
 *        Options: DIOSCURI_PRIMARY_VOLTAGE_SOC, DIOSCURI_PRIMARY_CURRENT_SOC, DIOSCURI_SECONDARY_VOLTAGE_SOC, DIOSCURI_SECONDARY_CURRENT_SOC
 */
#define CLLLC_SECONDARY_I_ADC_SOC DIOSCURI_SECONDARY_CURRENT_SOC

//=================================================================================================
/**
 * @brief Gate Driver GPIO.
 */

/**
 * @brief GPIO connected to SN74LVC8T245 OE#.
 *        Options: DIOSCURI_GATE_ENABLE_GPIO
 */
#define CLLLC_GATE_ENABLE_GPIO DIOSCURI_GATE_ENABLE_GPIO

//=================================================================================================
/**
 * @brief System Timer.
 */

/**
 * @brief CPU timer used for the GMP system tick.
 *        Options: DIOSCURI_TICK_TIMER_BASE
 */
#define CLLLC_SCHEDULER_TIMER_BASE DIOSCURI_TICK_TIMER_BASE

//=================================================================================================
/**
 * @brief Communication Peripheral.
 */

/**
 * @brief USB serial Datalink peripheral.
 *        Options: DIOSCURI_UART_BASE
 */
#define CLLLC_UART_BASE DIOSCURI_UART_BASE

//=================================================================================================
/**
 * @brief Status GPIO.
 */

/**
 * @brief Dioscuri status LED GPIO.
 *        Options: DIOSCURI_STATUS_LED_GPIO, 41
 */
#define CLLLC_STATUS_LED_GPIO DIOSCURI_STATUS_LED_GPIO

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (100)

/**
 * @brief F280025C ePWM and CPU Timer clock.
 */
#define CLLLC_TIMER_CLOCK_HZ (100000000.0f)

/**
 * @brief PWM cycles between ADC/control interrupts.
 */
#define CLLLC_PWM_CYCLES_PER_CONTROL (2)

/**
 * @brief Nominal controller frequency used for offline tuning.
 */
#define CONTROLLER_FREQUENCY (CLLLC_F_RESONANT_HZ / CLLLC_PWM_CYCLES_PER_CONTROL)

/**
 * @brief Nominal 100 kHz period in ePWM timer ticks.
 */
#define CLLLC_NOMINAL_PERIOD_TICKS ((uint32_t)(CLLLC_TIMER_CLOCK_HZ / CLLLC_F_RESONANT_HZ))

/**
 * @brief PWM-independent scheduler tick generated by CPU Timer0.
 */
#define CTRL_SYSTEM_TICK_HZ (1000)

/**
 * @brief Minimum CiA402 delay per startup transition.
 */
#define CTRL_STARTUP_DELAY_MS (100)

/**
 * @brief F280025C ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief ADC resolution in bits.
 */
#define CTRL_ADC_BITS (12)

/**
 * @brief Low level enables the PWM buffer.
 */
#define CLLLC_GATE_ENABLE_ACTIVE_LEVEL DIOSCURI_GATE_ENABLE_LEVEL

/**
 * @brief High level disables the PWM buffer.
 */
#define CLLLC_GATE_DISABLE_LEVEL DIOSCURI_GATE_DISABLE_LEVEL

//=================================================================================================
/**
 * @brief Common fallbacks: DPS CLLLC / DAB Common Control.
 */

//=================================================================================================
/**
 * @brief Sensing.
 */

/**
 * @brief Calibrate the TMCS1133B5A zero-current bias while all PWM outputs are disabled.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Enable the five ADC slots used by the CLLLC SIL input ABI.
 */
#ifndef GMP_PIL_RX_MASK
#define GMP_PIL_RX_MASK (31)
#endif // GMP_PIL_RX_MASK

/**
 * @brief Enable eight modulation slots and seven monitor slots used by the CLLLC SIL output ABI.
 */
#ifndef GMP_PIL_TX_MASK
#define GMP_PIL_TX_MASK (8323327)
#endif // GMP_PIL_TX_MASK

/**
 * @brief Nominal tank resonant frequency.
 */
#define CLLLC_F_RESONANT_HZ DIOSCURI_CLLLC_TANK_RESONANT_FREQUENCY_HZ

/**
 * @brief Lowest allowed hybrid modulation frequency.
 */
#define CLLLC_F_MIN_HZ (75000.0f)

/**
 * @brief Highest allowed hybrid modulation frequency.
 */
#define CLLLC_F_MAX_HZ (150000.0f)

/**
 * @brief Absolute complementary-switch dead time.
 */
#define CLLLC_DEADBAND_S (200e-9f)

/**
 * @brief Maximum signed primary-to-secondary bridge phase displacement, where 1 pu is 360 degrees.
 */
#define CLLLC_MAX_PHASE_SHIFT_PU (0.25f)

/**
 * @brief Transformer magnetizing inductance.
 */
#define CLLLC_LM_H DIOSCURI_CLLLC_TANK_MAGNETIZING_INDUCTANCE_H

/**
 * @brief Primary resonant inductance.
 */
#define CLLLC_LR_PRIMARY_H DIOSCURI_CLLLC_TANK_PRIMARY_RESONANT_INDUCTANCE_H

/**
 * @brief Secondary resonant inductance.
 */
#define CLLLC_LR_SECONDARY_H DIOSCURI_CLLLC_TANK_SECONDARY_RESONANT_INDUCTANCE_H

/**
 * @brief Primary resonant capacitance.
 */
#define CLLLC_CR_PRIMARY_F DIOSCURI_CLLLC_TANK_PRIMARY_RESONANT_CAPACITANCE_F

/**
 * @brief Secondary resonant capacitance.
 */
#define CLLLC_CR_SECONDARY_F DIOSCURI_CLLLC_TANK_SECONDARY_RESONANT_CAPACITANCE_F

/**
 * @brief Secondary-to-primary turns ratio.
 */
#define CLLLC_TRANSFORMER_NS_NP DIOSCURI_CLLLC_TANK_TRANSFORMER_RATIO_NS_NP

/**
 * @brief Equivalent output capacitance.
 */
#define CLLLC_COUT_F DIOSCURI_CLLLC_TANK_OUTPUT_CAPACITANCE_F

/**
 * @brief Minimum resistive load used for conservative tuning.
 */
#define CLLLC_RLOAD_MIN_OHM DIOSCURI_CLLLC_TANK_MINIMUM_LOAD_RESISTANCE_OHM

/**
 * @brief Equivalent resonant-tank series resistance.
 */
#define CLLLC_TANK_ESR_OHM DIOSCURI_CLLLC_TANK_TANK_ESR_OHM

/**
 * @brief Voltage per-unit base.
 */
#define CTRL_VOLTAGE_BASE (120.0f)

/**
 * @brief Current per-unit base, kept within the TMCS1133B5A rated measurement range.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief AMC1311 board-level ADC sensitivity.
 */
#define CLLLC_VOLTAGE_SENSITIVITY_V_PER_V DIOSCURI_VOLTAGE_SENSOR_SENSITIVITY_V_PER_V

/**
 * @brief Unidirectional AMC1311 board-level ADC bias.
 */
#define CLLLC_VOLTAGE_BIAS_V DIOSCURI_VOLTAGE_SENSOR_BIAS_V

/**
 * @brief AMC1311 front-end full-scale voltage.
 */
#define CLLLC_VOLTAGE_SENSOR_RANGE_V DIOSCURI_VOLTAGE_SENSOR_RATED_VOLTAGE_V

/**
 * @brief TMCS1133B5A sensitivity.
 */
#define CLLLC_CURRENT_SENSITIVITY_V_PER_A TMCS1133_B5A_SENSITIVITY_V_PER_A

/**
 * @brief TMCS1133B5A zero-current output bias.
 */
#define CLLLC_CURRENT_BIAS_V TMCS1133_B5A_BIAS_V

/**
 * @brief TMCS1133B5A rated current range.
 */
#define CLLLC_CURRENT_SENSOR_RANGE_A TMCS1133_B5A_RANGE_A

/**
 * @brief Default secondary voltage reference.
 */
#define CLLLC_VOLTAGE_TARGET_PU (0.40f)

/**
 * @brief Current-loop reference and parallel current limit.
 */
#define CLLLC_CURRENT_TARGET_PU (0.50f)

/**
 * @brief Requested current-loop bandwidth, constrained by CLLLC auto tuning.
 */
#define CLLLC_CURRENT_LOOP_BW_HZ (5000.0f)

/**
 * @brief Requested voltage-loop bandwidth, constrained below the current loop.
 */
#define CLLLC_VOLTAGE_LOOP_BW_HZ (400.0f)

/**
 * @brief Voltage-reference slew rate.
 */
#define CLLLC_VOLTAGE_SLOPE_PU_S (0.5f)

/**
 * @brief Current-reference slew rate.
 */
#define CLLLC_CURRENT_SLOPE_PU_S (1.0f)

/**
 * @brief Maximum current-sensor offset calibration time.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

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
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (open loop), 2 (current loop), 3 (voltage loop), or 4 (parallel CC/CV)."
#endif
/* PWM-resource checks live in xplt.peripheral.c, after SysConfig board.h symbols are available. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_CLLLC_DIOSCURI_SETTINGS_H_
