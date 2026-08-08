/**
 * @file sdpe_dps_clllc_simulate_settings.h
 * @brief SDPE project bindings for DPS CLLLC / DAB MATLAB Simulink SIL.
 * @note PC SIL runtime, BUILD_LEVEL and virtual advanced-PWM settings layered over the common CLLLC contract.
 */

#ifndef _PROJECT_SDPE_DPS_CLLLC_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_DPS_CLLLC_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/current_sensor/tmcs1133_b5a.h>
#include <ctl/hardware_preset/resonant_tank/dioscuri_clllc_resonant_tank.h>
#include <ctl/hardware_preset/voltage_sensor/dioscuri_voltage_sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Simulink/SIL platform bindings follow the shared CLLLC/DAB contract. */

// Common prefix code: DPS CLLLC / DAB Common Control
/* Shared CLLLC/DAB control and physical hardware contract. Floating-point physical ranges are constrained by their SDPE parameter definitions rather than non-portable preprocessor arithmetic. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_CLLLC_SIM_SDPE_PROJECT_ID "dps_clllc_simulate"
#define DPS_CLLLC_SIM_SDPE_PROJECT_SUITE "dps_clllc"
#define DPS_CLLLC_SIM_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_CLLLC_SIM_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Simulation.
 */

/**
 * @brief Enable PC/SIL-specific commissioning behavior.
 */
#define SPECIFY_PC_ENVIRONMENT

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
 * @brief Requirement bindings.
 */

/**
 * @brief Virtual ePWM clock.
 */
#define CLLLC_TIMER_CLOCK_HZ (100000000.0f)

/**
 * @brief PWM periods per ADC/control update.
 */
#define CLLLC_PWM_CYCLES_PER_CONTROL (2)

/**
 * @brief Nominal controller frequency.
 */
#define CONTROLLER_FREQUENCY (CLLLC_F_RESONANT_HZ / CLLLC_PWM_CYCLES_PER_CONTROL)

/**
 * @brief Virtual advanced-PWM base period.
 */
#define CLLLC_NOMINAL_PERIOD_TICKS ((uint32_t)(CLLLC_TIMER_CLOCK_HZ / CLLLC_F_RESONANT_HZ))

/**
 * @brief PC scheduler tick.
 */
#define CTRL_SYSTEM_TICK_HZ (1000)

/**
 * @brief Short SIL CiA402 transition delay.
 */
#define CTRL_STARTUP_DELAY_MS (2)

/**
 * @brief Virtual ADC reference.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Virtual ADC resolution.
 */
#define CTRL_ADC_BITS (12)

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

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (open loop), 2 (current loop), 3 (voltage loop), or 4 (parallel CC/CV)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_CLLLC_SIMULATE_SETTINGS_H_
