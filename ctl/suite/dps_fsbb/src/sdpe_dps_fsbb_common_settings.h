/**
 * @file sdpe_dps_fsbb_common_settings.h
 * @brief SDPE project bindings for DPS FSBB Common Control.
 * @note Platform-independent FSBB topology, sensing, protection and control contract shared by all hardware and SIL targets.
 */

#ifndef _PROJECT_SDPE_DPS_FSBB_COMMON_SETTINGS_H_
#define _PROJECT_SDPE_DPS_FSBB_COMMON_SETTINGS_H_

#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Shared FSBB control contract. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_FSBB_COMMON_SDPE_PROJECT_ID "dps_fsbb_common"
#define DPS_FSBB_COMMON_SDPE_PROJECT_SUITE "dps_fsbb"
#define DPS_FSBB_COMMON_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_FSBB_COMMON_SDPE_PROJECT_UPDATED_AT "2026-07-16"

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief FSBB switching frequency.
 */
#define PWM_FREQ (20e3f)

/**
 * @brief SIL controller sample frequency.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Voltage per-unit base.
 */
#define CTRL_VOLTAGE_BASE (34.0f)

/**
 * @brief Current per-unit base.
 */
#define CTRL_CURRENT_BASE (14.14f)

/**
 * @brief Minimum load resistance.
 */
#define FSBB_PARAM_RLOAD_MIN (20.0f)

/**
 * @brief Input capacitance.
 */
#define FSBB_PARAM_CIN (440e-6f)

/**
 * @brief Output capacitance.
 */
#define FSBB_PARAM_COUT (440e-6f)

/**
 * @brief Output capacitor ESR.
 */
#define FSBB_PARAM_COUT_ESR (0.1f)

/**
 * @brief Main FSBB inductance.
 */
#define FSBB_PARAM_L (1.5e-3f)

/**
 * @brief Main inductor ESR.
 */
#define FSBB_PARAM_L_ESR (0.05f)

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
#define FSBB_INPUT_VOLTAGE_MAX (60.0f)

/**
 * @brief Minimum input voltage.
 */
#define FSBB_INPUT_VOLTAGE_MIN (12.0f)

/**
 * @brief Nominal model source voltage.
 */
#define FSBB_INPUT_VOLTAGE_NOMINAL (24.0f)

/**
 * @brief Maximum output voltage.
 */
#define FSBB_OUTPUT_VOLTAGE_MAX (72.0f)

/**
 * @brief Minimum output voltage.
 */
#define FSBB_OUTPUT_VOLTAGE_MIN (3.0f)

/**
 * @brief Output current limit.
 */
#define FSBB_OUTPUT_CURRENT_LIM (10.0f)

/**
 * @brief Default voltage-loop command.
 */
#define FSBB_DEFAULT_OUTPUT_VOLTAGE (24.0f)

/**
 * @brief Default current limit.
 */
#define FSBB_DEFAULT_CURRENT_LIMIT (5.0f)

/**
 * @brief Maximum leg duty.
 */
#define FSBB_DUTY_MAX (0.95f)

/**
 * @brief Minimum leg duty.
 */
#define FSBB_DUTY_MIN (0.05f)

/**
 * @brief Positive inductor-current protection threshold.
 */
#define FSBB_PROTECT_IL_MAX (25.0f)

/**
 * @brief Negative inductor-current protection threshold.
 */
#define FSBB_PROTECT_IL_MIN (-2.0f)

/**
 * @brief Open-loop equivalent voltage command.
 */
#define FSBB_OPEN_LOOP_VOLTAGE_COMMAND (12.0f)

/**
 * @brief Current-loop crossover frequency.
 */
#define FSBB_CURRENT_LOOP_BANDWIDTH (800.0f)

/**
 * @brief Voltage-loop crossover frequency.
 */
#define FSBB_VOLTAGE_LOOP_BANDWIDTH (40.0f)

/**
 * @brief Buck-to-transition boundary.
 */
#define FSBB_TRANSITION_RATIO_LOW (0.90f)

/**
 * @brief Transition-to-boost boundary.
 */
#define FSBB_TRANSITION_RATIO_HIGH (1.10f)

/**
 * @brief Compatibility setting used by the suite framework.
 */
#define CTRL_SPLL_EPSILON (0.005f)

/**
 * @brief ADC calibration timeout.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief Voltage command ramp in pu/s.
 */
#define FSBB_VOLTAGE_RAMP_PU_S (1.0f)

/**
 * @brief Current command ramp in pu/s.
 */
#define FSBB_CURRENT_RAMP_PU_S (1.0f)

// User project tail code
/* FSBB common extension point. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_FSBB_COMMON_SETTINGS_H_
