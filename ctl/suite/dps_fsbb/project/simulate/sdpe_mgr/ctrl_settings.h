/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for DPS FSBB MATLAB/Simulink SIL.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* MATLAB/Simulink SIL project settings. */

// Common prefix code: DPS FSBB Common Control
/* Shared FSBB control contract. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_FSBB_SIM_SDPE_PROJECT_ID "dps_fsbb_simulate"
#define DPS_FSBB_SIM_SDPE_PROJECT_SUITE "dps_fsbb"
#define DPS_FSBB_SIM_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_FSBB_SIM_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief SIL.
 */

/**
 * @brief The simulated ADC already includes the configured sensor bias.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Use the native ASIO SIL path instead of datalink PIL.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Allow ENABLE_OPERATION to advance through the complete CiA402 startup sequence.
 */
#define CIA402_CONFIG_ENABLE_SEQUENCE_SWITCH

//=================================================================================================
/**
 * @brief Sampling.
 */

/**
 * @brief Use the simulated input-voltage ADC channel.
 */
#define FSBB_ENABLE_VIN_SAMPLE

/**
 * @brief Use the simulated Boost-side output-current ADC channel.
 */
#define FSBB_ENABLE_IOUT_SAMPLE

//=================================================================================================
/**
 * @brief PWM.
 */

/**
 * @brief The Simulink phase model uses compare-proportional duty; Boost Q4 complementing is handled by xplt.
 */
// #define PWM_MODULATOR_USING_NEGATIVE_LOGIC 1

//=================================================================================================
/**
 * @brief Controller Settings.
 */

/**
 * @brief This macro will disable the fault request of the controller. That's dangerous.
 */
// #define DISABLE_FSBB_PROTECTION_FAULT_LOGIC

//=================================================================================================
/**
 * @brief Controller.
 */

/**
 * @brief 1: open loop, 2: current loop, 3: voltage/current cascade.
 *        Options: (1), (2), (3)
 */
#define BUILD_LEVEL (3)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief PWM compare maximum shared by controller and plant model.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief Symmetric PWM deadband in compare counts.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief ADC resolution used by all simulated sensor channels.
 */
#define CTRL_ADC_RESOLUTION (12)

/**
 * @brief Input capacitor ESR used by the plant.
 */
#define FSBB_PARAM_CIN_ESR real2param(0.1)

/**
 * @brief Buck-side input-current sensor sensitivity in V/A.
 */
#define CTRL_FSBB_IIN_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief Buck-side input-current sensor bias in V.
 */
#define CTRL_FSBB_IIN_BIAS GMP_LVFB_CURRENT_BIAS_V

/**
 * @brief BSC093N15NS5 nominal on resistance used by the plant.
 */
#define FSBB_MODEL_MOSFET_RON real2param(9.3e-3)

/**
 * @brief Plant-model snubber resistance.
 */
#define FSBB_MODEL_SNUBBER_R real2param(1e5)

/**
 * @brief Large plant-model snubber capacitance approximating an open capacitive branch.
 */
#define FSBB_MODEL_SNUBBER_C real2param(1e12)

/**
 * @brief MOSFET body-diode forward voltage.
 */
#define FSBB_MODEL_DIODE_VF real2param(0.8)

/**
 * @brief MOSFET body-diode on resistance.
 */
#define FSBB_MODEL_DIODE_RON real2param(1e-3)

/**
 * @brief Ideal-switch series inductance.
 */
#define FSBB_MODEL_SWITCH_LON real2param(0)

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

// Common tail code: DPS FSBB Common Control
/* FSBB common extension point. */

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 3)
#error "BUILD_LEVEL must be 1, 2, or 3."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
