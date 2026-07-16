/**
 * @file sdpe_dps_fsbb_simulate_settings.h
 * @brief SDPE project bindings for DPS FSBB MATLAB/Simulink SIL.
 */

#ifndef _PROJECT_SDPE_DPS_FSBB_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_DPS_FSBB_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_dps_fsbb_common_settings.h>
/* MATLAB/Simulink SIL project settings. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define DPS_FSBB_SIM_SDPE_PROJECT_ID "dps_fsbb_simulate"
#define DPS_FSBB_SIM_SDPE_PROJECT_SUITE "dps_fsbb"
#define DPS_FSBB_SIM_SDPE_PROJECT_VERSION "1.0.0"
#define DPS_FSBB_SIM_SDPE_PROJECT_UPDATED_AT "2026-07-16"

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
#define FSBB_PARAM_CIN_ESR (0.1f)

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
#define FSBB_MODEL_MOSFET_RON (9.3e-3f)

/**
 * @brief Plant-model snubber resistance.
 */
#define FSBB_MODEL_SNUBBER_R (1e5f)

/**
 * @brief Large plant-model snubber capacitance approximating an open capacitive branch.
 */
#define FSBB_MODEL_SNUBBER_C (1e12f)

/**
 * @brief MOSFET body-diode forward voltage.
 */
#define FSBB_MODEL_DIODE_VF (0.8f)

/**
 * @brief MOSFET body-diode on resistance.
 */
#define FSBB_MODEL_DIODE_RON (1e-3f)

/**
 * @brief Ideal-switch series inductance.
 */
#define FSBB_MODEL_SWITCH_LON (0f)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 3)
#error "BUILD_LEVEL must be 1, 2, or 3."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DPS_FSBB_SIMULATE_SETTINGS_H_
