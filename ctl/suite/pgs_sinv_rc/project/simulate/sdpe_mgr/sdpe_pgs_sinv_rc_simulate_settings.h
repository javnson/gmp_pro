/**
 * @file sdpe_pgs_sinv_rc_simulate_settings.h
 * @brief SDPE project bindings for PGS SINV RC MATLAB/Simulink SIL.
 * @note Windows SIL platform, plant and sensing settings for all SINV commissioning models.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_SIMULATE_SETTINGS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_pgs_sinv_rc_common_settings.h>
#define SPECIFY_PC_ENVIRONMENT

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_SINV_RC_SIM_SDPE_PROJECT_ID "pgs_sinv_rc_simulate"
#define PGS_SINV_RC_SIM_SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define PGS_SINV_RC_SIM_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_SINV_RC_SIM_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief SIL Runtime.
 */

/**
 * @brief Automatically request CiA402 ENABLE_OPERATION in the simulation executable.
 */
#define SINV_SIM_AUTO_ENABLE

/**
 * @brief Simulated sensor ADC offsets are deterministic and do not require startup calibration.
 */
// #define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Use the native ASIO UDP SIL path instead of datalink PIL.
 */
// #define ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief 1 open-loop R load; 2 current-loop R load; 3 grid current loop; 4 grid power loop; 5 DC-bus rectifier loop.
 *        Options: (1), (2), (3), (4), (5)
 */
#define BUILD_LEVEL (5)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief SIL controller and PWM update frequency.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Plant switching frequency.
 */
#define SINV_PWM_FREQUENCY_HZ (20e3f)

/**
 * @brief Virtual PWM compare maximum.
 */
#define CTRL_PWM_CMP_MAX (2999)

/**
 * @brief Virtual PWM deadband counts.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief ADC resolution used by all sensor blocks.
 */
#define CTRL_ADC_RESOLUTION (12)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Rated and nominal DC-bus voltage.
 */
#define CTRL_DCBUS_VOLTAGE (60.0f)

/**
 * @brief Nominal grid/load RMS voltage.
 */
#define CTRL_GRID_VOLTAGE_RMS (24.0f)

/**
 * @brief Rated RMS AC current.
 */
#define CTRL_RATED_CURRENT_RMS (10.0f)

/**
 * @brief Peak voltage PU base.
 */
#define CTRL_VOLTAGE_BASE (34.0f)

/**
 * @brief Peak current PU base.
 */
#define CTRL_CURRENT_BASE (14.14f)

/**
 * @brief AC filter series inductance.
 */
#define CTRL_AC_INDUCTANCE (480e-6f)

/**
 * @brief AC filter series resistance.
 */
#define CTRL_AC_RESISTANCE (0.10f)

/**
 * @brief AC filter shunt capacitance.
 */
#define SINV_FILTER_CAPACITANCE_F (22e-6f)

/**
 * @brief Filter capacitor ESR.
 */
#define SINV_FILTER_CAP_ESR_OHM (0.10f)

/**
 * @brief DC-link capacitance.
 */
#define SINV_DC_CAPACITANCE_F (2200e-6f)

/**
 * @brief Resistive load for levels 1 and 2.
 */
#define SINV_RLOAD_OHM (12.0f)

/**
 * @brief DC-side load for level 5. At 60 V this draws 120 W, within the configured converter current rating.
 */
#define SINV_RECTIFIER_RLOAD_OHM (30.0f)

/**
 * @brief AC voltage sensor sensitivity in V/V.
 */
#define CTRL_AC_VOLTAGE_SENSITIVITY (0.020f)

/**
 * @brief AC voltage ADC bias.
 */
#define CTRL_AC_VOLTAGE_BIAS (1.65f)

/**
 * @brief AC current sensor sensitivity in V/A.
 */
#define CTRL_AC_CURRENT_SENSITIVITY (0.150f)

/**
 * @brief AC current ADC bias.
 */
#define CTRL_AC_CURRENT_BIAS (1.65f)

/**
 * @brief DC bus voltage sensor sensitivity in V/V.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.040f)

/**
 * @brief DC bus voltage ADC bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC bus overvoltage threshold.
 */
#define CTRL_PROT_VBUS_MAX (90.0f)

/**
 * @brief Fast AC peak-current threshold.
 */
#define CTRL_PROT_IAC_PEAK_MAX (18.0f)

/**
 * @brief Controller divergence threshold; BUILD_LEVEL 5 masks it only during passive-rectifier takeover.
 */
#define CTRL_PROT_VCTRL_MAX_PU (1.5f)

/**
 * @brief Minimum precharged DC bus accepted by startup.
 */
#define CTRL_DCBUS_READY_MIN (25.0f)

/**
 * @brief Maximum DC bus accepted by startup.
 */
#define CTRL_DCBUS_READY_MAX (90.0f)

/**
 * @brief ADC calibration timeout.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief Plant MOSFET on resistance.
 */
#define SINV_MODEL_MOSFET_RON (4.6e-3f)

/**
 * @brief Body-diode on resistance.
 */
#define SINV_MODEL_DIODE_RON (0.01f)

/**
 * @brief Body-diode forward voltage.
 */
#define SINV_MODEL_DIODE_VF (0.5f)

// User project tail code
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_SIMULATE_SETTINGS_H_
