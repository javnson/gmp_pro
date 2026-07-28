/**
 * @file sdpe_pgs_inv_gfl_simulate_settings.h
 * @brief SDPE project bindings for PGS GFL Inverter Simulation.
 * @note Host simulation timing and sensing settings for the common GFL controller.
 */

#ifndef _PROJECT_SDPE_PGS_INV_GFL_SIMULATE_SETTINGS_H_
#define _PROJECT_SDPE_PGS_INV_GFL_SIMULATE_SETTINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/inverter_3ph/gmp_helios_3phganinv_lv.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
#include <sdpe_pgs_inv_gfl_common_settings.h>

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_SIM_SDPE_PROJECT_ID "pgs_inv_gfl_simulate"
#define PGS_INV_GFL_SIM_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_SIM_SDPE_PROJECT_VERSION "1.1.0"
#define PGS_INV_GFL_SIM_SDPE_PROJECT_UPDATED_AT "2026-07-28"

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief BUILD_LEVEL descriptor: 1=open-loop standalone voltage/PWM sensing check; 2=standalone d-q current loop with internal RG; 3=grid PLL plus positive/negative-sequence current control; 4=level 3 plus decoupling, active damping and lead compensation; 5=grid P/Q outer loop over level 4; 6=standalone LC capacitor-voltage outer loop using ordinary PI, final vector limiting and clamping correction over the current loop. With USING_3D_SVPWM, levels 3-6 also enable zero-sequence QPR control and a fourth neutral-leg PWM.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#define BUILD_LEVEL (6)

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
 * @brief Voltage sampling topology: 1=two line-to-line samples (Uab/Ubc), 2=two phase-to-neutral samples, 3=three phase-to-neutral samples. The supplied SIL models use mode 1.
 *        Options: (1), (2), (3)
 */
#define GFL_VOLTAGE_SAMPLE_PHASE_MODE (1)

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (50)

/**
 * @brief Simulation control step frequency.
 */
#define CONTROLLER_FREQUENCY (20e3)

/**
 * @brief Virtual PWM compare range.
 */
#define CTRL_PWM_CMP_MAX (3000)

/**
 * @brief Virtual PWM dead-band count.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief Virtual ADC reference.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE (80.0f)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current base.
 */
#define CTRL_CURRENT_BASE (10.0f)

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
#define CTRL_INVERTER_CURRENT_SENSITIVITY (0.05f)

/**
 * @brief Converter current bias.
 */
#define CTRL_INVERTER_CURRENT_BIAS (1.65f)

/**
 * @brief Converter voltage gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief Converter voltage bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (0.0f)

/**
 * @brief DC current sensitivity.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (0.02475f)

/**
 * @brief DC current bias.
 */
#define CTRL_DC_CURRENT_BIAS (1.65f)

/**
 * @brief DC voltage gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (0.02738589f)

/**
 * @brief DC voltage bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (0.0f)

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

#endif // _PROJECT_SDPE_PGS_INV_GFL_SIMULATE_SETTINGS_H_
