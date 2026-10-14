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
#include <sdpe_pgs_inv_gfl_common_settings.h>

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_ID "pgs_inv_gfl_stm32g431"
#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_SUITE "pgs_inv_GFL_inverter"
#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_VERSION "1.0.0"
#define PGS_INV_GFL_STM32G431_SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief Incremental control level; level 5 enables the cascaded P/Q power loop.
 *        Options: (1), (2), (3), (4), (5)
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
 * @brief Startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Control interrupt frequency.
 */
#define CONTROLLER_FREQUENCY (10e3)

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
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 5)
#error BUILD_LEVEL_must_be_between_1_and_5
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_INV_GFL_STM32G431_SETTINGS_H_
