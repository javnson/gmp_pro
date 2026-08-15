/**
 * @file launchpad_board.h
 * @brief SDPE project bindings for LAUNCHXL-F2800137 GMP Board Bindings.
 */

#ifndef _PROJECT_LAUNCHPAD_BOARD_H_
#define _PROJECT_LAUNCHPAD_BOARD_H_

#include "hardware_preset/mcu_board/launchxl_f2800137.h"

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

#define LAUNCHXL_F2800137_SDPE_PROJECT_ID "launchxl_f2800137_bindings"
#define LAUNCHXL_F2800137_SDPE_PROJECT_SUITE "c28x_syscfg_launchpad"
#define LAUNCHXL_F2800137_SDPE_PROJECT_VERSION "0.1.0"
#define LAUNCHXL_F2800137_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Generated LAUNCHPAD_BOARD_NAME board binding.
 */
#define LAUNCHPAD_BOARD_NAME LAUNCHXL_F2800137_BOARD_NAME

/**
 * @brief Generated LAUNCHPAD_CONTROL_ISR_HZ board binding.
 */
#define LAUNCHPAD_CONTROL_ISR_HZ LAUNCHXL_F2800137_CONTROL_ISR_HZ

/**
 * @brief BOOSTXL physical ADC SOC alias.
 */
#define LAUNCHPAD_CONTROL_ADC_SOC BOOSTXL_DUAL_SITE_CONTROL_ADC_SOC

/**
 * @brief BOOSTXL ADC result base.
 */
#define LAUNCHPAD_CONTROL_ADC_RESULT BOOSTXL_DUAL_SITE_CONTROL_ADC_RESULT

/**
 * @brief BOOSTXL PWM output base.
 */
#define LAUNCHPAD_CONTROL_PWM_BASE BOOSTXL_DUAL_SITE_CONTROL_PWM_BASE

/**
 * @brief Generated LAUNCHPAD_USER_LED_GPIO board binding.
 */
#define LAUNCHPAD_USER_LED_GPIO LAUNCHXL_F2800137_USER_LED_GPIO

/**
 * @brief Generated LAUNCHPAD_HAS_EXTERNAL_DAC board binding.
 */
#define LAUNCHPAD_HAS_EXTERNAL_DAC LAUNCHXL_F2800137_HAS_EXTERNAL_DAC

/**
 * @brief Generated LAUNCHPAD_CONTROL_DAC_BASE board binding.
 */
#define LAUNCHPAD_CONTROL_DAC_BASE LAUNCHXL_F2800137_CONTROL_DAC_BASE

/**
 * @brief Generated LAUNCHPAD_CAN_CLASSIC board binding.
 */
#define LAUNCHPAD_CAN_CLASSIC LAUNCHXL_F2800137_CAN_CLASSIC

/**
 * @brief Generated LAUNCHPAD_CAN_MCAN board binding.
 */
#define LAUNCHPAD_CAN_MCAN LAUNCHXL_F2800137_CAN_MCAN

/**
 * @brief Generated LAUNCHPAD_CAN_BASE board binding.
 */
#define LAUNCHPAD_CAN_BASE LAUNCHXL_F2800137_CAN_BASE

// User project tail code
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_LAUNCHPAD_BOARD_H_
