/**
 * @file launchpad_board.h
 * @brief SDPE project bindings for LAUNCHXL-F28P65X GMP application bindings.
 * @note Board-specific channel choices for the portable GMP LaunchPad reference application.
 */

#ifndef _PROJECT_LAUNCHPAD_BOARD_H_
#define _PROJECT_LAUNCHPAD_BOARD_H_

#include "hardware_preset/mcu_board/launchxl_f28p65x.h"

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

#define LAUNCHXL_F28P65X_SDPE_PROJECT_ID "gmp_launchpad_f28p65x_bindings"
#define LAUNCHXL_F28P65X_SDPE_PROJECT_SUITE "c28x_syscfg_launchpad"
#define LAUNCHXL_F28P65X_SDPE_PROJECT_VERSION "0.2.0"
#define LAUNCHXL_F28P65X_SDPE_PROJECT_UPDATED_AT "2026-08-16"

//=================================================================================================
/**
 * @brief Control Channel Selection.
 */

/**
 * @brief BOOSTXL ADC input mirrored by the reference controller.
 *        Options: BOOSTXL_ADC23_SELECT, BOOSTXL_ADC24_SELECT, BOOSTXL_ADC27_SELECT, BOOSTXL_ADC42_SELECT, BOOSTXL_ADC64_SELECT, BOOSTXL_ADC67_SELECT, BOOSTXL_ADC2_SELECT, BOOSTXL_ADC25_SELECT, BOOSTXL_ADC28_SELECT, BOOSTXL_ADC63_SELECT, BOOSTXL_ADC65_SELECT, BOOSTXL_ADC68_SELECT, BOOSTXL_ADC6_SELECT, BOOSTXL_ADC26_SELECT, BOOSTXL_ADC29_SELECT, BOOSTXL_ADC46_SELECT, BOOSTXL_ADC66_SELECT, BOOSTXL_ADC69_SELECT
 */
#define LAUNCHPAD_CONTROL_ADC_SELECT BOOSTXL_ADC23_SELECT

/**
 * @brief BOOSTXL ePWM pair receiving the normalized ADC duty.
 *        Options: BOOSTXL_EPWM4039_BASE, BOOSTXL_EPWM3837_BASE, BOOSTXL_EPWM3635_BASE, BOOSTXL_EPWM8079_BASE, BOOSTXL_EPWM7877_BASE, BOOSTXL_EPWM7675_BASE
 */
#define LAUNCHPAD_CONTROL_PWM_BASE BOOSTXL_EPWM4039_BASE

/**
 * @brief DAC receiving the raw ADC code; 0U on devices without a DAC.
 *        Options: BOOSTXL_DACA_BASE, BOOSTXL_DACC_BASE
 */
#define LAUNCHPAD_CONTROL_DAC_BASE BOOSTXL_DACA_BASE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Selected LaunchPad name.
 */
#define LAUNCHPAD_BOARD_NAME LAUNCHXL_F28P65X_BOARD_NAME

/**
 * @brief Board LED used by the scheduler heartbeat.
 */
#define LAUNCHPAD_USER_LED_GPIO LAUNCHXL_F28P65X_USER_LED_GPIO

/**
 * @brief Compile-time device DAC capability.
 */
#define LAUNCHPAD_HAS_EXTERNAL_DAC LAUNCHXL_F28P65X_HAS_EXTERNAL_DAC

/**
 * @brief Classic CAN service selection.
 */
#define LAUNCHPAD_CAN_CLASSIC LAUNCHXL_F28P65X_CAN_CLASSIC

/**
 * @brief MCAN service selection.
 */
#define LAUNCHPAD_CAN_MCAN LAUNCHXL_F28P65X_CAN_MCAN

/**
 * @brief Selected CAN or MCAN base.
 */
#define LAUNCHPAD_CAN_BASE LAUNCHXL_F28P65X_CAN_BASE

//=================================================================================================
/**
 * @brief Common fallbacks: GMP C2000 LaunchPad Reference Application.
 */

//=================================================================================================
/**
 * @brief Reference Features.
 */

/**
 * @brief Mirror the normalized control ADC sample to an external DAC when the selected device provides one.
 */
#define LAUNCHPAD_ENABLE_ADC_TO_DAC

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Center-aligned switching and ADC control-interrupt frequency applied to all six BOOSTXL ePWM pairs during peripheral initialization.
 */
#define GMP_LAUNCHPAD_PWM_FREQUENCY_HZ (20000)

/**
 * @brief XDS virtual COM Data Link baud rate.
 */
#define GMP_DL_UART_BAUDRATE (115200)

/**
 * @brief Classic CAN or MCAN nominal bit rate.
 */
#define GMP_CAN_NOMINAL_BAUDRATE (1000000)

/**
 * @brief Standard identifier accepted by the reference command object.
 */
#define GMP_CAN_RX_ID (0x101)

/**
 * @brief Standard identifier used for ADC/PWM telemetry.
 */
#define GMP_CAN_TX_ID (0x201)

/**
 * @brief Non-blocking CAN telemetry period.
 */
#define GMP_CAN_TX_PERIOD_MS (100)

/**
 * @brief Scheduler period for the serial Data Link service.
 */
#define GMP_DATALINK_TASK_PERIOD_MS (2)

/**
 * @brief Independent scheduler period for CAN or MCAN service.
 */
#define GMP_CAN_TASK_PERIOD_MS (2)

/**
 * @brief Scheduler period for the user LED task.
 */
#define GMP_HEARTBEAT_TASK_PERIOD_MS (500)

/**
 * @brief Base command reserved for optional GMP PIL service.
 */
#define GMP_PIL_DL_BASE_COMMAND (16)

// User project tail code
#define GMP_LAUNCHPAD_CAT_RAW(a, b) a##b
#define GMP_LAUNCHPAD_CAT(a, b) GMP_LAUNCHPAD_CAT_RAW(a, b)
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC23_SELECT BOOSTXL_ADC23
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC23_SELECT BOOSTXL_ADC23_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC24_SELECT BOOSTXL_ADC24
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC24_SELECT BOOSTXL_ADC24_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC27_SELECT BOOSTXL_ADC27
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC27_SELECT BOOSTXL_ADC27_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC42_SELECT BOOSTXL_ADC42
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC42_SELECT BOOSTXL_ADC42_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC64_SELECT BOOSTXL_ADC64
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC64_SELECT BOOSTXL_ADC64_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC67_SELECT BOOSTXL_ADC67
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC67_SELECT BOOSTXL_ADC67_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC2_SELECT BOOSTXL_ADC2
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC2_SELECT BOOSTXL_ADC2_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC25_SELECT BOOSTXL_ADC25
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC25_SELECT BOOSTXL_ADC25_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC28_SELECT BOOSTXL_ADC28
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC28_SELECT BOOSTXL_ADC28_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC63_SELECT BOOSTXL_ADC63
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC63_SELECT BOOSTXL_ADC63_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC65_SELECT BOOSTXL_ADC65
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC65_SELECT BOOSTXL_ADC65_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC68_SELECT BOOSTXL_ADC68
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC68_SELECT BOOSTXL_ADC68_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC6_SELECT BOOSTXL_ADC6
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC6_SELECT BOOSTXL_ADC6_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC26_SELECT BOOSTXL_ADC26
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC26_SELECT BOOSTXL_ADC26_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC29_SELECT BOOSTXL_ADC29
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC29_SELECT BOOSTXL_ADC29_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC46_SELECT BOOSTXL_ADC46
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC46_SELECT BOOSTXL_ADC46_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC66_SELECT BOOSTXL_ADC66
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC66_SELECT BOOSTXL_ADC66_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC_BOOSTXL_ADC69_SELECT BOOSTXL_ADC69
#define GMP_LAUNCHPAD_ADC_RESULT_BOOSTXL_ADC69_SELECT BOOSTXL_ADC69_RESULT_BASE
#define GMP_LAUNCHPAD_ADC_SOC(selection) GMP_LAUNCHPAD_CAT(GMP_LAUNCHPAD_ADC_SOC_, selection)
#define GMP_LAUNCHPAD_ADC_RESULT(selection) GMP_LAUNCHPAD_CAT(GMP_LAUNCHPAD_ADC_RESULT_, selection)
#define LAUNCHPAD_CONTROL_ADC_SOC GMP_LAUNCHPAD_ADC_SOC(LAUNCHPAD_CONTROL_ADC_SELECT)
#define LAUNCHPAD_CONTROL_ADC_RESULT GMP_LAUNCHPAD_ADC_RESULT(LAUNCHPAD_CONTROL_ADC_SELECT)

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_LAUNCHPAD_BOARD_H_
