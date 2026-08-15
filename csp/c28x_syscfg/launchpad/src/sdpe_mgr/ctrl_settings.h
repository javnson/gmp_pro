/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for GMP C2000 LaunchPad Reference Application.
 * @note Portable one-ADC/one-PWM LaunchPad example with Data Link, CAN and optional ADC-to-DAC mirroring.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

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

#define GMP_C2000_LAUNCHPAD_SDPE_PROJECT_ID "gmp_c2000_launchpad_reference"
#define GMP_C2000_LAUNCHPAD_SDPE_PROJECT_SUITE "c28x_syscfg_launchpad"
#define GMP_C2000_LAUNCHPAD_SDPE_PROJECT_VERSION "0.1.0"
#define GMP_C2000_LAUNCHPAD_SDPE_PROJECT_UPDATED_AT "2026-08-15"

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
 * @brief Scheduler period for Data Link and CAN service.
 */
#define GMP_DATALINK_TASK_PERIOD_MS (2)

/**
 * @brief Scheduler period for the user LED task.
 */
#define GMP_HEARTBEAT_TASK_PERIOD_MS (500)

/**
 * @brief Base command reserved for optional GMP PIL service.
 */
#define GMP_PIL_DL_BASE_COMMAND (16)

// User project tail code
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
