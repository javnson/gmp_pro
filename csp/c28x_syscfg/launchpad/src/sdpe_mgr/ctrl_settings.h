/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for GMP C2000 LaunchPad Reference Application.
 * @note Portable one-ADC/one-PWM LaunchPad example with Data Link, CAN and optional ADC-to-DAC mirroring.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/mcu_board/c2000_launchpad_boostxl.h>

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
 * @brief BOOSTXL Routing.
 */

/**
 * @brief ADC SOC selected by the physical BOOSTXL pin number. DAC-shared pins 30 and 70 are intentionally excluded.
 *        Options: BOOSTXL_ADC23, BOOSTXL_ADC24, BOOSTXL_ADC25, BOOSTXL_ADC26, BOOSTXL_ADC27, BOOSTXL_ADC28, BOOSTXL_ADC29, BOOSTXL_ADC63, BOOSTXL_ADC64, BOOSTXL_ADC65, BOOSTXL_ADC66, BOOSTXL_ADC67, BOOSTXL_ADC68, BOOSTXL_ADC69
 */
#define LAUNCHPAD_AC_ADC_SOC BOOSTXL_ADC23

/**
 * @brief ADC result register paired with LAUNCHPAD_AC_ADC_SOC.
 *        Options: BOOSTXL_ADC23_RESULT_BASE, BOOSTXL_ADC24_RESULT_BASE, BOOSTXL_ADC25_RESULT_BASE, BOOSTXL_ADC26_RESULT_BASE, BOOSTXL_ADC27_RESULT_BASE, BOOSTXL_ADC28_RESULT_BASE, BOOSTXL_ADC29_RESULT_BASE, BOOSTXL_ADC63_RESULT_BASE, BOOSTXL_ADC64_RESULT_BASE, BOOSTXL_ADC65_RESULT_BASE, BOOSTXL_ADC66_RESULT_BASE, BOOSTXL_ADC67_RESULT_BASE, BOOSTXL_ADC68_RESULT_BASE, BOOSTXL_ADC69_RESULT_BASE
 */
#define LAUNCHPAD_AC_ADC_RESULT_BASE BOOSTXL_ADC23_RESULT_BASE

/**
 * @brief One PWM pair selected by the physical BOOSTXL pin pair.
 *        Options: BOOSTXL_EPWM3635_BASE, BOOSTXL_EPWM3837_BASE, BOOSTXL_EPWM4039_BASE, BOOSTXL_EPWM7675_BASE, BOOSTXL_EPWM7877_BASE, BOOSTXL_EPWM8079_BASE
 */
#define LAUNCHPAD_PWM_BASE BOOSTXL_EPWM8079_BASE

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
