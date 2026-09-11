/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for GMP F28388D ControlCARD three-core reference.
 * @note CPU1 control and serial DL, CPU2 computation, and CM-owned USB/Ethernet/EtherCAT communication.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include "hardware_preset/mcu_board/controlcard_f28388d.h"

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

#define GMP_F28388D_SDPE_PROJECT_ID "gmp_controlcard_f28388d_tricore"
#define GMP_F28388D_SDPE_PROJECT_SUITE "c28x_syscfg_controlcard"
#define GMP_F28388D_SDPE_PROJECT_VERSION "0.1.0"
#define GMP_F28388D_SDPE_PROJECT_UPDATED_AT "2026-09-12"

//=================================================================================================
/**
 * @brief CM Communication.
 */

/**
 * @brief Initialize USB device mode on CM.
 */
#define GMP_F28388D_CM_ENABLE_USB

/**
 * @brief Initialize Ethernet and GMP DL on CM.
 */
#define GMP_F28388D_CM_ENABLE_ETHERNET

/**
 * @brief Initialize EtherCAT ownership and ESC memory on CM.
 */
#define GMP_F28388D_CM_ENABLE_ETHERCAT

//=================================================================================================
/**
 * @brief Control Resource Selection.
 */

/**
 * @brief Primary PWM and ADC trigger source.
 *        Options: CONTROL_EPWM1_BASE, CONTROL_EPWM2_BASE, CONTROL_EPWM3_BASE
 */
#define GMP_F28388D_PRIMARY_PWM_BASE CONTROL_EPWM1_BASE

/**
 * @brief Primary feedback input.
 *        Options: FEEDBACK_ADCA_SOC0, FEEDBACK_ADCA_SOC1, FEEDBACK_ADCA_SOC2, FEEDBACK_ADCC_SOC0, FEEDBACK_ADCC_SOC1, FEEDBACK_ADCC_SOC2
 */
#define GMP_F28388D_PRIMARY_ADC_SOC FEEDBACK_ADCA_SOC0

/**
 * @brief Analog debug output.
 *        Options: DACA_BASE, DACB_BASE, DACC_BASE
 */
#define GMP_F28388D_DEBUG_DAC_BASE DACA_BASE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Selected ControlCARD.
 */
#define GMP_F28388D_BOARD_NAME CONTROLCARD_F28388D_BOARD_NAME

/**
 * @brief Shared CPU1/CPU2 clock contract.
 */
#define GMP_F28388D_CPU_CLOCK_HZ CONTROLCARD_F28388D_CPU_CLOCK_HZ

/**
 * @brief All three cores use the system-u16 Data Link model.
 */
#define GMP_F28388D_DATA_UNIT_BITS CONTROLCARD_F28388D_DATA_UNIT_BITS

/**
 * @brief CPU1 XDS virtual COM port.
 */
#define GMP_F28388D_SCI_BASE CONTROLCARD_F28388D_SERIAL_BASE

/**
 * @brief Serial GMP DL baud rate.
 */
#define GMP_F28388D_SCI_BAUDRATE CONTROLCARD_F28388D_SERIAL_BAUDRATE

/**
 * @brief Three-phase complementary control PWM.
 */
#define GMP_F28388D_PWM_HZ CONTROLCARD_F28388D_PWM_FREQUENCY_HZ

/**
 * @brief ABZ capture interface.
 */
#define GMP_F28388D_QEP_BASE CONTROLCARD_F28388D_QEP_BASE

/**
 * @brief Number of fixed feedback channels.
 */
#define GMP_F28388D_ADC_CHANNEL_COUNT CONTROLCARD_F28388D_ADC_CHANNEL_COUNT

/**
 * @brief CM owns USB at runtime.
 */
#define GMP_F28388D_CM_HAS_USB CONTROLCARD_F28388D_HAS_USB

/**
 * @brief CM owns Ethernet at runtime.
 */
#define GMP_F28388D_CM_HAS_ETHERNET CONTROLCARD_F28388D_HAS_ETHERNET

/**
 * @brief CM owns EtherCAT at runtime.
 */
#define GMP_F28388D_CM_HAS_ETHERCAT CONTROLCARD_F28388D_HAS_ETHERCAT

/**
 * @brief Static Ethernet address.
 */
#define GMP_F28388D_CM_IPV4 CONTROLCARD_F28388D_ETHERNET_IPV4

/**
 * @brief TCP-u16 DL endpoint.
 */
#define GMP_F28388D_CM_TCP_PORT CONTROLCARD_F28388D_TCP_PORT

/**
 * @brief UDP-u16 DL endpoint.
 */
#define GMP_F28388D_CM_UDP_PORT CONTROLCARD_F28388D_UDP_PORT

// User project tail code
// SDPE extension point: add before_footer code in the Project Requirement Code page if needed.

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
