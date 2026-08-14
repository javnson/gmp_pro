/**
 * @file ctrl_settings.h
 * @brief SDPE project bindings for MCS ACIM NT LaunchXL-F280049C.
 * @note ACIM sensored/sensorless controller and PIL bindings for LaunchXL-F280049C with BOOSTXL-3PHGANINV.
 */

#ifndef _PROJECT_CTRL_SETTINGS_H_
#define _PROJECT_CTRL_SETTINGS_H_

#include <ctl/hardware_preset/acm_motor/acm_4p24v.h>
#include <ctl/hardware_preset/inverter_3ph/ti_boostxl_3phganinv.h>
#include <ctl/hardware_preset/mcu_board/launchxl_f280049c.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* The inverter is selected as an SDPE hardware entity. */

#define LAUNCHPAD 1
#define GMP_IRIS  0
#define BOARD_SELECTION LAUNCHPAD

// Common prefix code: MCS ACIM NT Common Controller Settings
#define MCS_ACIM_FEEDBACK_SENSORED (1)
#define MCS_ACIM_FEEDBACK_SENSORLESS (2)
#define MCS_FO_VOLTAGE_FROM_COMMAND (1)
#define MCS_FO_VOLTAGE_FROM_MEASUREMENT (2)

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_ACIM_NT_F280049C_SDPE_PROJECT_ID "mcs_acim_nt_f280049c"
#define MCS_ACIM_NT_F280049C_SDPE_PROJECT_SUITE "mcs_acm_nt"
#define MCS_ACIM_NT_F280049C_SDPE_PROJECT_VERSION "0.1.0"
#define MCS_ACIM_NT_F280049C_SDPE_PROJECT_UPDATED_AT "2026-08-15"

//=================================================================================================
/**
 * @brief PIL Runtime.
 */

/**
 * @brief Run controller steps only from Data Link PIL requests. Physical ADC ISR control dispatch and physical PWM enable remain isolated while this switch is enabled.
 */
#define ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief PIL Commissioning.
 */

/**
 * @brief Target-local incremental level used for both PIL deployment and physical commissioning.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Number of directly sampled phase currents.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)

/**
 * @brief Select sensored IFOC or sensorless flux-observer feedback.
 *        Options: MCS_ACIM_FEEDBACK_SENSORED, MCS_ACIM_FEEDBACK_SENSORLESS
 */
#define MCS_ACIM_FEEDBACK_MODE MCS_ACIM_FEEDBACK_SENSORED

/**
 * @brief Select pre-dead-time command voltage or calibrated phase-voltage ADC feedback.
 *        Options: MCS_FO_VOLTAGE_FROM_COMMAND, MCS_FO_VOLTAGE_FROM_MEASUREMENT
 */
#define MCS_FO_VOLTAGE_SOURCE MCS_FO_VOLTAGE_FROM_COMMAND

//=================================================================================================
/**
 * @brief Board GPIO.
 */

/**
 * @brief Gate-driver enable GPIO.
 *        Options: ENABLE_GATE, RESET_GATE, MONITOR_IO
 */
#define PWM_ENABLE_PORT ENABLE_GATE

/**
 * @brief Gate-driver reset GPIO.
 *        Options: ENABLE_GATE, RESET_GATE, MONITOR_IO
 */
#define PWM_RESET_PORT RESET_GATE

/**
 * @brief System status LED.
 *        Options: LED_R, LED_G
 */
#define SYSTEM_LED LED_R

/**
 * @brief Controller-running LED.
 *        Options: LED_R, LED_G
 */
#define CONTROLLER_LED LED_G

//=================================================================================================
/**
 * @brief PWM Channel.
 */

/**
 * @brief U-phase PWM pair.
 *        Options: BOOSTXL_J4_PWM1_BASE, BOOSTXL_J4_PWM2_BASE, BOOSTXL_J4_PWM3_BASE, BOOSTXL_J8_PWM1_BASE, BOOSTXL_J8_PWM2_BASE, BOOSTXL_J8_PWM3_BASE
 */
#define PHASE_U_BASE BOOSTXL_J4_PWM1_BASE

/**
 * @brief V-phase PWM pair.
 *        Options: BOOSTXL_J4_PWM1_BASE, BOOSTXL_J4_PWM2_BASE, BOOSTXL_J4_PWM3_BASE, BOOSTXL_J8_PWM1_BASE, BOOSTXL_J8_PWM2_BASE, BOOSTXL_J8_PWM3_BASE
 */
#define PHASE_V_BASE BOOSTXL_J4_PWM2_BASE

/**
 * @brief W-phase PWM pair.
 *        Options: BOOSTXL_J4_PWM1_BASE, BOOSTXL_J4_PWM2_BASE, BOOSTXL_J4_PWM3_BASE, BOOSTXL_J8_PWM1_BASE, BOOSTXL_J8_PWM2_BASE, BOOSTXL_J8_PWM3_BASE
 */
#define PHASE_W_BASE BOOSTXL_J4_PWM3_BASE

//=================================================================================================
/**
 * @brief Encoder.
 */

/**
 * @brief Rotor encoder connector.
 *        Options: EQEP1_J12_BASE, EQEP2_J13_BASE
 */
#define EQEP_Encoder_BASE EQEP2_J13_BASE

//=================================================================================================
/**
 * @brief ADC DC Bus.
 */

/**
 * @brief DC-bus voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_VBUS BOOSTXL_J3_AIN1

/**
 * @brief DC-bus voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE BOOSTXL_J3_AIN1_RESULT_BASE

/**
 * @brief DC-bus current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IBUS BOOSTXL_J3_AIN8

/**
 * @brief DC-bus current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IBUS_RESULT_BASE BOOSTXL_J3_AIN8_RESULT_BASE

//=================================================================================================
/**
 * @brief ADC Phase Current.
 */

/**
 * @brief U-phase current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IU BOOSTXL_J3_AIN5

/**
 * @brief U-phase current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IU_RESULT_BASE BOOSTXL_J3_AIN5_RESULT_BASE

/**
 * @brief V-phase current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IV BOOSTXL_J3_AIN6

/**
 * @brief V-phase current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IV_RESULT_BASE BOOSTXL_J3_AIN6_RESULT_BASE

/**
 * @brief W-phase current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IW BOOSTXL_J3_AIN7

/**
 * @brief W-phase current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IW_RESULT_BASE BOOSTXL_J3_AIN7_RESULT_BASE

//=================================================================================================
/**
 * @brief ADC Phase Voltage.
 */

/**
 * @brief U-phase voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_UU BOOSTXL_J3_AIN2

/**
 * @brief U-phase voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_UU_RESULT_BASE BOOSTXL_J3_AIN2_RESULT_BASE

/**
 * @brief V-phase voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_UV BOOSTXL_J3_AIN3

/**
 * @brief V-phase voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_UV_RESULT_BASE BOOSTXL_J3_AIN3_RESULT_BASE

/**
 * @brief W-phase voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_UW BOOSTXL_J3_AIN4

/**
 * @brief W-phase voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_UW_RESULT_BASE BOOSTXL_J3_AIN4_RESULT_BASE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Base Data Link command allocated to the PIL service.
 */
#define GMP_PIL_DL_BASE_COMMAND (16)

/**
 * @brief UART baud rate shared by the F280049C Data Link transport and the host PIL bridge. The standard 115200 baud rate avoids the XDS110 high-baud small-packet aggregation behavior.
 */
#define GMP_DL_UART_BAUDRATE (115200)

/**
 * @brief IPv4 host used by the local Simulink PIL bridge.
 */
#define GMP_PIL_UDP_HOST "127.0.0.1"

/**
 * @brief UDP port on which the bridge receives the Simulink packed input datagram.
 */
#define GMP_PIL_BRIDGE_UDP_LISTEN_PORT (12501)

/**
 * @brief UDP port on which Simulink receives the packed controller output datagram.
 */
#define GMP_PIL_MATLAB_UDP_LISTEN_PORT (12500)

/**
 * @brief Command transmit port configured by the standard GMP Simulink block.
 */
#define GMP_PIL_MATLAB_COMMAND_TX_PORT (12502)

/**
 * @brief Command receive port configured by the standard GMP Simulink block.
 */
#define GMP_PIL_MATLAB_COMMAND_RX_PORT (12503)

/**
 * @brief Receive mask enabling the seven ADC slots used by the ACIM controller; panel slots remain disabled.
 */
#define GMP_PIL_RX_MASK (127)

/**
 * @brief Transmit mask enabling three PWM compares and six controller monitor values.
 */
#define GMP_PIL_TX_MASK (4128775)

/**
 * @brief Maximum host wait for one target PIL transaction before the transport retries.
 */
#define GMP_PIL_MCU_TIMEOUT_MS (200)

/**
 * @brief Maximum idle time while waiting for the next Simulink input datagram.
 */
#define GMP_PIL_MATLAB_TIMEOUT_MS (5000)

/**
 * @brief PIL ADC slot carrying DC-bus voltage feedback.
 */
#define GMP_PIL_RX_ADC_UDC_INDEX (0)

/**
 * @brief PIL ADC slot carrying phase-U voltage feedback.
 */
#define GMP_PIL_RX_ADC_UU_INDEX (1)

/**
 * @brief PIL ADC slot carrying phase-V voltage feedback.
 */
#define GMP_PIL_RX_ADC_UV_INDEX (2)

/**
 * @brief PIL ADC slot carrying phase-W voltage feedback.
 */
#define GMP_PIL_RX_ADC_UW_INDEX (3)

/**
 * @brief PIL ADC slot carrying phase-U current feedback.
 */
#define GMP_PIL_RX_ADC_IU_INDEX (4)

/**
 * @brief PIL ADC slot carrying phase-V current feedback.
 */
#define GMP_PIL_RX_ADC_IV_INDEX (5)

/**
 * @brief PIL ADC slot carrying phase-W current feedback.
 */
#define GMP_PIL_RX_ADC_IW_INDEX (6)

/**
 * @brief Index of the encoder count inside the standard eight-channel Simulink digital vector.
 */
#define GMP_PIL_UDP_ENCODER_INDEX (0)

/**
 * @brief PIL PWM output slot for phase U.
 */
#define GMP_PIL_TX_PWM_U_INDEX (0)

/**
 * @brief PIL PWM output slot for phase V.
 */
#define GMP_PIL_TX_PWM_V_INDEX (1)

/**
 * @brief PIL PWM output slot for phase W.
 */
#define GMP_PIL_TX_PWM_W_INDEX (2)

/**
 * @brief PIL monitor slot for normalized phase-U current.
 */
#define GMP_PIL_TX_MONITOR_IU_INDEX (0)

/**
 * @brief PIL monitor slot for normalized phase-V current.
 */
#define GMP_PIL_TX_MONITOR_IV_INDEX (1)

/**
 * @brief PIL monitor slot for normalized d-axis current.
 */
#define GMP_PIL_TX_MONITOR_ID_INDEX (2)

/**
 * @brief PIL monitor slot for normalized q-axis current.
 */
#define GMP_PIL_TX_MONITOR_IQ_INDEX (3)

/**
 * @brief PIL monitor slot for normalized electrical position.
 */
#define GMP_PIL_TX_MONITOR_POSITION_INDEX (4)

/**
 * @brief PIL monitor slot for normalized mechanical speed.
 */
#define GMP_PIL_TX_MONITOR_SPEED_INDEX (5)

/**
 * @brief Number of samples stored per channel by the four-channel hardware Data Link Scope.
 */
#define GMP_DL_SCOPE_DEPTH (400)

/**
 * @brief Maximum compare count of the platform PWM peripheral at the configured controller switching frequency.
 */
#define CTRL_PWM_CMP_MAX (2500 - 1)

/**
 * @brief Dead-time count interpreted in the selected PWM peripheral clock domain.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief Platform CPU or system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (100e6)

/**
 * @brief C2000 millisecond system-tick divider derived from the CPU clock and ePWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC reference voltage used by all sensor conversions.
 */
#define CTRL_ADC_VOLTAGE_REF real2param(3.3)

/**
 * @brief Configured DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE real2param(48.0)

/**
 * @brief Phase-voltage per-unit base derived from the DC-bus base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE real2param(10.0)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (TI_BOOSTXL_3PHGANINV_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Encoder counts per mechanical revolution.
 */
#define CTRL_POS_ENC_FS (16384)

/**
 * @brief Mechanical encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS real2param(0.0)

/**
 * @brief Mechanical speed and position division factor.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief DC-bus current sensing gain. The selected inverter reports SENSOR_NONE for this path.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief Phase-current sensor sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (TI_BOOSTXL_3PHGANINV_PH_SHUNT_RESISTANCE_OHM * TI_BOOSTXL_3PHGANINV_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (TI_BOOSTXL_3PHGANINV_PH_CSA_BIAS_V)

/**
 * @brief ACIM stator resistance.
 */
#define MOTOR_PARAM_RS ACM_4P24V_RS

/**
 * @brief ACIM rotor resistance referred to stator.
 */
#define MOTOR_PARAM_RR ACM_4P24V_RR

/**
 * @brief ACIM stator leakage inductance.
 */
#define MOTOR_PARAM_L1S ACM_4P24V_L1S

/**
 * @brief ACIM rotor leakage inductance referred to stator.
 */
#define MOTOR_PARAM_L1R ACM_4P24V_L1R

/**
 * @brief ACIM magnetizing inductance.
 */
#define MOTOR_PARAM_LM ACM_4P24V_LM

/**
 * @brief ACIM pole pairs.
 */
#define MOTOR_PARAM_POLE_PAIRS ACM_4P24V_POLE_PAIRS

/**
 * @brief ACIM rated electrical frequency.
 */
#define MOTOR_PARAM_RATED_FREQUENCY ACM_4P24V_RATED_FREQUENCY

/**
 * @brief ACIM maximum mechanical speed.
 */
#define MOTOR_PARAM_MAX_SPEED ACM_4P24V_MAX_SPEED

/**
 * @brief Centered bridge command-to-phase voltage scale; command is taken before digital dead-time compare compensation.
 */
#define MCS_FO_COMMAND_VOLTAGE_SCALE real2param(0.5)

//=================================================================================================
/**
 * @brief Common fallbacks: MCS ACIM NT Common Controller Settings.
 */

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Apply ACIM-specific cross-coupling through the generic FOC feedforward port.
 */
#define ENABLE_ACIM_DECOUPLING

/**
 * @brief Enable current-polarity PWM compare compensation for physical inverter dead time. Keep disabled for an ideal SIL bridge; identify the required count and current polarity on hardware.
 */
// #ifndef ENABLE_PWM_DEADTIME_COMPENSATION
// #define ENABLE_PWM_DEADTIME_COMPENSATION
// #endif // ENABLE_PWM_DEADTIME_COMPENSATION

//=================================================================================================
/**
 * @brief Protection.
 */

/**
 * @brief Enable motor fault protection processing.
 */
// #define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning gate: 1 open-angle V/f and PWM/ADC polarity; 2 I/F dq-current loop with synthetic field angle; 3 real field-angle current loop using external ACIM_POS_CALC or ACIM_FO; 4 mechanical speed loop. Do not advance until the previous level passes the commissioning record.
 *        Options: (1), (2), (3), (4)
 */
#ifndef BUILD_LEVEL
#define BUILD_LEVEL (1)
#endif // BUILD_LEVEL

/**
 * @brief Select speed-feedback IFOC or the voltage/current flux observer.
 *        Options: MCS_ACIM_FEEDBACK_SENSORED, MCS_ACIM_FEEDBACK_SENSORLESS
 */
#ifndef MCS_ACIM_FEEDBACK_MODE
#define MCS_ACIM_FEEDBACK_MODE MCS_ACIM_FEEDBACK_SENSORED
#endif // MCS_ACIM_FEEDBACK_MODE

/**
 * @brief Select pre-dead-time controller command reconstruction or sampled phase voltage for the ACIM voltage model.
 *        Options: MCS_FO_VOLTAGE_FROM_COMMAND, MCS_FO_VOLTAGE_FROM_MEASUREMENT
 */
#ifndef MCS_FO_VOLTAGE_SOURCE
#define MCS_FO_VOLTAGE_SOURCE MCS_FO_VOLTAGE_FROM_COMMAND
#endif // MCS_FO_VOLTAGE_SOURCE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Fast-loop frequency in hertz.
 */
#define CONTROLLER_FREQUENCY real2param(20e3)

/**
 * @brief Motor Rs
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS real2param(0.329)
#endif // MOTOR_PARAM_RS

/**
 * @brief Motor Rr
 */
#ifndef MOTOR_PARAM_RR
#define MOTOR_PARAM_RR real2param(0.44)
#endif // MOTOR_PARAM_RR

/**
 * @brief Motor L1s
 */
#ifndef MOTOR_PARAM_L1S
#define MOTOR_PARAM_L1S real2param(0.0003)
#endif // MOTOR_PARAM_L1S

/**
 * @brief Motor L1r
 */
#ifndef MOTOR_PARAM_L1R
#define MOTOR_PARAM_L1R real2param(0.0005)
#endif // MOTOR_PARAM_L1R

/**
 * @brief Motor Lm
 */
#ifndef MOTOR_PARAM_LM
#define MOTOR_PARAM_LM real2param(0.0012)
#endif // MOTOR_PARAM_LM

/**
 * @brief Motor Pole Pairs
 */
#ifndef MOTOR_PARAM_POLE_PAIRS
#define MOTOR_PARAM_POLE_PAIRS (2)
#endif // MOTOR_PARAM_POLE_PAIRS

/**
 * @brief Motor Rated Frequency
 */
#ifndef MOTOR_PARAM_RATED_FREQUENCY
#define MOTOR_PARAM_RATED_FREQUENCY real2param(50.0)
#endif // MOTOR_PARAM_RATED_FREQUENCY

/**
 * @brief Motor Maximum Speed
 */
#ifndef MOTOR_PARAM_MAX_SPEED
#define MOTOR_PARAM_MAX_SPEED real2param(1450.0)
#endif // MOTOR_PARAM_MAX_SPEED

/**
 * @brief Synchronous Speed Base
 */
#define CTRL_SPEED_RPM_BASE real2param(1500.0)

/**
 * @brief Nominal DC Bus Voltage
 */
#define MCS_NOMINAL_DC_BUS_VOLTAGE_V real2param(48.0)

/**
 * @brief Rectangular Voltage Limit
 */
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V real2param(24.0)

/**
 * @brief Circular Voltage Limit
 */
#define MCS_MAX_CIR_SATURATION_VOLTAGE_V real2param(24.0)

/**
 * @brief Deadtime Current Deadband
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A real2param(0.2)

/**
 * @brief Deadtime Current Hysteresis
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A real2param(0.05)

/**
 * @brief Open Loop Frequency
 */
#define MCS_OPEN_LOOP_FREQ_HZ real2param(30.0)

/**
 * @brief Open Loop Frequency Slope
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S real2param(20.0)

/**
 * @brief Optional constant d-axis voltage boost for BUILD_LEVEL 1, in volts.
 */
#define MCS_OPEN_LOOP_VD_REF_V real2param(0.0)

/**
 * @brief BUILD_LEVEL 1 q-axis voltage at MCS_OPEN_LOOP_FREQ_HZ. Runtime command is ramped proportionally with frequency (V/f).
 */
#define MCS_OPEN_LOOP_VQ_REF_V real2param(6.0)

/**
 * @brief Closed-loop ACIM magnetizing-current reference; unlike PMSM this normally must remain nonzero.
 */
#define MCS_COMMISSIONING_ID_REF_A real2param(3.0)

/**
 * @brief High I/F startup excitation current before the observer is sufficiently observable.
 */
#define MCS_SENSORLESS_STARTUP_ID_REF_A real2param(4.0)

/**
 * @brief I/F excitation reached at the angle-handover speed.
 */
#define MCS_SENSORLESS_HANDOVER_ID_REF_A real2param(3.0)

/**
 * @brief Absolute I/F electrical frequency where startup Id begins to decrease.
 */
#define MCS_STARTUP_ID_FADE_START_HZ real2param(5.0)

/**
 * @brief Absolute I/F electrical frequency where startup Id reaches the handover value.
 */
#define MCS_STARTUP_ID_FADE_END_HZ real2param(25.0)

/**
 * @brief Commissioning Iq
 */
#define MCS_COMMISSIONING_IQ_REF_A real2param(1.0)

/**
 * @brief Commissioning Speed
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM real2param(300.0)

/**
 * @brief Magnetizing Time
 */
#define MCS_MAGNETIZING_TIME_MS real2param(300.0)

/**
 * @brief Mech Position Kp
 */
#define MCS_MECH_POSITION_KP_PU real2param(5.0)

/**
 * @brief Mech Position Ki
 */
#define MCS_MECH_POSITION_KI_PU_S real2param(1.0)

/**
 * @brief Mech Velocity Kp
 */
#define MCS_MECH_VELOCITY_KP_PU real2param(1.0)

/**
 * @brief Mech Velocity Ki
 */
#define MCS_MECH_VELOCITY_KI_PU_S real2param(10.0)

/**
 * @brief Mech Speed Limit
 */
#define MCS_MECH_SPEED_LIMIT_RPM real2param(1400.0)

/**
 * @brief Mech Speed Slope
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S real2param(1000.0)

/**
 * @brief Mech Current Limit
 */
#define MCS_MECH_CURRENT_LIMIT_A real2param(3.0)

/**
 * @brief Encoder Speed Filter
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ real2param(20.0)

/**
 * @brief Current-model correction bandwidth while the known I/F field frame owns the current loop and initializes the voltage model.
 */
#define MCS_FO_COMP_BW_HZ real2param(20.0)

/**
 * @brief Post-handover correction bandwidth used only to reject voltage-integrator DC drift; keep it far below the sensorless operating frequency.
 */
#define MCS_FO_RUN_COMP_BW_HZ real2param(0.5)

/**
 * @brief Voltage-model leaky-integrator cutoff used to block DC voltage and current offsets. Keep it below the handover frequency and account for its phase lag during tuning.
 */
#define MCS_FO_VM_LEAK_HZ real2param(5.0)

/**
 * @brief Rotor-flux PLL bandwidth. A modest value avoids speed saturation from low-speed voltage-model phase error.
 */
#define MCS_FO_ATO_BW_HZ real2param(10.0)

/**
 * @brief Flux Observer Fault Time
 */
#define MCS_FO_FAULT_TIME_MS real2param(20.0)

/**
 * @brief Multiply DC-bus PU by this target-specific modulator gain when reconstructing applied phase voltage from alpha-beta command.
 */
#ifndef MCS_FO_COMMAND_VOLTAGE_SCALE
#define MCS_FO_COMMAND_VOLTAGE_SCALE real2param(0.5)
#endif // MCS_FO_COMMAND_VOLTAGE_SCALE

/**
 * @brief I/F electrical frequency at which the flux PLL is released to acquire the voltage-model angle while the current loop remains on the I/F field angle.
 */
#define MCS_FO_ACQUIRE_FREQ_HZ real2param(10.0)

/**
 * @brief Minimum I/F electrical frequency before a qualified flux observer may take ownership of field angle. Keep this well above MCS_FO_COMP_BW_HZ so the voltage model has directional authority.
 */
#define MCS_FO_HANDOVER_FREQ_HZ real2param(25.0)

/**
 * @brief Maximum wrapped angle mismatch in electrical-turn PU (1 pu = 360 degrees).
 */
#define MCS_FO_HANDOVER_ANGLE_ERR_PU real2param(0.08)

/**
 * @brief Maximum observer-versus-I/F synchronous-speed mismatch in PU before the enforced startup synchronization is applied.
 */
#define MCS_FO_HANDOVER_SPEED_ERR_PU real2param(0.10)

/**
 * @brief Larger speed mismatch that resets handover qualification; the gap above the enter tolerance is noise hysteresis.
 */
#define MCS_FO_HANDOVER_SPEED_EXIT_ERR_PU real2param(0.15)

/**
 * @brief Continuous qualification time before the sensorless field-angle handover.
 */
#define MCS_FO_HANDOVER_DEBOUNCE_MS real2param(100.0)

/**
 * @brief Time used to decay captured angle offset and synchronously blend startup Id to closed-loop Id.
 */
#define MCS_FO_HANDOVER_TRANSITION_MS real2param(100.0)

/**
 * @brief Maximum post-handover synchronous-speed deviation from the continuing I/F reference before fallback qualification.
 */
#define MCS_FO_LOSS_SPEED_ERR_PU real2param(0.15)

/**
 * @brief Post-handover observer-loss debounce time. A confirmed loss latches I/F fallback until controllers are cleared.
 */
#define MCS_FO_LOSS_DEBOUNCE_MS real2param(20.0)

/**
 * @brief Maximum DC Bus Voltage
 */
#define MCS_MAX_DC_BUS_VOLTAGE_V (MCS_NOMINAL_DC_BUS_VOLTAGE_V * 1.2f)

/**
 * @brief Minimum DC Bus Voltage
 */
#define MCS_MIN_DC_BUS_VOLTAGE_V (MCS_NOMINAL_DC_BUS_VOLTAGE_V * 0.2f)

/**
 * @brief Maximum Shutdown Current
 */
#define MCS_MAX_SHUTDOWN_CURRENT_A real2param(10.0)

/**
 * @brief CiA402 Enable Delay
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (0)

//=================================================================================================
/**
 * @brief Common fallbacks: GMP Suite PIL Common Transport.
 */

//=================================================================================================
/**
 * @brief PIL Runtime.
 */

/**
 * @brief Run control steps only from Data Link PIL transactions while physical control dispatch and power-stage enable remain isolated.
 */
// #ifndef ENABLE_GMP_DL_PIL_SIM
// #define ENABLE_GMP_DL_PIL_SIM
// #endif // ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Base command followed by mask, step, status, and abort subcommands.
 */
#ifndef GMP_PIL_DL_BASE_COMMAND
#define GMP_PIL_DL_BASE_COMMAND (16)
#endif // GMP_PIL_DL_BASE_COMMAND

/**
 * @brief Portable commissioning rate used unless a hardware project overrides it.
 */
#ifndef GMP_DL_UART_BAUDRATE
#define GMP_DL_UART_BAUDRATE (115200)
#endif // GMP_DL_UART_BAUDRATE

/**
 * @brief Host running Simulink and the GMP PIL bridge.
 */
#ifndef GMP_PIL_UDP_HOST
#define GMP_PIL_UDP_HOST "127.0.0.1"
#endif // GMP_PIL_UDP_HOST

/**
 * @brief Bridge port receiving plant samples from Simulink.
 */
#ifndef GMP_PIL_BRIDGE_UDP_LISTEN_PORT
#define GMP_PIL_BRIDGE_UDP_LISTEN_PORT (12501)
#endif // GMP_PIL_BRIDGE_UDP_LISTEN_PORT

/**
 * @brief Simulink port receiving controller results from the bridge.
 */
#ifndef GMP_PIL_MATLAB_UDP_LISTEN_PORT
#define GMP_PIL_MATLAB_UDP_LISTEN_PORT (12500)
#endif // GMP_PIL_MATLAB_UDP_LISTEN_PORT

/**
 * @brief Bridge port receiving out-of-band Simulink commands.
 */
#ifndef GMP_PIL_MATLAB_COMMAND_TX_PORT
#define GMP_PIL_MATLAB_COMMAND_TX_PORT (12502)
#endif // GMP_PIL_MATLAB_COMMAND_TX_PORT

/**
 * @brief Simulink port receiving command acknowledgements.
 */
#ifndef GMP_PIL_MATLAB_COMMAND_RX_PORT
#define GMP_PIL_MATLAB_COMMAND_RX_PORT (12503)
#endif // GMP_PIL_MATLAB_COMMAND_RX_PORT

/**
 * @brief Maximum wait for one target Data Link response.
 */
#ifndef GMP_PIL_MCU_TIMEOUT_MS
#define GMP_PIL_MCU_TIMEOUT_MS (200)
#endif // GMP_PIL_MCU_TIMEOUT_MS

/**
 * @brief Maximum wait for the next Simulink plant sample.
 */
#ifndef GMP_PIL_MATLAB_TIMEOUT_MS
#define GMP_PIL_MATLAB_TIMEOUT_MS (5000)
#endif // GMP_PIL_MATLAB_TIMEOUT_MS

/**
 * @brief Digital input slot packed into the standard Data Link PIL request.
 */
#ifndef GMP_PIL_UDP_DIGITAL_INDEX
#define GMP_PIL_UDP_DIGITAL_INDEX (0)
#endif // GMP_PIL_UDP_DIGITAL_INDEX

// Common tail code: MCS ACIM NT Common Controller Settings
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be in the range 1..4."
#endif
#if (MCS_ACIM_FEEDBACK_MODE != MCS_ACIM_FEEDBACK_SENSORED) && (MCS_ACIM_FEEDBACK_MODE != MCS_ACIM_FEEDBACK_SENSORLESS)
#error "MCS_ACIM_FEEDBACK_MODE must select sensored or sensorless feedback."
#endif
#if (MCS_FO_VOLTAGE_SOURCE != MCS_FO_VOLTAGE_FROM_COMMAND) && (MCS_FO_VOLTAGE_SOURCE != MCS_FO_VOLTAGE_FROM_MEASUREMENT)
#error "MCS_FO_VOLTAGE_SOURCE must select command reconstruction or phase-voltage measurement."
#endif

// Common tail code: GMP Suite PIL Common Transport
/** Validate the shared PIL command allocation. */
#if (GMP_PIL_DL_BASE_COMMAND > 251U)
#error "GMP_PIL_DL_BASE_COMMAND must leave room for four PIL subcommands."
#endif
#if (GMP_PIL_UDP_DIGITAL_INDEX >= 8U)
#error "GMP_PIL_UDP_DIGITAL_INDEX must be in the range [0, 7]."
#endif

// User project tail code
/** Validate the target-local PIL command allocation and channel map. */
#if (GMP_PIL_DL_BASE_COMMAND > 251U)
#error "GMP_PIL_DL_BASE_COMMAND must leave room for the four PIL subcommands."
#endif
#if (GMP_PIL_RX_ADC_UDC_INDEX >= 24U) || (GMP_PIL_RX_ADC_UU_INDEX >= 24U) || (GMP_PIL_RX_ADC_UV_INDEX >= 24U) || (GMP_PIL_RX_ADC_UW_INDEX >= 24U) || (GMP_PIL_RX_ADC_IU_INDEX >= 24U) || (GMP_PIL_RX_ADC_IV_INDEX >= 24U) || (GMP_PIL_RX_ADC_IW_INDEX >= 24U)
#error "Every PIL ADC index must be in the range [0, 23]."
#endif
#if (GMP_PIL_UDP_ENCODER_INDEX >= 8U)
#error "GMP_PIL_UDP_ENCODER_INDEX must be in the range [0, 7]."
#endif
#if (GMP_PIL_TX_PWM_U_INDEX >= 8U) || (GMP_PIL_TX_PWM_V_INDEX >= 8U) || (GMP_PIL_TX_PWM_W_INDEX >= 8U)
#error "Every PIL PWM index must be in the range [0, 7]."
#endif
#if (GMP_PIL_TX_MONITOR_IU_INDEX >= 16U) || (GMP_PIL_TX_MONITOR_IV_INDEX >= 16U) || (GMP_PIL_TX_MONITOR_ID_INDEX >= 16U) || (GMP_PIL_TX_MONITOR_IQ_INDEX >= 16U) || (GMP_PIL_TX_MONITOR_POSITION_INDEX >= 16U) || (GMP_PIL_TX_MONITOR_SPEED_INDEX >= 16U)
#error "Every PIL monitor index must be in the range [0, 15]."
#endif

/**
 * @brief Convert volts to normalized phase voltage.
 */
#define VOLT_PU(_X_X_) (((_X_X_)/CTRL_VOLTAGE_BASE))

/**
 * @brief Convert amperes to normalized phase current.
 */
#define CURR_PU(_X_X_) (((_X_X_)/CTRL_CURRENT_BASE))

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_CTRL_SETTINGS_H_
