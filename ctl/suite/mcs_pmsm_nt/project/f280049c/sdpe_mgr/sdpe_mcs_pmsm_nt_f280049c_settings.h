/**
 * @file sdpe_mcs_pmsm_nt_f280049c_settings.h
 * @brief SDPE project bindings for MCS PMSM NT LaunchXL-F280049C.
 * @note PMSM sensored-control settings and project bindings for the LaunchXL-F280049C. Board resources remain physical BOOSTXL connector resources and are assigned application roles here.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_NT_F280049C_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_NT_F280049C_SETTINGS_H_

#include <ctl/hardware_preset/mcu_board/launchxl_f280049c.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* The BOOSTXL inverter remains a legacy preset until it receives an SDPE entity. */
#define BOOSTXL_3PHGANINV_IS_DEFAULT_PARAM
#include <ctl/component/hardware_preset/inverter_3ph/GMP_3PH_2136SINV_DUAL_TMPL.h>

/* Compatibility names consumed by the existing PMSM controller. */
#define MOTOR_TYPE                     SM060R20B30MNAD_MOTOR_TYPE
#define MOTOR_PARAM_RS                 SM060R20B30MNAD_RS
#define MOTOR_PARAM_LS                 SM060R20B30MNAD_LD
#define MOTOR_PARAM_LD                 SM060R20B30MNAD_LD
#define MOTOR_PARAM_LQ                 SM060R20B30MNAD_LQ
#define MOTOR_PARAM_FLUX               SM060R20B30MNAD_FLUX
#define MOTOR_PARAM_POLE_PAIRS         SM060R20B30MNAD_POLE_PAIRS
#define MOTOR_PARAM_INERTIA            SM060R20B30MNAD_INERTIA
#define MOTOR_PARAM_FRICTION           SM060R20B30MNAD_FRICTION
#define MOTOR_PARAM_KV                 SM060R20B30MNAD_KV
#define MOTOR_PARAM_EMF                SM060R20B30MNAD_EMF
#define MOTOR_PARAM_RATED_VOLTAGE      SM060R20B30MNAD_RATED_VOLTAGE
#define MOTOR_PARAM_RATED_CURRENT      SM060R20B30MNAD_RATED_CURRENT
#define MOTOR_PARAM_NO_LOAD_CURRENT    SM060R20B30MNAD_NO_LOAD_CURRENT
#define MOTOR_PARAM_RATED_FREQUENCY    SM060R20B30MNAD_RATED_FREQUENCY
#define MOTOR_PARAM_MAX_SPEED          SM060R20B30MNAD_MAX_SPEED
#define MOTOR_PARAM_MAX_TORQUE         SM060R20B30MNAD_MAX_TORQUE
#define MOTOR_PARAM_MAX_DC_VOLTAGE     SM060R20B30MNAD_MAX_DC_VOLTAGE
#define MOTOR_PARAM_MAX_PH_CURRENT     SM060R20B30MNAD_MAX_PH_CURRENT

#define LAUNCHPAD 1
#define GMP_IRIS  0
#define BOARD_SELECTION LAUNCHPAD

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define SDPE_PROJECT_ID "mcs_pmsm_nt_f280049c"
#define SDPE_PROJECT_SUITE "mcs_pmsm_nt"
#define SDPE_PROJECT_VERSION "1.0.0"
#define SDPE_PROJECT_UPDATED_AT "2026-07-14"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Use the discrete controller implementation.
 */
// #define PMSM_CTRL_USING_DISCRETE_CTRL

/**
 * @brief Enable discrete-PID anti-saturation path.
 */
#define _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable the sliding-mode observer path, preserving the original LaunchPad project setting.
 */
#define ENABLE_SMO

/**
 * @brief Enable motor fault protection processing.
 */
#define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Runtime.
 */

/**
 * @brief Calibrate ADC offsets at startup.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable processor-in-the-loop mode.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable CiA402 framework debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use the inverter's active-low PWM logic.
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)

/**
 * @brief Enable three-level NPC modulation.
 */
// #define USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning level.
 *        Options: (1), (2), (3), (4)
 */
#define BUILD_LEVEL (4)

/**
 * @brief Number of directly sampled phase currents.
 *        Options: (2), (3)
 */
#define MC_CURRENT_SAMPLE_PHASE_MODE (2)

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
 * @brief Controller startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Main motor-control ISR frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Center-aligned PWM compare maximum at 100 MHz and 20 kHz.
 */
#define CTRL_PWM_CMP_MAX (2500 - 1)

/**
 * @brief PWM dead-band count.
 */
#define CTRL_PWM_DEADBAND_CMP (100)

/**
 * @brief F280049C system clock frequency in hertz.
 */
#define CTRL_SYS_FREQUENCY (100e6)

/**
 * @brief System tick divider derived from clock and PWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (CTRL_SYS_FREQUENCY / 1000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC reference voltage.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief ADC offset calibration timeout in milliseconds.
 */
#define TIMEOUT_ADC_CALIB_MS (10000)

/**
 * @brief DC-bus voltage per-unit base.
 */
#define CTRL_DCBUS_VOLTAGE (80.0f)

/**
 * @brief SVPWM phase-voltage base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief Encoder counts per mechanical revolution.
 */
#define CTRL_POS_ENC_FS (10000)

/**
 * @brief Mechanical encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS (0.0207000002f)

/**
 * @brief Mechanical speed and position division factor.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Phase-current sensor sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (MY_BOARD_PH_SHUNT_RESISTANCE_OHM * MY_BOARD_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (MY_BOARD_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensor gain.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (MY_BOARD_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-bus current sensor gain.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (MY_BOARD_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (MY_BOARD_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief DC-bus voltage sensor gain.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias.
 */
#define CTRL_DC_VOLTAGE_BIAS (MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Current deadband for dead-time polarity selection in amperes.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)

/**
 * @brief Dead-time polarity hysteresis in amperes.
 */
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)

/**
 * @brief Commissioning open-loop electrical frequency.
 */
#define MCS_OPEN_LOOP_FREQ_HZ (20.0f)

/**
 * @brief Open-loop electrical-frequency slew rate.
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S (20.0f)

/**
 * @brief Outer position-loop proportional gain.
 */
#define MCS_MECH_POSITION_KP_PU (5.0f)

/**
 * @brief Outer position-loop integral gain per second.
 */
#define MCS_MECH_POSITION_KI_PU_S (1.0f)

/**
 * @brief Velocity-loop proportional gain.
 */
#define MCS_MECH_VELOCITY_KP_PU (5.0f)

/**
 * @brief Velocity-loop integral gain per second.
 */
#define MCS_MECH_VELOCITY_KI_PU_S (1.0f)

/**
 * @brief Mechanical speed-command limit in rpm.
 */
#define MCS_MECH_SPEED_LIMIT_RPM (MOTOR_PARAM_MAX_SPEED)

/**
 * @brief Mechanical speed-command slew rate in rpm/s.
 */
#define MCS_MECH_SPEED_SLOPE_RPM_S (MOTOR_PARAM_MAX_SPEED)

/**
 * @brief Mechanical controller torque-current limit in amperes.
 */
#define MCS_MECH_CURRENT_LIMIT_A (0.3f * CTRL_CURRENT_BASE)

/**
 * @brief Encoder speed-estimator cutoff frequency.
 */
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)

/**
 * @brief Commissioning d-axis current reference.
 */
#define MCS_COMMISSIONING_ID_REF_A (0.1f * CTRL_CURRENT_BASE)

/**
 * @brief Commissioning q-axis current reference.
 */
#define MCS_COMMISSIONING_IQ_REF_A (0.1f * CTRL_CURRENT_BASE)

/**
 * @brief Commissioning speed reference.
 */
#define MCS_COMMISSIONING_SPEED_REF_RPM (0.1f * MOTOR_PARAM_MAX_SPEED)

/**
 * @brief CiA402 operation-enable transition delay.
 */
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)

/**
 * @brief ADC offset calibrator filter cutoff.
 */
#define MCS_ADC_CALIBRATOR_FC_HZ (20.0f)

/**
 * @brief ADC offset calibrator filter quality factor.
 */
#define MCS_ADC_CALIBRATOR_Q (0.707f)

// User project tail code
/* Accept the historical misspelling while source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_NT_F280049C_SETTINGS_H_
