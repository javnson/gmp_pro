/**
 * @file sdpe_mcs_pmsm_id_f280049c_settings.h
 * @brief SDPE project bindings for MCS PMSM Identification f280049c.
 * @note Structured SDPE hardware, peripheral and control settings migrated from f280049c/xplt/ctrl_settings.h.
 */

#ifndef _PROJECT_SDPE_MCS_PMSM_ID_F280049C_SETTINGS_H_
#define _PROJECT_SDPE_MCS_PMSM_ID_F280049C_SETTINGS_H_

#include <ctl/hardware_preset/inverter_3ph/ti_boostxl_3phganinv.h>
#include <ctl/hardware_preset/mcu_board/launchxl_f280049c.h>
#include <ctl/hardware_preset/pmsm_motor/sm060r20b30mnad.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* The inverter is selected as an SDPE hardware entity. */

#define LAUNCHPAD 1
#define GMP_IRIS  0
#define BOARD_SELECTION LAUNCHPAD

// Common prefix code: MCS PMSM Identification Common Control
/* Platform-independent settings only. Project hardware is supplied by the including project header. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define MCS_PMSM_ID_F280049C_SDPE_PROJECT_ID "mcs_pmsm_id_f280049c"
#define MCS_PMSM_ID_F280049C_SDPE_PROJECT_SUITE "mcs_pmsm_id"
#define MCS_PMSM_ID_F280049C_SDPE_PROJECT_VERSION "1.0.0"
#define MCS_PMSM_ID_F280049C_SDPE_PROJECT_UPDATED_AT "2026-08-08"

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Migrated from f280049c/xplt/ctrl_settings.h.
 */
#define _USE_DEBUG_DISCRETE_PID

//=================================================================================================
/**
 * @brief Protection.
 */

/**
 * @brief Migrated from f280049c/xplt/ctrl_settings.h.
 */
#define ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Sensing and Calibration.
 */

/**
 * @brief Migrated from f280049c/xplt/ctrl_settings.h.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

//=================================================================================================
/**
 * @brief Commissioning.
 */

/**
 * @brief 1=open-loop hardware validation; 2=current loop; 3=measured-angle current loop; 4=speed loop; 5=position loop; 6=communication mode.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#define BUILD_LEVEL (2)

//=================================================================================================
/**
 * @brief Controller Options.
 */

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
#define PHASE_U_BASE EPWM_J4_PHASE_U_BASE

/**
 * @brief V-phase PWM pair.
 *        Options: BOOSTXL_J4_PWM1_BASE, BOOSTXL_J4_PWM2_BASE, BOOSTXL_J4_PWM3_BASE, BOOSTXL_J8_PWM1_BASE, BOOSTXL_J8_PWM2_BASE, BOOSTXL_J8_PWM3_BASE
 */
#define PHASE_V_BASE EPWM_J4_PHASE_V_BASE

/**
 * @brief W-phase PWM pair.
 *        Options: BOOSTXL_J4_PWM1_BASE, BOOSTXL_J4_PWM2_BASE, BOOSTXL_J4_PWM3_BASE, BOOSTXL_J8_PWM1_BASE, BOOSTXL_J8_PWM2_BASE, BOOSTXL_J8_PWM3_BASE
 */
#define PHASE_W_BASE EPWM_J4_PHASE_W_BASE

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
#define INV_VBUS J3_VDC

/**
 * @brief DC-bus voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE J3_VDC_RESULT_BASE

/**
 * @brief DC-bus current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IBUS

/**
 * @brief DC-bus current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IBUS_RESULT_BASE

//=================================================================================================
/**
 * @brief ADC Phase Current.
 */

/**
 * @brief U-phase current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IU J3_IU

/**
 * @brief U-phase current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IU_RESULT_BASE J3_IU_RESULT_BASE

/**
 * @brief V-phase current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IV J3_IV

/**
 * @brief V-phase current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IV_RESULT_BASE J3_IV_RESULT_BASE

/**
 * @brief W-phase current SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_IW J3_IW

/**
 * @brief W-phase current result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_IW_RESULT_BASE J3_IW_RESULT_BASE

//=================================================================================================
/**
 * @brief ADC Phase Voltage.
 */

/**
 * @brief U-phase voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_UU J3_VU

/**
 * @brief U-phase voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_UU_RESULT_BASE J3_VU_RESULT_BASE

/**
 * @brief V-phase voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_UV J3_VV

/**
 * @brief V-phase voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_UV_RESULT_BASE J3_VV_RESULT_BASE

/**
 * @brief W-phase voltage SOC.
 *        Options: BOOSTXL_J3_AIN1, BOOSTXL_J3_AIN2, BOOSTXL_J3_AIN3, BOOSTXL_J3_AIN4, BOOSTXL_J3_AIN5, BOOSTXL_J3_AIN6, BOOSTXL_J3_AIN7, BOOSTXL_J3_AIN8, BOOSTXL_J7_AIN1, BOOSTXL_J7_AIN2, BOOSTXL_J7_AIN3, BOOSTXL_J7_AIN4, BOOSTXL_J7_AIN5, BOOSTXL_J7_AIN6, BOOSTXL_J7_AIN7, BOOSTXL_J7_AIN8
 */
#define INV_UW J3_VW

/**
 * @brief W-phase voltage result base.
 *        Options: BOOSTXL_J3_AIN1_RESULT_BASE, BOOSTXL_J3_AIN2_RESULT_BASE, BOOSTXL_J3_AIN3_RESULT_BASE, BOOSTXL_J3_AIN4_RESULT_BASE, BOOSTXL_J3_AIN5_RESULT_BASE, BOOSTXL_J3_AIN6_RESULT_BASE, BOOSTXL_J3_AIN7_RESULT_BASE, BOOSTXL_J3_AIN8_RESULT_BASE, BOOSTXL_J7_AIN1_RESULT_BASE, BOOSTXL_J7_AIN2_RESULT_BASE, BOOSTXL_J7_AIN3_RESULT_BASE, BOOSTXL_J7_AIN4_RESULT_BASE, BOOSTXL_J7_AIN5_RESULT_BASE, BOOSTXL_J7_AIN6_RESULT_BASE, BOOSTXL_J7_AIN7_RESULT_BASE, BOOSTXL_J7_AIN8_RESULT_BASE
 */
#define INV_UW_RESULT_BASE J3_VW_RESULT_BASE

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

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
#define DSP_C2000_DSP_TIME_DIV (100000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC reference voltage used by all sensor conversions.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Configured DC-bus voltage base.
 */
#define CTRL_DCBUS_VOLTAGE (36.0f)

/**
 * @brief Phase-voltage per-unit base derived from the DC-bus base.
 */
#define CTRL_VOLTAGE_BASE (CTRL_DCBUS_VOLTAGE / 1.73205081f)

/**
 * @brief Phase-current per-unit base in amperes.
 */
#define CTRL_CURRENT_BASE (10.0f)

/**
 * @brief
 */
#define CTRL_SPEED_RPM_BASE SM060R20B30MNAD_MAX_SPEED

/**
 * @brief
 */
#define MOTOR_PARAM_RATED_FREQUENCY SM060R20B30MNAD_RATED_FREQUENCY

/**
 * @brief Encoder counts per mechanical revolution.
 */
#define CTRL_POS_ENC_FS (4096*4)

/**
 * @brief Mechanical encoder position bias in per unit.
 */
#define CTRL_POS_ENC_BIAS (0.0207000002f)

/**
 * @brief DC-bus voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY (MY_BOARD_DCBUS_VOLTAGE_SENSE_GAIN)

/**
 * @brief DC-bus voltage sensor bias in volts.
 */
#define CTRL_DC_VOLTAGE_BIAS (MY_BOARD_DCBUS_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief DC-bus current sensing gain. The selected inverter reports SENSOR_NONE for this path.
 */
#define CTRL_DC_CURRENT_SENSITIVITY (MY_BOARD_DCBUS_CURRENT_SENSE_GAIN)

/**
 * @brief DC-bus current sensor bias.
 */
#define CTRL_DC_CURRENT_BIAS (MY_BOARD_DCBUS_CURRENT_SENSE_BIAS_V)

/**
 * @brief Phase-current sensor sensitivity in volts per ampere.
 */
#define CTRL_INVERTER_CURRENT_SENSITIVITY (MY_BOARD_PH_SHUNT_RESISTANCE_OHM * MY_BOARD_PH_CSA_GAIN_V_V)

/**
 * @brief Phase-current sensor zero-current bias in volts.
 */
#define CTRL_INVERTER_CURRENT_BIAS (MY_BOARD_PH_CSA_BIAS_V)

/**
 * @brief Phase-voltage sensing gain in ADC volts per measured volt.
 */
#define CTRL_INVERTER_VOLTAGE_SENSITIVITY (MY_BOARD_PH_VOLTAGE_SENSE_GAIN)

/**
 * @brief Phase-voltage sensor bias in volts.
 */
#define CTRL_INVERTER_VOLTAGE_BIAS (MY_BOARD_PH_VOLTAGE_SENSE_BIAS_V)

/**
 * @brief Mechanical speed and position division factor.
 */
#define CTRL_MECH_DIV (5)

/**
 * @brief Electrical frequency command in hertz used by the BUILD_LEVEL 1 V/f path and the BUILD_LEVEL 2 synthetic-angle current-loop path.
 */
#define MCS_OPEN_LOOP_FREQ_HZ (20.0f)

/**
 * @brief Maximum electrical-frequency slew rate in hertz per second for the synthetic angle generator.
 */
#define MCS_OPEN_LOOP_FREQ_SLOPE_HZ_S (20.0f)

/**
 * @brief The current limit value at which the machine must be shut down.
 */
#ifndef MCS_MAX_SHUTDOWN_CURRENT_A
#define MCS_MAX_SHUTDOWN_CURRENT_A (10.0f)
#endif // MCS_MAX_SHUTDOWN_CURRENT_A

/**
 * @brief Circular saturation limit for voltage vector magnitude in V.
 */
#define MCS_MAX_CIR_SATURATION_VOLTAGE_V (10.0f)

/**
 * @brief
 */
#define MOTOR_PARAM_MAX_SPEED SM060R20B30MNAD_MAX_SPEED

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_TYPE SM060R20B30MNAD_MOTOR_TYPE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_KV SM060R20B30MNAD_KV

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_EMF SM060R20B30MNAD_EMF

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_VOLTAGE SM060R20B30MNAD_RATED_VOLTAGE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_RATED_CURRENT SM060R20B30MNAD_RATED_CURRENT

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_NO_LOAD_CURRENT SM060R20B30MNAD_NO_LOAD_CURRENT

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_TORQUE SM060R20B30MNAD_MAX_TORQUE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_DC_VOLTAGE SM060R20B30MNAD_MAX_DC_VOLTAGE

/**
 * @brief Alias the selected sm060r20b30mnad hardware parameter into the MCS control contract.
 */
#define MOTOR_PARAM_MAX_PH_CURRENT SM060R20B30MNAD_MAX_PH_CURRENT

/**
 * @brief Main motor-control ISR frequency in hertz.
 */
#define CONTROLLER_FREQUENCY (20e3f)

/**
 * @brief Controller startup delay in milliseconds.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Migrated from f280049c/xplt/ctrl_settings.h.
 */
#define TIMEOUT_ADC_CALIB_MS (10000)

//=================================================================================================
/**
 * @brief Common fallbacks: MCS PMSM Identification Common Control.
 */

//=================================================================================================
/**
 * @brief Control Algorithm.
 */

/**
 * @brief Use the discrete controller implementation instead of the default continuous controller path.
 */
// #ifndef PMSM_CTRL_USING_DISCRETE_CTRL
// #define PMSM_CTRL_USING_DISCRETE_CTRL
// #endif // PMSM_CTRL_USING_DISCRETE_CTRL

/**
 * @brief Enable the existing discrete-PID anti-saturation debug path.
 */
// #ifndef _USE_DEBUG_DISCRETE_PID
// #define _USE_DEBUG_DISCRETE_PID
// #endif // _USE_DEBUG_DISCRETE_PID

/**
 * @brief Enable the sliding-mode observer path. Disabled to preserve the current sensored-control build.
 */
// #ifndef ENABLE_SMO
// #define ENABLE_SMO
// #endif // ENABLE_SMO

/**
 * @brief Enable motor fault protection processing.
 */
// #ifndef ENABLE_MOTOR_FAULT_PROTECTION
// #define ENABLE_MOTOR_FAULT_PROTECTION
// #endif // ENABLE_MOTOR_FAULT_PROTECTION

//=================================================================================================
/**
 * @brief Controller Runtime.
 */

/**
 * @brief Calibrate ADC offsets before enabling normal control.
 */
// #ifndef SPECIFY_ENABLE_ADC_CALIBRATE
// #define SPECIFY_ENABLE_ADC_CALIBRATE
// #endif // SPECIFY_ENABLE_ADC_CALIBRATE

/**
 * @brief Enable processor-in-the-loop mode and suppress direct PWM controller output.
 */
// #ifndef ENABLE_GMP_DL_PIL_SIM
// #define ENABLE_GMP_DL_PIL_SIM
// #endif // ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable CiA402/GMP framework debug information.
 */
// #ifndef GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO
// #endif // GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use active-low PWM modulation for the selected inverter gate path.
 */
// #ifndef PWM_MODULATOR_USING_NEGATIVE_LOGIC
// #define PWM_MODULATOR_USING_NEGATIVE_LOGIC (1)
// #endif // PWM_MODULATOR_USING_NEGATIVE_LOGIC

/**
 * @brief Use the three-level NPC modulator. Disabled for the two-level three-phase inverter.
 */
// #ifndef USING_NPC_MODULATOR
// #define USING_NPC_MODULATOR
// #endif // USING_NPC_MODULATOR

//=================================================================================================
/**
 * @brief Controller Options.
 */

/**
 * @brief Incremental commissioning level. 1: V/f voltage open loop; 2: current loop with synthetic electrical angle; 3: current loop with encoder angle; 4: speed loop with encoder feedback.
 *        Options: (1), (2), (3), (4), (5), (6)
 */
#ifndef BUILD_LEVEL
#define BUILD_LEVEL (2)
#endif // BUILD_LEVEL

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

/**
 * @brief Main motor-control ISR frequency in hertz.
 */
#ifndef CONTROLLER_FREQUENCY
#define CONTROLLER_FREQUENCY (20e3f)
#endif // CONTROLLER_FREQUENCY

/**
 * @brief Minimum absolute phase current in amperes before PWM dead-time compensation selects a current direction. Converted to PU with CTRL_CURRENT_BASE at initialization.
 */
#ifndef MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A
#define MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A (0.2f)
#endif // MCS_PWM_DEADTIME_COMP_CURRENT_DEADBAND_A

/**
 * @brief Phase-current hysteresis band in amperes used to prevent dead-time compensation direction chatter around zero current.
 */
#ifndef MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A
#define MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A (0.05f)
#endif // MCS_PWM_DEADTIME_COMP_CURRENT_HYSTERESIS_A

/**
 * @brief Cutoff frequency in hertz of the low-pass filter applied to encoder-derived mechanical speed.
 */
#ifndef MCS_ENCODER_SPEED_FILTER_FC_HZ
#define MCS_ENCODER_SPEED_FILTER_FC_HZ (20.0f)
#endif // MCS_ENCODER_SPEED_FILTER_FC_HZ

/**
 * @brief Position-loop proportional gain. Input is mechanical position error in PU revolutions and output is speed reference in PU, so the gain is speed_pu/position_pu.
 */
#ifndef MCS_MECH_POSITION_KP_PU
#define MCS_MECH_POSITION_KP_PU (5.0f)
#endif // MCS_MECH_POSITION_KP_PU

/**
 * @brief Position-loop integral gain in speed_pu/(position_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#ifndef MCS_MECH_POSITION_KI_PU_S
#define MCS_MECH_POSITION_KI_PU_S (1.0f)
#endif // MCS_MECH_POSITION_KI_PU_S

/**
 * @brief Velocity-loop proportional gain. Input is speed error in PU and output is q-axis current/torque reference in PU, so the gain is current_pu/speed_pu.
 */
#ifndef MCS_MECH_VELOCITY_KP_PU
#define MCS_MECH_VELOCITY_KP_PU (5.0f)
#endif // MCS_MECH_VELOCITY_KP_PU

/**
 * @brief Velocity-loop integral gain in current_pu/(speed_pu*s). The continuous gain is divided by the mechanical-loop sampling frequency internally.
 */
#ifndef MCS_MECH_VELOCITY_KI_PU_S
#define MCS_MECH_VELOCITY_KI_PU_S (1.0f)
#endif // MCS_MECH_VELOCITY_KI_PU_S

/**
 * @brief D-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#ifndef MCS_COMMISSIONING_ID_REF_A
#define MCS_COMMISSIONING_ID_REF_A (1.0f)
#endif // MCS_COMMISSIONING_ID_REF_A

/**
 * @brief Q-axis current reference in amperes used by BUILD_LEVEL 2 and 3 commissioning. Converted to PU using CTRL_CURRENT_BASE.
 */
#ifndef MCS_COMMISSIONING_IQ_REF_A
#define MCS_COMMISSIONING_IQ_REF_A (1.0f)
#endif // MCS_COMMISSIONING_IQ_REF_A

/**
 * @brief Mechanical speed reference in rpm used by BUILD_LEVEL 4 commissioning. Converted to PU using MOTOR_PARAM_MAX_SPEED.
 */
#ifndef MCS_COMMISSIONING_SPEED_REF_RPM
#define MCS_COMMISSIONING_SPEED_REF_RPM (300.0f)
#endif // MCS_COMMISSIONING_SPEED_REF_RPM

/**
 * @brief Absolute mechanical speed-command limit in rpm. It is divided by MOTOR_PARAM_MAX_SPEED to obtain the controller PU limit.
 */
#ifndef MCS_MECH_SPEED_LIMIT_RPM
#define MCS_MECH_SPEED_LIMIT_RPM (3000.0f)
#endif // MCS_MECH_SPEED_LIMIT_RPM

/**
 * @brief Maximum mechanical speed-command slew rate in rpm/s. The configured value corresponds to 1 PU/s for the selected 3000 rpm motor.
 */
#ifndef MCS_MECH_SPEED_SLOPE_RPM_S
#define MCS_MECH_SPEED_SLOPE_RPM_S (3000.0f)
#endif // MCS_MECH_SPEED_SLOPE_RPM_S

/**
 * @brief Absolute q-axis current/torque command limit in amperes. It is divided by CTRL_CURRENT_BASE to obtain the controller PU limit; 3 A corresponds to the previous 0.3 PU setting.
 */
#ifndef MCS_MECH_CURRENT_LIMIT_A
#define MCS_MECH_CURRENT_LIMIT_A (3.0f)
#endif // MCS_MECH_CURRENT_LIMIT_A

/**
 * @brief Rectangular saturation limit for d/q axes in V.
 */
#ifndef MCS_MAX_RECT_SATURATION_VOLTAGE_V
#define MCS_MAX_RECT_SATURATION_VOLTAGE_V (10.0f)
#endif // MCS_MAX_RECT_SATURATION_VOLTAGE_V

/**
 * @brief
 */
#ifndef MCS_MAX_DC_BUS_VOLTAGE_V
#define MCS_MAX_DC_BUS_VOLTAGE_V (CTRL_DCBUS_VOLTAGE*1.2f)
#endif // MCS_MAX_DC_BUS_VOLTAGE_V

/**
 * @brief
 */
#ifndef MCS_MIN_DC_BUS_VOLTAGE_V
#define MCS_MIN_DC_BUS_VOLTAGE_V (CTRL_DCBUS_VOLTAGE*0.2f)
#endif // MCS_MIN_DC_BUS_VOLTAGE_V

/**
 * @brief
 */
#ifndef MOTOR_PARAM_INERTIA
#define MOTOR_PARAM_INERTIA SM060R20B30MNAD_INERTIA
#endif // MOTOR_PARAM_INERTIA

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FLUX
#define MOTOR_PARAM_FLUX SM060R20B30MNAD_FLUX
#endif // MOTOR_PARAM_FLUX

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LD
#define MOTOR_PARAM_LD SM060R20B30MNAD_LD
#endif // MOTOR_PARAM_LD

/**
 * @brief
 */
#ifndef MOTOR_PARAM_RS
#define MOTOR_PARAM_RS SM060R20B30MNAD_RS
#endif // MOTOR_PARAM_RS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LQ
#define MOTOR_PARAM_LQ SM060R20B30MNAD_LQ
#endif // MOTOR_PARAM_LQ

/**
 * @brief
 */
#ifndef MOTOR_PARAM_LS
#define MOTOR_PARAM_LS SM060R20B30MNAD_LD
#endif // MOTOR_PARAM_LS

/**
 * @brief
 */
#ifndef MOTOR_PARAM_FRICTION
#define MOTOR_PARAM_FRICTION SM060R20B30MNAD_FRICTION
#endif // MOTOR_PARAM_FRICTION

/**
 * @brief
 */
#ifndef MOTOR_PARAM_POLE_PAIRS
#define MOTOR_PARAM_POLE_PAIRS SM060R20B30MNAD_POLE_PAIRS
#endif // MOTOR_PARAM_POLE_PAIRS

/**
 * @brief Controller startup delay in milliseconds.
 */
#ifndef CTRL_STARTUP_DELAY
#define CTRL_STARTUP_DELAY (100)
#endif // CTRL_STARTUP_DELAY

/**
 * @brief Minimum delay in milliseconds before the CiA402 state machine enters Operation Enabled. The project control tick is expressed in milliseconds.
 */
#ifndef MCS_CIA402_OPERATION_ENABLE_DELAY_MS
#define MCS_CIA402_OPERATION_ENABLE_DELAY_MS (100)
#endif // MCS_CIA402_OPERATION_ENABLE_DELAY_MS

/**
 * @brief Cutoff frequency in hertz of the second-order low-pass filter used while estimating ADC zero offsets.
 */
#ifndef MCS_ADC_CALIBRATOR_FC_HZ
#define MCS_ADC_CALIBRATOR_FC_HZ (20.0f)
#endif // MCS_ADC_CALIBRATOR_FC_HZ

/**
 * @brief Quality factor of the ADC calibration low-pass filter; 0.707 gives an approximately Butterworth second-order response.
 */
#ifndef MCS_ADC_CALIBRATOR_Q
#define MCS_ADC_CALIBRATOR_Q (0.707f)
#endif // MCS_ADC_CALIBRATOR_Q

// Common tail code: MCS PMSM Identification Common Control
/* Accept the historical misspelling while all source code uses the canonical switch. */
#if defined ENBALE_GMP_DL_PIL_SIM && !defined ENABLE_GMP_DL_PIL_SIM
#define ENABLE_GMP_DL_PIL_SIM
#endif

/* Reject unsupported incremental build levels at preprocessing time. */
#if (BUILD_LEVEL < 1) || (BUILD_LEVEL > 4)
#error "BUILD_LEVEL must be 1 (V/f), 2 (current loop/synthetic angle), 3 (current loop/encoder), or 4 (speed loop)."
#endif

// User project tail code
/* No additional platform-specific tail definitions. */

/**
 * @brief Unit convert from V into phase voltage pu.
 */
#define VOLT_PU(_X_X_) (((_X_X_)/CTRL_VOLTAGE_BASE))

/**
 * @brief Unit convert from A into phase current pu.
 */
#define CURR_PU(_X_X_) (((_X_X_)/CTRL_CURRENT_BASE))

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_MCS_PMSM_ID_F280049C_SETTINGS_H_
