/**
 * @file ctrl_settings.h
 * @brief SDPE Generated Project Configuration
 */

#ifndef _CTRL_SETTINGS_H_
#define _CTRL_SETTINGS_H_

/* USER CODE BEGIN GLOBAL_INCLUDES */

/* USER CODE END GLOBAL_INCLUDES */

/* ============================================================================
 * 1. Project-Level Overrides (Highest Priority)
 * ============================================================================ */

/* Overrides for ext_encoder */

#ifndef SDPE_EXT_ENCODER_MECH_OFFSET_PU
#define SDPE_EXT_ENCODER_MECH_OFFSET_PU                 0.618
#endif



/* USER CODE BEGIN OVERRIDES_POST */

/* USER CODE END OVERRIDES_POST */

/* ============================================================================
 * 2. Hardware Preset Includes
 * Paths are relative to $GMP_PRO_LOCATION (System Search Path)
 * ============================================================================ */

#include <ctl/hardware_presets/mcu_settings/global_f280049c_100mhz_20khz.h>

#include <ctl/hardware_presets/motor_parameters/mtr_sm060_24v_150w.h>

#include <ctl/hardware_presets/boards/brd_lvhb_v1_sic.h>

#include <ctl/hardware_presets/boards/brd_lvhb_v1_sic.h>

#include <ctl/hardware_presets/boards/brd_lvhb_v1_sic.h>

#include <ctl/hardware_presets/encoders/inst_enc_as5047p_14bit.h>


/* USER CODE BEGIN INCLUDES_POST */

/* USER CODE END INCLUDES_POST */

/* ============================================================================
 * 3. Exported Cross-Platform API Macros (Topology Mapped)
 * ============================================================================ */

/** @brief CTRL_ADC_VOLTAGE_REF mapped to SDPE_SYS_CONFIG_ADC_VREF */
#ifndef CTRL_ADC_VOLTAGE_REF
#define CTRL_ADC_VOLTAGE_REF                SDPE_SYS_CONFIG_ADC_VREF
#endif

/** @brief CONTROLLER_FREQUENCY mapped to SDPE_SYS_CONFIG_CONTROLLER_FREQ_HZ */
#ifndef CONTROLLER_FREQUENCY
#define CONTROLLER_FREQUENCY                SDPE_SYS_CONFIG_CONTROLLER_FREQ_HZ
#endif

/** @brief CTRL_MAX_PHASE_CURRENT mapped to SDPE_MAIN_INV_BOARD_MAX_CONT_ARMS */
#ifndef CTRL_MAX_PHASE_CURRENT
#define CTRL_MAX_PHASE_CURRENT              SDPE_MAIN_INV_BOARD_MAX_CONT_ARMS
#endif

/** @brief CTRL_MAX_BUS_VOLTAGE mapped to SDPE_MAIN_INV_BOARD_MAX_VOLTAGE_V */
#ifndef CTRL_MAX_BUS_VOLTAGE
#define CTRL_MAX_BUS_VOLTAGE                SDPE_MAIN_INV_BOARD_MAX_VOLTAGE_V
#endif

/** @brief CTRL_ENCODER_OFFSET_PU mapped to SDPE_EXT_ENCODER_MECH_OFFSET_PU */
#ifndef CTRL_ENCODER_OFFSET_PU
#define CTRL_ENCODER_OFFSET_PU              SDPE_EXT_ENCODER_MECH_OFFSET_PU
#endif

/** @brief CTRL_MOTOR_POLE_PAIRS mapped to SDPE_MTR_LOAD_POLE_PAIRS */
#ifndef CTRL_MOTOR_POLE_PAIRS
#define CTRL_MOTOR_POLE_PAIRS               SDPE_MTR_LOAD_POLE_PAIRS
#endif


/* USER CODE BEGIN HEADER_BOTTOM */
#define NULL 0

#define NULL_PTR 1
/* USER CODE END HEADER_BOTTOM */

#endif // _CTRL_SETTINGS_H_