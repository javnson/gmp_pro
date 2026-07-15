/**
 * @file sdpe_dps_fsbb_iris_bindings.h
 * @brief SDPE project bindings for DPS FSBB on F280039C IRIS Node.
 */

#ifndef _PROJECT_SDPE_DPS_FSBB_IRIS_BINDINGS_H_
#define _PROJECT_SDPE_DPS_FSBB_IRIS_BINDINGS_H_

#include <ctl/component/hardware_preset/current_sensor/tmcs1133_b2a.h>
#include <ctl/component/hardware_preset/half_bridge/fsbb_inline_shunt_half_bridge.h>
#include <ctl/component/hardware_preset/half_bridge/lvfb_half_bridge_phase_a.h>
#include <ctl/component/hardware_preset/mcu_board/iris_f280039c_node.h>

// Project metadata
#define SDPE_PROJECT_ID "dps_fsbb_iris_node"
#define SDPE_PROJECT_SUITE "dps_fsbb"

// Requirement bindings
/**
 * @brief Voltage sensor sensitivity for FSBB input voltage ADC channel.
 */
#define CTRL_VIN_VOLTAGE_SENSITIVITY QUAD_SENSOR_VCH_SENSITIVITY_V_PER_V

/**
 * @brief Voltage sensor bias for FSBB input voltage ADC channel.
 */
#define CTRL_VIN_VOLTAGE_BIAS QUAD_SENSOR_VCH_BIAS_V

/**
 * @brief Current sensor sensitivity for FSBB inductor current ADC channel.
 */
#define CTRL_INDUCTOR_CURRENT_SENSITIVITY FSBB_5M_SHUNT_SENSITIVITY_V_PER_A

/**
 * @brief Current sensor bias for FSBB inductor current ADC channel.
 */
#define CTRL_INDUCTOR_CURRENT_BIAS FSBB_5M_SHUNT_BIAS_V

/**
 * @brief Voltage sensor sensitivity for FSBB output voltage ADC channel.
 */
#define CTRL_VOUT_VOLTAGE_SENSITIVITY LVFB_VDIV_150V_SENSITIVITY_V_PER_V

/**
 * @brief Voltage sensor bias for FSBB output voltage ADC channel.
 */
#define CTRL_VOUT_VOLTAGE_BIAS LVFB_VDIV_150V_BIAS_V

/**
 * @brief Direct parameter binding example using scope-style entity.parameter syntax.
 */
#define CTRL_REFERENCE_CURRENT_RANGE_A TMCS1133_B2A_RANGE_A

/**
 * @brief Manual binding example for requirement values that are not owned by a hardware entity.
 */
#define CTRL_VIN_ADC_OFFSET_MANUAL (2048U)

// Board peripheral mapping
#define PHASE_BUCK_BASE IRIS_F280039C_EPWM1_BASE
#define PHASE_BOOST_BASE IRIS_F280039C_EPWM2_BASE
#define FSBB_VIN IRIS_F280039C_ADC_CH1
#define FSBB_VOUT IRIS_F280039C_ADC_CH2
#define FSBB_IL IRIS_F280039C_ADC_CH3
#define FSBB_IOUT IRIS_F280039C_ADC_CH4

// Global project macros
#define SDPE_ENABLE_DPS_FSBB_BINDINGS 1

#endif // _PROJECT_SDPE_DPS_FSBB_IRIS_BINDINGS_H_
