/**
 * @file sdpe_pgs_sinv_rc_iris_bindings.h
 * @brief SDPE project bindings for PGS SINV RC F280039C IRIS Node.
 * @note Single-phase inverter project requirement prepared from xplt/ctrl_settings.h.
 *       This requirement introduces the GMP LVFB 150V 2-phase inverter board as the switching stage and the GMP Harmonia 3-phase LC filter as the grid filter. IRIS F280039C is included as the peripheral option provider.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_

#include "hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h"
#include "hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h"
#include "hardware_preset/mcu_board/iris_f280039c_node.h"

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Original ctrl_settings.h includes are intentionally ignored during this SDPE migration trial. */

// Project metadata
#define SDPE_PROJECT_ID "pgs_sinv_rc_iris_node"
#define SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define SDPE_PROJECT_VERSION "0.1.0"
#define SDPE_PROJECT_UPDATED_AT "2026-07-10"

// Selection macros
// Enable Discrete PID controller anti-saturation algorithm.
#define _USE_DEBUG_DISCRETE_PID
// Enable ADC calibration.
#define SPECIFY_ENABLE_ADC_CALIBRATE
// Enable PIL simulation function. This macro disables controller output.
// #define ENBALE_GMP_DL_PIL_SIM
// Enable CiA402 debug information.
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

// Option macros
// Single-phase inverter incremental debug build level.
// BUILD_LEVEL 1: modulator and resistive-load validation.
// BUILD_LEVEL 2: voltage closed-loop validation.
// BUILD_LEVEL 3: current-loop and full controller validation.
// Options: (1), (2), (3)
#define BUILD_LEVEL (1)
// Use negative PWM modulator logic.
// Options: (0), (1)
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (0)
// PWM base for inverter phase L.
// Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
#define PHASE_L_BASE IRIS_EPWM3_BASE
#define PHASE_L_BASE_IRIS_EPWM3_BASE 1
// PWM base for inverter phase N.
// Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
#define PHASE_N_BASE IRIS_EPWM4_BASE
#define PHASE_N_BASE_IRIS_EPWM4_BASE 1
// Gate-driver enable GPIO.
// Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_ENABLE_PORT_IRIS_GPIO1 1
// Gate-driver reset GPIO.
// Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
#define PWM_RESET_PORT IRIS_GPIO3
#define PWM_RESET_PORT_IRIS_GPIO3 1
// System status LED.
// Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
#define SYSTEM_LED IRIS_LED1
#define SYSTEM_LED_IRIS_LED1 1
// Controller status LED.
// Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
#define CONTROLLER_LED IRIS_LED2
#define CONTROLLER_LED_IRIS_LED2 1
// AC current ADC result register base.
// Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
#define INV_IAC_RESULT_BASE ADC_CH1_RESULT_BASE
#define INV_IAC_RESULT_BASE_ADC_CH1_RESULT_BASE 1
// AC current ADC channel.
// Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
#define INV_IAC ADC_CH1
#define INV_IAC_ADC_CH1 1
// AC voltage ADC result register base.
// Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
#define INV_VAC_RESULT_BASE ADC_CH2_RESULT_BASE
#define INV_VAC_RESULT_BASE_ADC_CH2_RESULT_BASE 1
// AC voltage ADC channel.
// Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
#define INV_VAC ADC_CH2
#define INV_VAC_ADC_CH2 1
// DC bus voltage ADC result register base.
// Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
#define INV_VBUS_RESULT_BASE ADC_CH3_RESULT_BASE
#define INV_VBUS_RESULT_BASE_ADC_CH3_RESULT_BASE 1
// DC bus voltage ADC channel.
// Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
#define INV_VBUS ADC_CH3
#define INV_VBUS_ADC_CH3 1

// Requirement bindings
/**
 * @brief Startup delay in ms.
 */
#define CTRL_STARTUP_DELAY (100)

/**
 * @brief Controller ISR frequency.
 */
#define CONTROLLER_FREQUENCY (20e3)

/**
 * @brief PWM compare maximum.
 */
#define CTRL_PWM_CMP_MAX (3000 - 1)

/**
 * @brief PWM deadband compare value.
 */
#define CTRL_PWM_DEADBAND_CMP (50)

/**
 * @brief System tick divider derived from PWM period.
 */
#define DSP_C2000_DSP_TIME_DIV (120000 / CTRL_PWM_CMP_MAX / 2)

/**
 * @brief ADC voltage reference.
 */
#define CTRL_ADC_VOLTAGE_REF (3.3f)

/**
 * @brief Rated DC bus voltage.
 */
#define CTRL_DCBUS_VOLTAGE (60.0f)

/**
 * @brief Rated AC grid/load RMS voltage.
 */
#define CTRL_GRID_VOLTAGE_RMS (24.0f)

/**
 * @brief Rated AC output RMS current.
 */
#define CTRL_RATED_CURRENT_RMS (10.0f)

/**
 * @brief Voltage per-unit base, using peak value.
 */
#define CTRL_VOLTAGE_BASE (34.0f)

/**
 * @brief Current per-unit base, using peak value.
 */
#define CTRL_CURRENT_BASE (14.14f)

/**
 * @brief AC voltage sensing gain from the grid LC filter voltage sense path.
 */
#define CTRL_AC_VOLTAGE_SENSITIVITY HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_GAIN

/**
 * @brief AC voltage sensing ADC bias from the grid LC filter.
 */
#define CTRL_AC_VOLTAGE_BIAS HARMONIA_3PH_LC_FILTER_PH_VOLTAGE_SENSE_BIAS_V

/**
 * @brief AC current sensing sensitivity from the LVFB inverter current sensor.
 */
#define CTRL_AC_CURRENT_SENSITIVITY GMP_LVFB_CURRENT_SENSITIVITY

/**
 * @brief AC current sensing ADC bias from the LVFB inverter current sensor.
 */
#define CTRL_AC_CURRENT_BIAS GMP_LVFB_CURRENT_BIAS_V

/**
 * @brief DC bus voltage sensing gain from the LVFB inverter voltage sensor.
 */
#define CTRL_DC_VOLTAGE_SENSITIVITY GMP_LVFB_VOLTAGE_SENSITIVITY

/**
 * @brief DC bus voltage sensing ADC bias from the LVFB inverter voltage sensor.
 */
#define CTRL_DC_VOLTAGE_BIAS GMP_LVFB_VOLTAGE_BIAS_V

/**
 * @brief Maximum hardware DC bus voltage from the LVFB inverter board.
 */
#define CTRL_MAX_HW_VOLTAGE GMP_LVFB_VBUS_MAX_V

/**
 * @brief Maximum continuous RMS hardware current from the LVFB inverter board.
 */
#define CTRL_MAX_HW_CURRENT GMP_LVFB_CURRENT_MAX_RMS_A

/**
 * @brief Project DC bus over-voltage protection threshold.
 */
#define CTRL_PROT_VBUS_MAX (100.0f)

/**
 * @brief ADC calibration timeout in ms.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief SPLL close-loop convergence criterion.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// User project tail code
/* Project-specific tail extension point. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_
