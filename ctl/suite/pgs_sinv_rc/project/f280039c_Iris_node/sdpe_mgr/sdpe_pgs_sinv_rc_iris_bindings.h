/**
 * @file sdpe_pgs_sinv_rc_iris_bindings.h
 * @brief SDPE project bindings for PGS SINV RC F280039C IRIS Node.
 * @note Single-phase inverter project requirement prepared from xplt/ctrl_settings.h.
 *       This requirement introduces the GMP LVFB 150V 2-phase inverter board as the switching stage and the GMP Harmonia 3-phase LC filter as the grid filter. IRIS F280039C is included as the peripheral option provider.
 */

#ifndef _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_
#define _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_

#include <ctl/hardware_preset/grid_lc_filter/gmp_harmonia_3ph_lc_filter.h>
#include <ctl/hardware_preset/half_bridge/gmp_lvfb_150_2ph_v2.h>
#include <ctl/hardware_preset/mcu_board/iris_f280039c_node.h>

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Original ctrl_settings.h includes are intentionally ignored during this SDPE migration trial. */

//=================================================================================================
/**
 * @brief Project metadata.
 */

#define SDPE_PROJECT_ID "pgs_sinv_rc_iris_node"
#define SDPE_PROJECT_SUITE "pgs_sinv_rc"
#define SDPE_PROJECT_VERSION "0.2.0"
#define SDPE_PROJECT_UPDATED_AT "2026-07-15"

//=================================================================================================
/**
 * @brief Controller Features.
 */

/**
 * @brief Enable Discrete PID controller anti-saturation algorithm.
 */
#define _USE_DEBUG_DISCRETE_PID

//=================================================================================================
/**
 * @brief Sensing and Calibration.
 */

/**
 * @brief Enable ADC calibration.
 */
#define SPECIFY_ENABLE_ADC_CALIBRATE

//=================================================================================================
/**
 * @brief Diagnostics and Simulation.
 */

/**
 * @brief Enable PIL simulation function. This macro disables controller output.
 */
// #define ENABLE_GMP_DL_PIL_SIM

/**
 * @brief Enable CiA402 debug information.
 */
// #define GMP_CTL_FM_CONFIG_ENABLE_DEBUG_INFO

//=================================================================================================
/**
 * @brief Control Mode.
 */

/**
 * @brief Single-phase inverter incremental debug build level.
 *        BUILD_LEVEL 1: modulator and resistive-load validation.
 *        BUILD_LEVEL 2: voltage closed-loop validation.
 *        BUILD_LEVEL 3: current-loop and full controller validation.
 *        Options: (1), (2), (3)
 */
#define BUILD_LEVEL (1)

//=================================================================================================
/**
 * @brief PWM Modulator.
 */

/**
 * @brief Use negative PWM modulator logic.
 *        Options: (0), (1)
 */
#define PWM_MODULATOR_USING_NEGATIVE_LOGIC (0)

//=================================================================================================
/**
 * @brief PWM Channel Mapping.
 */

/**
 * @brief PWM base for inverter phase L.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_L_BASE IRIS_EPWM3_BASE

/**
 * @brief PWM base for inverter phase N.
 *        Options: IRIS_EPWM1_BASE, IRIS_EPWM2_BASE, IRIS_EPWM3_BASE, IRIS_EPWM4_BASE, IRIS_EPWM5_BASE, IRIS_EPWM6_BASE
 */
#define PHASE_N_BASE IRIS_EPWM4_BASE

//=================================================================================================
/**
 * @brief Gate Driver GPIO.
 */

/**
 * @brief Gate-driver enable GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_ENABLE_PORT IRIS_GPIO1

/**
 * @brief Gate-driver reset GPIO.
 *        Options: IRIS_GPIO1, IRIS_GPIO2, IRIS_GPIO3, IRIS_GPIO4, IRIS_GPIO5, IRIS_GPIO6
 */
#define PWM_RESET_PORT IRIS_GPIO3

//=================================================================================================
/**
 * @brief Status GPIO.
 */

/**
 * @brief System status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define SYSTEM_LED IRIS_LED1

/**
 * @brief Controller status LED.
 *        Options: IRIS_LED1, IRIS_LED2, LED_R, LED_G
 */
#define CONTROLLER_LED IRIS_LED2

//=================================================================================================
/**
 * @brief AC Current Sensing.
 */

/**
 * @brief AC current ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_IAC_RESULT_BASE ADC_CH1_RESULT_BASE

/**
 * @brief AC current ADC channel.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_IAC ADC_CH1

//=================================================================================================
/**
 * @brief AC Voltage Sensing.
 */

/**
 * @brief AC voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_VAC_RESULT_BASE ADC_CH2_RESULT_BASE

/**
 * @brief AC voltage ADC channel.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_VAC ADC_CH2

//=================================================================================================
/**
 * @brief DC Bus Sensing.
 */

/**
 * @brief DC bus voltage ADC result register base.
 *        Options: ADC_CH1_RESULT_BASE, ADC_CH2_RESULT_BASE, ADC_CH3_RESULT_BASE, ADC_CH4_RESULT_BASE, ADC_CH5_RESULT_BASE, ADC_CH6_RESULT_BASE, ADC_CH7_RESULT_BASE, ADC_CH8_RESULT_BASE, ADC_CH9_RESULT_BASE, ADC_CH10_RESULT_BASE, ADC_CH11_RESULT_BASE, ADC_CH12_RESULT_BASE
 */
#define INV_VBUS_RESULT_BASE ADC_CH3_RESULT_BASE

/**
 * @brief DC bus voltage ADC channel.
 *        Options: ADC_CH1, ADC_CH2, ADC_CH3, ADC_CH4, ADC_CH5, ADC_CH6, ADC_CH7, ADC_CH8, ADC_CH9, ADC_CH10, ADC_CH11, ADC_CH12
 */
#define INV_VBUS ADC_CH3

//=================================================================================================
/**
 * @brief Requirement bindings.
 */

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
 * @brief Nominal AC grid/fundamental frequency in Hz.
 */
#define CTRL_GRID_FREQUENCY (50.0f)

/**
 * @brief Total AC-side filter/grid inductance in H.
 */
#define CTRL_AC_INDUCTANCE (0.003f)

/**
 * @brief Total AC-side series resistance in Ohm.
 */
#define CTRL_AC_RESISTANCE (0.1f)

/**
 * @brief Single-phase PLL proportional gain.
 */
#define CTRL_PLL_KP (10.0f)

/**
 * @brief Single-phase PLL integral time constant in seconds.
 */
#define CTRL_PLL_TI (0.02f)

/**
 * @brief PLL q-axis error low-pass cutoff in Hz.
 */
#define CTRL_PLL_LPF_FC (20.0f)

/**
 * @brief Measured active/reactive power low-pass cutoff in Hz.
 */
#define CTRL_PQ_LPF_FC (200.0f)

/**
 * @brief Peak current-reference limit in per unit.
 */
#define CTRL_CURRENT_LIMIT_PU (1.5f)

/**
 * @brief Minimum PLL voltage magnitude used by P/Q reference division.
 */
#define CTRL_GRID_VMIN_PU (0.1f)

/**
 * @brief Active-power command slew limit in PU/s.
 */
#define CTRL_P_SLEW_PU_S (10.0f)

/**
 * @brief Reactive-power command slew limit in PU/s.
 */
#define CTRL_Q_SLEW_PU_S (20.0f)

/**
 * @brief Current polarity deadband for PWM dead-time compensation.
 */
#define CTRL_CURRENT_DB_PU (0.01f)

/**
 * @brief Minimum fundamental frequency tracked by the repetitive controller in Hz.
 */
#define CTRL_FDRC_MIN_FREQ (45.0f)

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
 * @brief Fast AC peak-current trip threshold in A.
 */
#define CTRL_PROT_IAC_PEAK_MAX (CTRL_MAX_HW_CURRENT * 0.9f * 1.41421356f)

/**
 * @brief Maximum unsaturated modulation command before controller-divergence trip.
 */
#define CTRL_PROT_VCTRL_MAX_PU (1.5f)

/**
 * @brief Minimum physical DC-bus voltage accepted by the startup state machine.
 */
#define CTRL_DCBUS_READY_MIN (CTRL_DCBUS_VOLTAGE * 0.8f)

/**
 * @brief Maximum physical DC-bus voltage accepted by the startup state machine.
 */
#define CTRL_DCBUS_READY_MAX (CTRL_PROT_VBUS_MAX)

/**
 * @brief ADC calibration timeout in ms.
 */
#define TIMEOUT_ADC_CALIB_MS (3000)

/**
 * @brief SPLL close-loop convergence criterion.
 */
#define CTRL_SPLL_EPSILON ((float2ctrl(0.005)))

// User project tail code
/* Compatibility with framework revisions that use the historical misspelling. */
#if defined(ENABLE_GMP_DL_PIL_SIM) && !defined(ENBALE_GMP_DL_PIL_SIM)
#define ENBALE_GMP_DL_PIL_SIM
#endif
#if defined(ENBALE_GMP_DL_PIL_SIM) && !defined(ENABLE_GMP_DL_PIL_SIM)
#define ENABLE_GMP_DL_PIL_SIM
#endif

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_PGS_SINV_RC_IRIS_BINDINGS_H_
