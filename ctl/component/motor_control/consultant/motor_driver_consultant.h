/**
 * @file motor_driver_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines hardware and control parameters for the motor driver.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a "consultant" structure and a set of build-time macros
 * to configure the motor driver's hardware limits, control loop settings,
 * and other operational parameters.
 */

#ifndef _FILE_MOTOR_DRIVER_CONSULTANT_H_
#define _FILE_MOTOR_DRIVER_CONSULTANT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Motor Driver Build-time Parameters                                        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_DRIVER_CONSULTANT Motor Driver Consultant
 * @ingroup MC_DEFINES
 * @brief Defines parameters and structures related to the motor driver hardware and software configuration.
 */

/**
 * @defgroup MC_DRIVER_MACROS Motor Driver Build-time Parameters
 * @ingroup MC_DRIVER_CONSULTANT
 * @brief These macros define default hardware and control parameters.
 *
 * They can be overridden in a project configuration file to tailor the build
 * for specific hardware without modifying the library source code.
 * @{
 */

// --- System Power Ratings ---
#ifndef MOTOR_DRIVER_RATED_DC_VOLTAGE
#define MOTOR_DRIVER_RATED_DC_VOLTAGE ((300.0)) /**< @brief Rated DC bus voltage in Volts (V). */
#endif

// NOTE: There is a typo in the macro name below (CURRNET should be CURRENT).
#ifndef MOTOR_DRIVER_RATED_DC_CURRNET
#define MOTOR_DRIVER_RATED_DC_CURRNET                                                                                  \
    ((100.0)) /**< @brief Rated DC bus current in Amperes (A), used as the base for p.u. calculations. */
#endif

#ifndef MOTOR_DRIVER_MAX_BRIDGE_CURRENT
#define MOTOR_DRIVER_MAX_BRIDGE_CURRENT                                                                                \
    ((100.0)) /**< @brief Maximum transient phase current for the inverter bridge in Amperes (A). */
#endif

#ifndef MOTOR_DRIVER_MAX_DC_CURRENT
#define MOTOR_DRIVER_MAX_DC_CURRENT ((100.0)) /**< @brief Maximum transient DC bus current in Amperes (A). */
#endif

// --- Control Frequencies & Timings ---
#ifndef MOTOR_DRIVER_PWM_FREQUENCY
#define MOTOR_DRIVER_PWM_FREQUENCY ((10e3)) /**< @brief PWM switching frequency in Hertz (Hz). */
#endif

#ifndef MOTOR_DRIVER_CONTROL_FREQUENCY
#define MOTOR_DRIVER_CONTROL_FREQUENCY ((10e3)) /**< @brief Main control loop frequency in Hertz (Hz). */
#endif

#ifndef MOTOR_DRIVER_PWM_HALF_CYCLE
#define MOTOR_DRIVER_PWM_HALF_CYCLE ((10000)) /**< @brief PWM timer half-period in timer ticks (for symmetric PWM). */
#endif

// --- Controller Bandwidths & Divisions ---
#ifndef MOTOR_DRIVER_CURRENT_BW
#define MOTOR_DRIVER_CURRENT_BW ((200)) /**< @brief Target bandwidth for the current control loop in Hertz (Hz). */
#endif

#ifndef MOTOR_DRIVER_SPEED_BW
#define MOTOR_DRIVER_SPEED_BW ((50)) /**< @brief Target bandwidth for the speed control loop in Hertz (Hz). */
#endif

#ifndef MOTOR_DRIVER_SPEED_DIV
#define MOTOR_DRIVER_SPEED_DIV                                                                                         \
    ((10)) /**< @brief Division factor for the speed controller execution rate relative to the main control frequency. */
#endif

#ifndef MOTOR_DRIVER_SPEED_CALC_DIV
#define MOTOR_DRIVER_SPEED_CALC_DIV ((10)) /**< @brief Division factor for the speed calculation execution rate. */
#endif

// --- ADC & Sensor Parameters ---
#ifndef MOTOR_DRIVER_ADC_RESOLUTION
#define MOTOR_DRIVER_ADC_RESOLUTION ((16)) /**< @brief Resolution of the ADC in bits. */
#endif

#ifndef MOTOR_DRIVER_ADC_VOLTAGE_FULL_SCALE
#define MOTOR_DRIVER_ADC_VOLTAGE_FULL_SCALE                                                                            \
    ((750.0)) /**< @brief Full-scale measurement range for phase voltage sensing in Volts (V). */
#endif

#ifndef MOTOR_DIRVER_ADC_VOLTAGE_BIAS_PU
#define MOTOR_DIRVER_ADC_VOLTAGE_BIAS_PU                                                                               \
    (0.5) /**< @brief Bias of the phase voltage ADC in per-unit (e.g., 0.5 for mid-range). */
#endif

#ifndef MOTOR_DRIVER_ADC_CURRENT_FULL_SCALE
#define MOTOR_DRIVER_ADC_CURRENT_FULL_SCALE                                                                            \
    ((300.0)) /**< @brief Full-scale measurement range for phase current sensing in Amperes (A). */
#endif

#ifndef MOTOR_DIRVER_ADC_CURRENT_BIAS_PU
#define MOTOR_DIRVER_ADC_CURRENT_BIAS_PU (0.5) /**< @brief Bias of the phase current ADC in per-unit. */
#endif

#ifndef MOTOR_DRIVER_ADC_VOLTAGE_DC_FULL_SCALE
#define MOTOR_DRIVER_ADC_VOLTAGE_DC_FULL_SCALE                                                                         \
    ((450.0)) /**< @brief Full-scale measurement range for DC bus voltage sensing in Volts (V). */
#endif

#ifndef MOTOR_DIRVER_ADC_VOLTAGE_DC_BIAS_PU
#define MOTOR_DIRVER_ADC_VOLTAGE_DC_BIAS_PU (0.15) /**< @brief Bias of the DC bus voltage ADC in per-unit. */
#endif

#ifndef MOTOR_DRIVER_ADC_CURRENT_DC_FULL_SCALE
#define MOTOR_DRIVER_ADC_CURRENT_DC_FULL_SCALE                                                                         \
    ((30.0)) /**< @brief Full-scale measurement range for DC bus current sensing in Amperes (A). */
#endif

#ifndef MOTOR_DIRVER_ADC_CURRENT_DC_BIAS_PU
#define MOTOR_DIRVER_ADC_CURRENT_DC_BIAS_PU (0.15) /**< @brief Bias of the DC bus current ADC in per-unit. */
#endif

// --- Motion Profile Limits ---
#ifndef MOTOR_DRIVER_ENCODER_BASE
#define MOTOR_DRIVER_ENCODER_BASE                                                                                      \
    ((1                                                                                                                \
      << 17)) /**< @brief The base value for a full mechanical revolution of the position encoder (e.g., 2^17 for a 17-bit encoder). */
#endif

#ifndef MOTOR_DRIVER_ACCLERATION
#define MOTOR_DRIVER_ACCLERATION ((3000)) /**< @brief Maximum rotational acceleration limit in RPM/s. */
#endif

#ifndef MOTOR_DRIVER_JERK
#define MOTOR_DRIVER_JERK ((0.02)) /**< @brief Maximum jerk limit in p.u./ms. */
#endif

/** @} */ // end of MC_DRIVER_MACROS group

/*---------------------------------------------------------------------------*/
/* Motor Driver Configuration Structure                                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_DRIVER_STRUCT Motor Driver Configuration Structure
 * @ingroup MC_DRIVER_CONSULTANT
 * @brief A structure to hold all runtime-configurable driver parameters.
 * @{
 */

/**
 * @brief Data structure for holding all motor driver parameters.
 *
 * This structure aggregates all the key hardware and software parameters
 * for the motor driver, which can be configured at runtime.
 */
typedef struct _tag_motor_driver_consultant_t
{
    // Power and System Ratings
    parameter_gt rated_dc_voltage;  /**< @brief Rated DC bus voltage (V), base for p.u. voltage. */
    parameter_gt rated_dc_current;  /**< @brief Rated DC bus current (A), base for p.u. current. */
    parameter_gt max_phase_current; /**< @brief Maximum transient phase current (A). */
    parameter_gt max_dc_current;    /**< @brief Maximum transient DC bus current (A). */

    // Frequencies and Timings
    parameter_gt pwm_freq;         /**< @brief PWM switching frequency (Hz). */
    parameter_gt control_law_freq; /**< @brief Main control loop frequency (Hz). */
    uint32_t pwm_half_cycle;       /**< @brief PWM timer half-period in ticks. */

    // Control Loop Parameters
    parameter_gt current_closeloop_bw; /**< @brief Current control loop bandwidth (Hz). */
    parameter_gt speed_closeloop_bw;   /**< @brief Speed control loop bandwidth (Hz). */
    uint32_t speed_div_times;          /**< @brief Speed controller execution frequency division factor. */
    uint32_t speed_calc_div_times;     /**< @brief Speed calculation frequency division factor. */

    // ADC and Sensor Parameters
    uint16_t adc_resolution_bit;            /**< @brief ADC resolution in bits. */
    parameter_gt adc_voltage_full_scale;    /**< @brief Phase voltage ADC full-scale range (V). */
    parameter_gt adc_voltage_bias;          /**< @brief Phase voltage ADC bias (p.u.). */
    parameter_gt adc_current_full_scale;    /**< @brief Phase current ADC full-scale range (A). */
    parameter_gt adc_current_bias;          /**< @brief Phase current ADC bias (p.u.). */
    parameter_gt adc_dc_voltage_full_scale; /**< @brief DC bus voltage ADC full-scale range (V). */
    parameter_gt adc_dc_voltage_bias;       /**< @brief DC bus voltage ADC bias (p.u.). */
    parameter_gt adc_dc_current_full_scale; /**< @brief DC bus current ADC full-scale range (A). */
    parameter_gt adc_dc_current_bias;       /**< @brief DC bus current ADC bias (p.u.). */

    // Motion Parameters
    uint32_t position_enc_base; /**< @brief Position encoder counts per revolution. */
    parameter_gt acceleration;  /**< @brief Acceleration limit (RPM/s). */
    parameter_gt jerk;          /**< @brief Jerk limit (p.u./ms). */

} ctl_motor_driver_consultant_t;

/**
 * @brief A macro to initialize the `ctl_motor_driver_consultant_t` structure from the build-time macros.
 */
#define MOTOR_DRIVER_CONSULTANT_WRAPPER                                                                                \
    {                                                                                                                  \
        MOTOR_DRIVER_RATED_DC_VOLTAGE, MOTOR_DRIVER_RATED_DC_CURRNET, MOTOR_DRIVER_MAX_BRIDGE_CURRENT,                 \
            MOTOR_DRIVER_MAX_DC_CURRENT, MOTOR_DRIVER_PWM_FREQUENCY, MOTOR_DRIVER_CONTROL_FREQUENCY,                   \
            MOTOR_DRIVER_PWM_HALF_CYCLE, MOTOR_DRIVER_CURRENT_BW, MOTOR_DRIVER_SPEED_BW, MOTOR_DRIVER_SPEED_DIV,       \
            MOTOR_DRIVER_SPEED_CALC_DIV, MOTOR_DRIVER_ADC_RESOLUTION, MOTOR_DRIVER_ADC_VOLTAGE_FULL_SCALE,             \
            MOTOR_DIRVER_ADC_VOLTAGE_BIAS_PU, MOTOR_DRIVER_ADC_CURRENT_FULL_SCALE, MOTOR_DIRVER_ADC_CURRENT_BIAS_PU,   \
            MOTOR_DRIVER_ADC_VOLTAGE_DC_FULL_SCALE, MOTOR_DIRVER_ADC_VOLTAGE_DC_BIAS_PU,                               \
            MOTOR_DRIVER_ADC_CURRENT_DC_FULL_SCALE, MOTOR_DIRVER_ADC_CURRENT_DC_BIAS_PU, MOTOR_DRIVER_ENCODER_BASE,    \
            MOTOR_DRIVER_ACCLERATION, MOTOR_DRIVER_JERK                                                                \
    }

/** @} */ // end of MC_DRIVER_STRUCT group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_DRIVER_CONSULTANT_H_
