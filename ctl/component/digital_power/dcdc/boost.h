/**
 * @file boost.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides a generic cascaded PID controller for a Boost converter.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 * 
 */

#include <ctl/component/interface/pwm_channel.h>

#include <ctl/component/digital_power/dcdc/dcdc_core.h>

#ifndef _FILE_BOOST_CTRL_H_
#define _FILE_BOOST_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Complete hardware, specification, and operational boundary dynamics for Boost auto-tuning.
 */
typedef struct _tag_boost_hardware_t
{
    parameter_gt fs; /**< System sampling and loop execution frequency (Hz). */

    /* Plant Physical Assets */
    parameter_gt v_in_min;      /**< Minimum operational input DC voltage (V) - [Worst Case for RHPZ]. */
    parameter_gt v_out_nominal; /**< Target nominal output DC voltage (V). */
    parameter_gt L_henry;       /**< Main boost storage inductor value (Henry). */
    parameter_gt R_esr_ohm;     /**< Inductor equivalent series resistance (Ohm). */
    parameter_gt C_farad;       /**< Total output filter parallel capacitance (Farad). */
    parameter_gt R_load_min;    /**< Minimum rated equivalent resistive load (Ohm) - [Worst Case Heavy Load]. */

    /* Normalization Bases */
    parameter_gt v_base; /**< ADC Voltage normalization base (V corresponding to 1.0 PU). */
    parameter_gt i_base; /**< ADC Current normalization base (A corresponding to 1.0 PU). */

    /* Operational Constraints */
    parameter_gt slope_v_pu_s; /**< Target voltage soft-start ramp rate (PU/sec). */
    parameter_gt slope_i_pu_s; /**< Target current protection ramp rate (PU/sec). */
    ctrl_gt i_out_max;         /**< Explicit maximum positive current saturation limit (PU). */
    ctrl_gt i_out_min;         /**< Explicit maximum reverse current saturation limit (PU). */

    /* Loop Dynamic Goals */
    parameter_gt fc_current_loop; /**< Target cross-over frequency for the inner current loop (Hz). */
    parameter_gt fc_voltage_loop; /**< Target desired cross-over frequency for the outer voltage loop (Hz). */
} ctl_boost_hardware_t;

/*---------------------------------------------------------------------------*/
/* Exported Blueprint Service & Calculation APIs                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Evaluates the worst-case Right Half Plane Zero (RHPZ) frequency for the given Boost hardware.
 * @details Calculates the RHPZ frequency at maximum duty cycle (minimum input voltage) and heavy load.
 * Formula: f_rhpz = (R_load_min * (1 - D)^2) / (2 * pi * L)
 * @param[in] hw Pointer to the comprehensive boost hardware asset structure.
 * @return parameter_gt The worst-case RHPZ frequency in Hz.
 */
parameter_gt ctl_boost_calc_rhp_zero(const ctl_boost_hardware_t* hw);

/**
 * @brief Computes exact PID coefficients mapped from physical boost assets with automatic RHPZ clamping.
 * @details Checks the user's desired fc_voltage_loop against the f_rhpz limit. If the desired bandwidth 
 * exceeds 1/5 of the RHPZ frequency, it is automatically throttled down to a safe value.
 * @param[out] init_config Pointer to the destination configuration profile to be populated.
 * @param[in] hw Pointer to the comprehensive boost hardware asset boundary structure.
 */
void ctl_dcdc_blueprint_boost_cascade(ctl_dcdc_core_init_t* init_config, const ctl_boost_hardware_t* hw);

/*---------------------------------------------------------------------------*/
/* Boost Modulator                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Boost Modulator.
 */
typedef struct _tag_boost_modulator_t
{
    pwm_channel_t pwm;
    ctrl_gt duty_max; //!< Max duty for bottom switch (prevents continuous inductor short).
    ctrl_gt duty_min; //!< Min duty to ensure bootstrap capacitor charges.
} boost_modulator_t;

GMP_STATIC_INLINE void ctl_init_boost_modulator(boost_modulator_t* mod, pwm_gt full_scale, ctrl_gt duty_max,
                                                ctrl_gt duty_min)
{
    ctl_init_pwm_channel(&mod->pwm, 0, full_scale);
    mod->duty_max = duty_max;
    mod->duty_min = duty_min;
}

GMP_STATIC_INLINE pwm_gt ctl_step_boost_modulator(boost_modulator_t* mod, ctrl_gt v_req, ctrl_gt v_out)
{
    ctrl_gt v_out_safe = (v_out > float2ctrl(0.1f)) ? v_out : float2ctrl(0.1f);
    ctrl_gt duty = float2ctrl(1.0f) - ctl_div(v_req, v_out_safe);

    duty = ctl_sat(duty, mod->duty_max, mod->duty_min);
    return ctl_step_pwm_channel(&mod->pwm, duty);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BOOST_CTRL_H_
