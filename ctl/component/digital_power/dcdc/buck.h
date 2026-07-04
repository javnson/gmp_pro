/**
 * @file buck.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides a generic cascaded PID controller for a Buck converter.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ctl/component/interface/pwm_channel.h>
#include <ctl/component/digital_power/dcdc/dcdc_core.h>

#ifndef _FILE_BUCK_CTRL_H_
#define _FILE_BUCK_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Complete hardware, specification, and target dynamics for Buck auto-tuning.
 */
typedef struct _tag_buck_hardware_t
{
    parameter_gt fs; /**< System sampling and loop execution frequency (Hz). */

    /* Plant Physical Assets */
    parameter_gt v_in_nominal;   /**< Nominal input DC link voltage (V). */
    parameter_gt L_henry;        /**< Main storage power inductor value (Henry). */
    parameter_gt R_esr_ohm;      /**< Inductor equivalent series resistance (Ohm). */
    parameter_gt C_farad;        /**< Total output filter parallel capacitance (Farad). */
    parameter_gt R_load_nominal; /**< Nominal rated equivalent resistive load (Ohm). */

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
    parameter_gt fc_voltage_loop; /**< Target cross-over frequency for the outer voltage loop (Hz). */
} ctl_buck_hardware_t;

/**
 * @brief Computes exact PID coefficients mapped from physical plant assets and load specifications.
 * @param[out] init_config Pointer to the destination configuration profile to be populated.
 * @param[in] hw Pointer to the comprehensive hardware asset boundary structure.
 */
void ctl_dcdc_blueprint_buck_cascade(ctl_dcdc_core_init_t* init_config, const ctl_buck_hardware_t* hw);

/*---------------------------------------------------------------------------*/
/* 1. Buck Modulator                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Buck Modulator.
 */
typedef struct _tag_buck_modulator_t
{
    pwm_channel_t pwm;
    ctrl_gt duty_max; //!< Max duty to ensure bottom switch turns on for bootstrap.
    ctrl_gt duty_min; //!< Min duty to ensure top switch minimum ON time.
} buck_modulator_t;

GMP_STATIC_INLINE void ctl_init_buck_modulator(buck_modulator_t* mod, pwm_gt full_scale, ctrl_gt duty_max,
                                               ctrl_gt duty_min)
{
    ctl_init_pwm_channel(&mod->pwm, 0, full_scale);
    mod->duty_max = duty_max;
    mod->duty_min = duty_min;
}

GMP_STATIC_INLINE pwm_gt ctl_step_buck_modulator(buck_modulator_t* mod, ctrl_gt v_req, ctrl_gt v_in)
{
    ctrl_gt v_in_safe = (v_in > float2ctrl(0.1f)) ? v_in : float2ctrl(0.1f);
    ctrl_gt duty = ctl_div(v_req, v_in_safe);

    // Strict limits for Bootstrap and Min-On time
    duty = ctl_sat(duty, mod->duty_max, mod->duty_min);
    return ctl_step_pwm_channel(&mod->pwm, duty);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BUCK_CTRL_H_
