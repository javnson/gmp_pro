/**
 * @file contant_vf.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifndef _FILE_CONST_VF_H_
#define _FILE_CONST_VF_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

//////////////////////////////////////////////////////////////////////////
// generate constant frequency
typedef struct _tag_const_f
{
    // encoder output
    rotation_ift enc;

    // ramp generator
    ctl_ramp_generator_t rg;

} ctl_const_f_controller;

void ctl_init_const_f_controller(ctl_const_f_controller *ctrl, parameter_gt frequency, parameter_gt isr_freq);

GMP_STATIC_INLINE
void ctl_step_const_f_controller(ctl_const_f_controller *ctrl)
{
    ctrl->enc.elec_position = ctl_step_ramp_generator(&ctrl->rg);
    ctrl->enc.position = ctrl->enc.elec_position;
}

//////////////////////////////////////////////////////////////////////////

// This module generate a constant slope frequency generator.
// When user provide a target frequency this module will change
// to this frequency stepwise.
typedef struct _tag_slope_f
{
    // encoder port
    rotation_ift enc;

    // input: target frequency
    // unit ticks / cycle
    ctrl_gt target_frequency;

    // output: current frequency
    // unit per unit
    ctrl_gt current_freq;

    // ramp_generator
    ctl_ramp_generator_t rg;

    // slope limit
    ctl_slope_limiter_t freq_slope;
} ctl_slope_f_controller;

void ctl_init_const_slope_f_controller(
    // controller object
    ctl_slope_f_controller *ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // ISR frequency
    parameter_gt isr_freq);

// return target voltage amplitude
GMP_STATIC_INLINE
ctrl_gt ctl_step_slope_f(ctl_slope_f_controller *ctrl)
{
    // step to next frequency
    ctrl->current_freq = ctl_step_slope_limiter(&ctrl->freq_slope, ctrl->target_frequency);

    // change ramp target
    ctl_set_ramp_generator_slope(&ctrl->rg, ctrl->current_freq);

    // move to next angle position
    ctrl->enc.elec_position = ctl_step_ramp_generator(&ctrl->rg);
    ctrl->enc.position = ctrl->enc.elec_position;

    return ctrl->enc.elec_position;
}

GMP_STATIC_INLINE
void ctl_clear_slope_f(ctl_slope_f_controller *ctrl)
{
    ctl_clear_slope_limiter(&ctrl->freq_slope);
}

// change target frequency
void ctl_set_slope_f_freq(
    // Const VF controller
    ctl_slope_f_controller *ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq);

//////////////////////////////////////////////////////////////////////////
//
// This modle generate a constant V/F profile
// User may use this module in Open loop mode or in ACM controller.
typedef struct _tag_const_vf
{
    // encoder output
    rotation_ift enc;

    // input: target frequency
    // unit ticks / cycle
    ctrl_gt target_frequency;

    // output: target voltage
    // unit: per unit
    ctrl_gt target_voltage;

    // output: current frequency
    // unit per unit
    ctrl_gt current_freq;

    // parameters
    // frequency dead band
    ctrl_gt freq_deadband;

    // parameters
    // E = 4.44 N \Phi \times f = v_over_f * f
    ctrl_gt v_over_f;

    // parameters
    // E output = saturation(v_over_f * frequency + v_bias)
    ctrl_gt v_bias;

    // ramp generator
    ctl_ramp_generator_t rg;

    // slope limit
    ctl_slope_limiter_t freq_slope;

    // saturation limit for Voltage
    // [-voltage_bound, volatage bound]
    ctl_saturation_t volt_sat;

} ctl_const_vf_controller;

// init const vf controller object
void ctl_init_const_vf_controller(
    // controller object
    ctl_const_vf_controller *ctrl,
    // ISR frequency
    parameter_gt isr_freq,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // voltage range
    ctrl_gt voltage_bound,
    // Voltage Frequency constant
    // unit p.u./Hz, p.u.
    ctrl_gt voltage_over_frequency, ctrl_gt voltage_bias);

// return target voltage amplitude
GMP_STATIC_INLINE
ctrl_gt ctl_step_const_vf(ctl_const_vf_controller *ctrl)
{
    // step to next frequency
    ctrl->current_freq = ctl_step_slope_limiter(&ctrl->freq_slope, ctrl->target_frequency);

    // calculate target voltage
    if (ctrl->current_freq > ctrl->freq_deadband)
    {
        ctrl->target_voltage =
            ctl_step_saturation(&ctrl->volt_sat, ctrl->v_bias + ctl_mul(ctrl->v_over_f, ctrl->current_freq));
    }
    else if (ctrl->current_freq < -ctrl->freq_deadband)
    {
        ctrl->target_voltage =
            ctl_step_saturation(&ctrl->volt_sat, -ctrl->v_bias - ctl_mul(ctrl->v_over_f, ctrl->current_freq));
    }
    // dead band
    else
    {
        ctrl->target_voltage = 0;
    }

    // change ramp target
    ctl_set_ramp_generator_slope(&ctrl->rg, ctrl->current_freq);

    // move to next angle position
    ctrl->enc.elec_position = ctl_step_ramp_generator(&ctrl->rg);
    ctrl->enc.position = ctrl->enc.elec_position;

    return ctrl->enc.elec_position;
}

// change target frequency
void ctl_set_const_vf_target_freq(
    // Const VF controller
    ctl_const_vf_controller *ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONST_VF_H_
