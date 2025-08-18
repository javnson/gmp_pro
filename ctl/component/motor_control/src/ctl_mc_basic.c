/**
 * @file ctl_motor_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Encoder module

#include <ctl/component/motor_control/basic/encoder.h>

// Absolute rotation position encoder
//

void ctl_init_pos_encoder(pos_encoder_t* enc, uint16_t poles, uint32_t position_base)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = position_base;
}

void ctl_init_multiturn_pos_encoder(pos_multiturn_encoder_t* enc, uint16_t poles, uint32_t position_base)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = position_base;
}

void ctl_init_autoturn_pos_encoder(pos_autoturn_encoder_t* enc, uint16_t poles, uint32_t position_base)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = position_base;
}

//
// Speed position encoder
//

//void ctl_init_spd_encoder(spd_encoder_t *enc, parameter_gt speed_base)
//{
//    enc->speed_base = speed_base;
//    enc->encif.speed = 0;
//    enc->speed_krpm = 0;
//}

void ctl_init_spd_calculator(
    // speed calculator objects
    spd_calculator_t* sc,
    // link to a position encoder
    rotation_ift* pos_encif,
    // control law frequency, unit Hz
    parameter_gt control_law_freq,
    // division of control law frequency, unit ticks
    uint32_t speed_calc_div,
    // Speed per unit base value, unit rpm
    parameter_gt rated_speed_rpm,
    // pole pairs, if you pass a elec-angle,
    uint16_t pole_pairs,
    // just set this value to 1.
    // generally, speed_filter_fc approx to speed_calc freq divided by 5
    parameter_gt speed_filter_fc)
{
    uint32_t maximum_div = (uint32_t)rated_speed_rpm / 30;
    if (speed_calc_div < maximum_div)
    {
        maximum_div = speed_calc_div;
    }

    sc->old_position = 0;
    sc->encif.speed = 0;

    sc->scale_factor = float2ctrl(60.0f * control_law_freq / maximum_div / pole_pairs / rated_speed_rpm);
    ctl_init_lp_filter(&sc->spd_filter, control_law_freq / maximum_div, speed_filter_fc);
    ctl_init_divider(&sc->div, maximum_div);

    sc->pos_encif = pos_encif;
}

//////////////////////////////////////////////////////////////////////////
// const f module

#include <ctl/component/motor_control/basic/vf_generator.h>

void ctl_init_const_f_controller(ctl_const_f_controller* ctrl, parameter_gt frequency, parameter_gt isr_freq)
{
    // ctl_setup_ramp_gen(&ctrl->rg, float2ctrl(frequency / isr_freq), 1, 0);

    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, frequency, 1, 0);
}

// Const slope Frequency module

void ctl_init_const_slope_f_controller(
    // controller object
    ctl_slope_f_controller* ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // ISR frequency
    parameter_gt isr_freq)
{
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // init ramp frequency is 0
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    ctrl->target_frequency = frequency / isr_freq;

    ctl_init_slope_limiter(&ctrl->freq_slope, float2ctrl(freq_slope / isr_freq), -float2ctrl(freq_slope / isr_freq),
                           isr_freq);
}

// change target frequency
void ctl_set_slope_f_freq(
    // Const VF controller
    ctl_slope_f_controller* ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq)
{
    ctrl->target_frequency = float2ctrl(target_freq / isr_freq);
}

// VF controller

void ctl_init_const_vf_controller(
    // controller object
    ctl_const_vf_controller* ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // voltage range
    ctrl_gt voltage_bound,
    // Voltage Frequency constant
    // unit p.u./Hz, p.u.
    ctrl_gt voltage_over_frequency, ctrl_gt voltage_bias,
    // ISR frequency
    parameter_gt isr_freq)
{
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // init ramp frequency is 0
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    ctrl->target_frequency = frequency / isr_freq;
    ctrl->target_voltage = 0;

#if !defined CTRL_GT_IS_FIXED
    ctrl->v_over_f = float2ctrl(voltage_over_frequency * isr_freq);
#elif defined CTRL_GT_IS_FLOAT
    ctrl->v_over_f = float2ctrl(voltage_over_frequency * isr_freq / (2 ^ GLOBAL_Q));
#else
#error("The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED.")
#endif // CTRL_GT_IS_XXX
    ctrl->v_bias = voltage_bias;

    ctl_init_slope_limiter(&ctrl->freq_slope, float2ctrl(freq_slope / isr_freq), -float2ctrl(freq_slope / isr_freq),
                           isr_freq);

    ctl_init_saturation(&ctrl->volt_sat, voltage_bound, -voltage_bound);
}

// change target frequency
void ctl_set_const_vf_target_freq(
    // Const VF controller
    ctl_const_vf_controller* ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq)
{
    ctrl->target_frequency = float2ctrl(target_freq / isr_freq);
}
