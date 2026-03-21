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

#include <ctl/component/motor_control/interface/encoder.h>

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

    sc->scale_factor = float2ctrl(60.0f * control_law_freq / maximum_div / rated_speed_rpm);
    ctl_init_lp_filter(&sc->spd_filter, control_law_freq / maximum_div, speed_filter_fc);
    ctl_init_divider(&sc->div, maximum_div);

    sc->pos_encif = pos_encif;
}

void ctl_init_spd_calculator_elecpos(
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

