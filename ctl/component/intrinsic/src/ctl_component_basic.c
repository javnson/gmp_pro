/**
 * @file ctl_component_basic.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 1.05
 * @date 2025-03-19
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Saturation

#include <ctl/component/intrinsic/basic/saturation.h>

void ctl_init_saturation(ctl_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    obj->out_min = out_min;
    obj->out_max = out_max;
}

void ctl_init_bipolar_saturation(ctl_bipolar_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    obj->out_min = out_min;
    obj->out_max = out_max;
    obj->out = 0;
}

void ctl_init_tanh_saturation(ctl_atan_saturation_t* sat, ctrl_gt gain, ctrl_gt scale_factor)
{
    sat->gain = ctl_mul(gain, CTL_CTRL_CONST_1_OVER_2PI);
    sat->sf = scale_factor;
    sat->out = 0;
}

//////////////////////////////////////////////////////////////////////////
// Divider

#include <ctl/component/intrinsic/basic/divider.h>

void ctl_init_divider(ctl_divider_t* obj, uint32_t counter_period)
{
    // Current counter
    obj->counter = 0;

    obj->target = counter_period;
}

//////////////////////////////////////////////////////////////////////////
// Slope Limiter

#include <ctl/component/intrinsic/basic/slope_limiter.h>

void ctl_init_slope_limiter(ctl_slope_limiter_t* obj, parameter_gt slope_max, parameter_gt slope_min, parameter_gt fs)
{
    obj->slope_min = float2ctrl(slope_min / fs);
    obj->slope_max = float2ctrl(slope_max / fs);

    obj->out = float2ctrl(0);
}

//////////////////////////////////////////////////////////////////////////
// HCC regular

#include <ctl/component/intrinsic/basic/hysteresis_controller.h>

void ctl_init_hysteresis_controller(ctl_hysteresis_controller_t* hcc, fast_gt flag_polarity, ctrl_gt half_width)
{
    hcc->flag_polarity = flag_polarity;
    hcc->half_width = half_width;
    hcc->target = 0;
    hcc->current = 0;
    // Initialize switch output to the state opposite of the upper bound state
    // to ensure predictable startup behavior.
    hcc->switch_out = 1 - flag_polarity;
}
