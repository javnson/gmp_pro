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
