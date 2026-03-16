#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Saturation

#include <ctl/component/intrinsic/basic/saturation.h>

void ctl_init_saturation(ctl_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    // Error prevention engineering
    gmp_base_assert(out_min < out_max);

    obj->out_min = out_min;
    obj->out_max = out_max;
}

void ctl_init_bipolar_saturation(ctl_bipolar_saturation_t* obj, ctrl_gt out_min, ctrl_gt out_max)
{
    // Error prevention engineering
    gmp_base_assert(out_min < out_max);

    obj->out_min = out_min;
    obj->out_max = out_max;
    obj->out = 0;
}

void ctl_init_atanh_saturation(ctl_atan_saturation_t* sat, ctrl_gt gain, ctrl_gt scale_factor)
{
    sat->gain = ctl_mul(gain, CTL_CTRL_CONST_2_OVER_PI);
    sat->sf = scale_factor;
    sat->out = 0;
}
