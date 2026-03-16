#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Slope Limiter

#include <ctl/component/intrinsic/basic/slope_limiter.h>

void ctl_init_slope_limiter(ctl_slope_limiter_t* obj, parameter_gt slope_max, parameter_gt slope_min, parameter_gt fs)
{
    // Error prevention engineering
    gmp_base_assert(slope_min < slope_max);
    gmp_base_assert(fs > 0.0f);

    obj->slope_min = float2ctrl(slope_min / fs);
    obj->slope_max = float2ctrl(slope_max / fs);

    obj->out = float2ctrl(0.0f);
}
