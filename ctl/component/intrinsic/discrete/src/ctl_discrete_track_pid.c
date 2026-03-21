#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Discrete track pid

#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>

void ctl_init_tracking_pid(
    // pointer to track pid object
    ctl_tracking_discrete_pid_t* tp,
    // pid parameters, unit sec
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // saturation limit
    ctrl_gt sat_max, ctrl_gt sat_min,
    // slope limit, unit: p.u./sec
    parameter_gt slope_max, parameter_gt slope_min,
    // division factor:
    uint32_t division,
    // controller frequency, unit Hz
    parameter_gt fs)
{
    // Error prevention engineering
    gmp_base_assert(slope_min < slope_max);
    gmp_base_assert(sat_min < sat_max);
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(division >= 1); // 确保分频系数合法

    parameter_gt effective_fs = fs / (parameter_gt)division;

    ctl_init_slope_limiter(&tp->traj, slope_max, slope_min, effective_fs);
    ctl_init_divider(&tp->div, division);

    ctl_init_discrete_pid(&tp->pid, kp, Ti, Td, effective_fs);
    ctl_set_discrete_pid_limit(&tp->pid, sat_max, sat_min);
}
