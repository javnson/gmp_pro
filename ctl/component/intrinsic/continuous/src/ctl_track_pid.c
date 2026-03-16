#include <gmp_core.h>

#include <ctl/component/intrinsic/continuous/track_pid.h>

void ctl_init_tracking_continuous_pid(ctl_tracking_continuous_pid_t* tp, parameter_gt kp, parameter_gt Ti,
                                      parameter_gt Td, ctrl_gt sat_max, ctrl_gt sat_min, parameter_gt slope_max,
                                      parameter_gt slope_min, uint32_t division, parameter_gt fs)
{
    // 1. Error prevention engineering
    gmp_base_assert(slope_min < slope_max);
    gmp_base_assert(sat_min < sat_max);
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(division >= 1);

    // 2. Calculate EFFECTIVE execution frequency
    parameter_gt effective_fs = fs / (parameter_gt)division;

    // 3. Initialize sub-modules using the effective frequency
    ctl_init_slope_limiter(&tp->traj, slope_max, slope_min, effective_fs);
    ctl_init_divider(&tp->div, division);

    // 4. Default D-term filter time constant (Safe industrial default)
    // Roughly 1/5th of the effective Nyquist frequency, suppresses high-frequency noise.
    parameter_gt default_Tf = 5.0f / effective_fs;

    // 5. Initialize the advanced Anti-Windup Series PID
    ctl_init_pid_aw_ser(&tp->pid, kp, Ti, Td, default_Tf, effective_fs);

    // Set PID output limits
    tp->pid.out_max = sat_max;
    tp->pid.out_min = sat_min;
}

void ctl_set_tracking_continuous_pid_filter(ctl_tracking_continuous_pid_t* tp, parameter_gt Tf, uint32_t division,
                                            parameter_gt fs)
{
    parameter_gt effective_fs = fs / (parameter_gt)division;

    if (Tf <= 0.0f)
    {
        tp->pid.alpha_d = float2ctrl(1.0f); // Disable filtering
    }
    else
    {
        tp->pid.alpha_d = float2ctrl((1.0f / effective_fs) / (Tf + (1.0f / effective_fs)));
    }
}
