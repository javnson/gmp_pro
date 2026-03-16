#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Discrete PID controller

#include <ctl/component/intrinsic/discrete/discrete_pid.h>
#ifdef _USE_DEBUG_DISCRETE_PID
void ctl_init_discrete_pid(
    // pointer to pid object
    discrete_pid_t* pid,
    // gain of the pid controller
    parameter_gt kp,
    // Time constant for integral and differential part, unit Hz
    parameter_gt Ti, parameter_gt Td,
    // sample frequency, unit Hz
    parameter_gt fs)
{
    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    parameter_gt ki = kp / Ti;
    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / 2.0f / fs - 2.0f * kd * fs;
    parameter_gt b0 = kd * fs + ki / 2.0f / fs;

    pid->kp = float2ctrl(kp);

    pid->b2 = float2ctrl(b2);
    pid->b1 = float2ctrl(b1);
    pid->b0 = float2ctrl(b0);

    pid->output_max = float2ctrl(1.0);
    pid->output_min = float2ctrl(-1.0);

    ctl_clear_discrete_pid(pid);
}
#else // _USE_DEBUG_DISCRETE_PID
void ctl_init_discrete_pid(
    // pointer to pid object
    discrete_pid_t* pid,
    // gain of the pid controller
    parameter_gt kp,
    // Time constant for integral and differential part, unit Hz
    parameter_gt Ti, parameter_gt Td,
    // sample frequency, unit Hz
    parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    // 1. 处理积分参数与防除零
    parameter_gt ki = 0.0;
    if (Ti > 1e-6) // 如果 Ti 极小，视为关闭积分
    {
        ki = kp / Ti;
    }

    parameter_gt ki = kp / Ti;
    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / 2.0f / fs - kp - 2.0f * kd * fs;
    parameter_gt b0 = kp + kd * fs + ki / 2.0f / fs;

    pid->kp = float2ctrl(kp);

    pid->b2 = float2ctrl(b2);
    pid->b1 = float2ctrl(b1);
    pid->b0 = float2ctrl(b0);

    pid->output_max = float2ctrl(1.0);
    pid->output_min = float2ctrl(-1.0);

    ctl_clear_discrete_pid(pid);
}

#endif // _USE_DEBUG_DISCRETE_PID
