#include <ctl/math_block/gmp_math.h>

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
    parameter_gt b1 = ki / CTL_PARAM_CONST_2 / fs - CTL_PARAM_CONST_2 * kd * fs;
    parameter_gt b0 = kd * fs + ki / CTL_PARAM_CONST_2 / fs;

    pid->b2 = param2ctrl(b2);
    pid->b1 = param2ctrl(b1);
    pid->b0 = param2ctrl(b0);

    pid->output_max = CTL_CTRL_CONST_1;
    pid->output_min = (-CTL_CTRL_CONST_1);

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
    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO);

    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    // Disable the integral term safely when Ti is effectively zero.
    parameter_gt ki = CTL_PARAM_CONST_ZERO;
    if (Ti > CTL_PARAM_CONST_EPSILON)
    {
        ki = kp / Ti;
    }

    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / CTL_PARAM_CONST_2 / fs - kp - CTL_PARAM_CONST_2 * kd * fs;
    parameter_gt b0 = kp + kd * fs + ki / CTL_PARAM_CONST_2 / fs;

    pid->b2 = param2ctrl(b2);
    pid->b1 = param2ctrl(b1);
    pid->b0 = param2ctrl(b0);

    pid->output_max = CTL_CTRL_CONST_1;
    pid->output_min = (-CTL_CTRL_CONST_1);

    ctl_clear_discrete_pid(pid);
}

#endif // _USE_DEBUG_DISCRETE_PID
