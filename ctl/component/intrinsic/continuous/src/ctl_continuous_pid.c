#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// PID regular

#include <ctl/component/intrinsic/continuous/continuous_pid.h>

// init a parallel PID object
void ctl_init_pid_Tmode(
    // continuous pid handle
    ctl_pid_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // controller frequency
    parameter_gt fs)
{
    parameter_gt ki;
    parameter_gt kd;

    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO);
    ki = (Ti <= CTL_PARAM_CONST_EPSILON) ? CTL_PARAM_CONST_ZERO : CTL_PARAM_CONST_1 / (fs * Ti);
    kd = fs * Td;

    hpid->kp = param2ctrl(kp);
    hpid->ki = param2ctrl(ki);
    hpid->kd = param2ctrl(kd);
    hpid->out_min = -CTL_CTRL_CONST_1;
    hpid->out_max = CTL_CTRL_CONST_1;
    hpid->integral_min = real2ctrl(-0.8);
    hpid->integral_max = real2ctrl(0.8);

    ctl_clear_pid(hpid);
}

void ctl_init_pid(
    // continuous pid handle
    ctl_pid_t* hpid,
    // PID parameters
    parameter_gt kp, parameter_gt ki, parameter_gt kd,
    // controller frequency
    parameter_gt fs)
{
    parameter_gt ki_per_sample;
    parameter_gt kd_per_sample;

    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO);
    ki_per_sample = ki / fs;
    kd_per_sample = fs * kd;

    hpid->kp = param2ctrl(kp);
    hpid->ki = param2ctrl(ki_per_sample);
    hpid->kd = param2ctrl(kd_per_sample);
    hpid->out_min = -CTL_CTRL_CONST_1;
    hpid->out_max = CTL_CTRL_CONST_1;
    hpid->integral_min = real2ctrl(-0.8);
    hpid->integral_max = real2ctrl(0.8);

    ctl_clear_pid(hpid);
}
