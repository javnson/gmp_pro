#include <gmp_core.h>

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
    gmp_base_assert(fs > 0.0f);

    hpid->kp = float2ctrl(kp);

    if (Ti <= 0.000001f)
    {
        hpid->ki = float2ctrl(0.0f);
    }
    else
    {
        hpid->ki = float2ctrl(1.0f / (fs * Ti));
    }

    hpid->kd = float2ctrl(1.0f * fs * Td);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->integral_min = float2ctrl(-0.8f);
    hpid->integral_max = float2ctrl(0.8f);

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
    hpid->kp = float2ctrl(kp);
    hpid->ki = float2ctrl(ki / fs);
    hpid->kd = float2ctrl(1.0f * fs * kd);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    hpid->integral_min = float2ctrl(-0.8f);
    hpid->integral_max = float2ctrl(0.8f);

    ctl_clear_pid(hpid);
}
