#include <ctl/component/intrinsic/continuous/continuous_pid_aw.h>
#include <gmp_core.h>

// init a parallel PID (Kp, Ki, Kd act independently)
void ctl_init_pid_aw_par(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt Tf,
                         parameter_gt fs)
{
    gmp_base_assert(fs > 0.0f);

    hpid->kp = float2ctrl(kp);

    // Safety for divide-by-zero
    if (Ti <= 0.000001f)
    {
        hpid->ki = float2ctrl(0.0f);
    }
    else
    {
        hpid->ki = float2ctrl(kp / (fs * Ti));
    }

    hpid->kd = float2ctrl(kp * fs * Td);

    // Derivative low-pass filter coefficient
    if (Tf <= 0.0f)
    {
        hpid->alpha_d = float2ctrl(1.0f); // No filter
    }
    else
    {
        hpid->alpha_d = float2ctrl((1.0f / fs) / (Tf + (1.0f / fs)));
    }

    // Set anti-windup parameter back-calculation gain based on kp
    if (kp < 0.7f)
        hpid->kc = float2ctrl(1.3f);
    else if (kp > 2.0f)
        hpid->kc = float2ctrl(0.5f);
    else
        hpid->kc = float2ctrl(1.0f / kp);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    ctl_clear_pid_aw(hpid);
}

// init a Series PID (Kp scales the sum of P, I, and D)
void ctl_init_pid_aw_ser(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt Tf,
                         parameter_gt fs)
{
    gmp_base_assert(fs > 0.0f);

    hpid->kp = float2ctrl(kp);

    // Safety for divide-by-zero, AND removed Kp from Ki calculation
    // because Kp is multiplied outside in the step_ser function.
    if (Ti <= 0.000001f)
    {
        hpid->ki = float2ctrl(0.0f);
    }
    else
    {
        hpid->ki = float2ctrl(1.0f / (fs * Ti));
    }

    // Removed Kp from Kd calculation for the same reason.
    hpid->kd = float2ctrl(fs * Td);

    // Derivative low-pass filter coefficient
    if (Tf <= 0.0f)
    {
        hpid->alpha_d = float2ctrl(1.0f); // No filter
    }
    else
    {
        hpid->alpha_d = float2ctrl((1.0f / fs) / (Tf + (1.0f / fs)));
    }

    // Set anti-windup parameter back-calculation gain based on kp
    if (kp < 0.7f)
        hpid->kc = float2ctrl(1.3f);
    else if (kp > 2.0f)
        hpid->kc = float2ctrl(0.5f);
    else
        hpid->kc = float2ctrl(1.0f / kp);

    hpid->out_min = float2ctrl(-1.0f);
    hpid->out_max = float2ctrl(1.0f);

    ctl_clear_pid_aw(hpid);
}
