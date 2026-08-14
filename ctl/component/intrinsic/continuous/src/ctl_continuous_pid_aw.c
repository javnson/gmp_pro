#include <ctl/math_block/gmp_math.h>

#include <ctl/component/intrinsic/continuous/continuous_pid_aw.h>


// init a parallel PID (Kp, Ki, Kd act independently)
void ctl_init_pid_aw_par(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt Tf,
                         parameter_gt fs)
{
    gmp_ctl_assert(fs > 0.0f);

    hpid->kp = real2ctrl(kp);

    // Safety for divide-by-zero
    if (Ti <= 0.000001f)
    {
        hpid->ki = CTL_CTRL_CONST_ZERO;
    }
    else
    {
        hpid->ki = real2ctrl(kp / (fs * Ti));
    }

    hpid->kd = real2ctrl(kp * fs * Td);

    // Derivative low-pass filter coefficient
    if (Tf <= 0.0f)
    {
        hpid->alpha_d = CTL_CTRL_CONST_1; // No filter
    }
    else
    {
        hpid->alpha_d = real2ctrl((1.0f / fs) / (Tf + (1.0f / fs)));
    }

    // Set anti-windup parameter back-calculation gain based on kp
    if (kp < 0.7f)
        hpid->kc = real2ctrl(1.3f);
    else if (kp > 2.0f)
        hpid->kc = CTL_CTRL_CONST_1_OVER_2;
    else
        hpid->kc = real2ctrl(1.0f / kp);

    hpid->out_min = (-CTL_CTRL_CONST_1);
    hpid->out_max = CTL_CTRL_CONST_1;

    ctl_clear_pid_aw(hpid);
}

// init a Series PID (Kp scales the sum of P, I, and D)
void ctl_init_pid_aw_ser(ctl_pid_aw_t* hpid, parameter_gt kp, parameter_gt Ti, parameter_gt Td, parameter_gt Tf,
                         parameter_gt fs)
{
    gmp_ctl_assert(fs > 0.0f);

    hpid->kp = real2ctrl(kp);

    // Safety for divide-by-zero, AND removed Kp from Ki calculation
    // because Kp is multiplied outside in the step_ser function.
    if (Ti <= 0.000001f)
    {
        hpid->ki = CTL_CTRL_CONST_ZERO;
    }
    else
    {
        hpid->ki = real2ctrl(1.0f / (fs * Ti));
    }

    // Removed Kp from Kd calculation for the same reason.
    hpid->kd = real2ctrl(fs * Td);

    // Derivative low-pass filter coefficient
    if (Tf <= 0.0f)
    {
        hpid->alpha_d = CTL_CTRL_CONST_1; // No filter
    }
    else
    {
        hpid->alpha_d = real2ctrl((1.0f / fs) / (Tf + (1.0f / fs)));
    }

    // Set anti-windup parameter back-calculation gain based on kp
    if (kp < 0.7f)
        hpid->kc = real2ctrl(1.3f);
    else if (kp > 2.0f)
        hpid->kc = CTL_CTRL_CONST_1_OVER_2;
    else
        hpid->kc = real2ctrl(1.0f / kp);

    hpid->out_min = (-CTL_CTRL_CONST_1);
    hpid->out_max = CTL_CTRL_CONST_1;

    ctl_clear_pid_aw(hpid);
}
