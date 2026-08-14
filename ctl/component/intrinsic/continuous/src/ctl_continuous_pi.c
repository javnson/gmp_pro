#include <ctl/math_block/gmp_math.h>


#include <ctl/component/intrinsic/continuous/continuous_pi.h>

// init a PI object (Time-constant mode, effectively Ideal/Series form)
void ctl_init_pi_Tmode(
    // continuous pi handle
    ctl_pi_t* hpi,
    // PI parameters
    parameter_gt kp, parameter_gt Ti,
    // controller frequency
    parameter_gt fs)
{
    gmp_ctl_assert(fs > 0.0f);

    hpi->kp = real2ctrl(kp);

    // Safety protection: if Ti is extremely small or zero, disable integral action
    if (Ti <= 0.000001f)
    {
        hpi->ki = CTL_CTRL_CONST_ZERO;
    }
    else
    {
        hpi->ki = real2ctrl(1.0f / (fs * Ti));
    }

    hpi->out_min = (-CTL_CTRL_CONST_1);
    hpi->out_max = CTL_CTRL_CONST_1;

    hpi->integral_min = real2ctrl(-0.8f);
    hpi->integral_max = real2ctrl(0.8f);

    ctl_clear_pi(hpi);
}

// init a parallel PI
void ctl_init_pi(
    // continuous pi handle
    ctl_pi_t* hpi,
    // PI parameters
    parameter_gt kp, parameter_gt ki,
    // controller frequency
    parameter_gt fs)
{
    gmp_ctl_assert(fs > 0.0f);

    hpi->kp = real2ctrl(kp);
    hpi->ki = real2ctrl(ki / fs);

    hpi->out_min = (-CTL_CTRL_CONST_1);
    hpi->out_max = CTL_CTRL_CONST_1;

    hpi->integral_min = real2ctrl(-0.8f);
    hpi->integral_max = real2ctrl(0.8f);

    ctl_clear_pi(hpi);
}
