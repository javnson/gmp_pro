#include <ctl/component/intrinsic/continuous/continuous_pi.h>
#include <gmp_core.h>

// init a PI object (Time-constant mode, effectively Ideal/Series form)
void ctl_init_pi_Tmode(
    // continuous pi handle
    ctl_pi_t* hpi,
    // PI parameters
    parameter_gt kp, parameter_gt Ti,
    // controller frequency
    parameter_gt fs)
{
    gmp_base_assert(fs > 0.0f);

    hpi->kp = float2ctrl(kp);

    // Safety protection: if Ti is extremely small or zero, disable integral action
    if (Ti <= 0.000001f)
    {
        hpi->ki = float2ctrl(0.0f);
    }
    else
    {
        hpi->ki = float2ctrl(1.0f / (fs * Ti));
    }

    hpi->out_min = float2ctrl(-1.0f);
    hpi->out_max = float2ctrl(1.0f);

    hpi->integral_min = float2ctrl(-0.8f);
    hpi->integral_max = float2ctrl(0.8f);

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
    gmp_base_assert(fs > 0.0f);

    hpi->kp = float2ctrl(kp);
    hpi->ki = float2ctrl(ki / fs);

    hpi->out_min = float2ctrl(-1.0f);
    hpi->out_max = float2ctrl(1.0f);

    hpi->integral_min = float2ctrl(-0.8f);
    hpi->integral_max = float2ctrl(0.8f);

    ctl_clear_pi(hpi);
}
