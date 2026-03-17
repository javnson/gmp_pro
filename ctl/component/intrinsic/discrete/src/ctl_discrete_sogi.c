#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
//

#include <ctl/component/intrinsic/discrete/discrete_sogi.h>

void ctl_init_discrete_sogi(
    // Handle of discrete SOGI object
    discrete_sogi_t* sogi,
    // damp coefficient, generally is 0.5
    parameter_gt k_damp,
    // center frequency, Hz
    parameter_gt fn,
    // isr frequency, Hz
    parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);
    gmp_base_assert(fn > 0.0);

    ctl_clear_discrete_sogi(sogi);

    parameter_gt osgx, osgy, temp, wn, delta_t;
    delta_t = 1.0f / fs;
    wn = fn * CTL_PARAM_CONST_2PI;
    // wn = fn * GMP_CONST_2_PI;

    osgx = (2.0f * k_damp * wn * delta_t);
    osgy = (parameter_gt)(wn * delta_t * wn * delta_t);
    temp = (parameter_gt)1.0 / (osgx + osgy + 4.0f);

    sogi->b0 = float2ctrl(osgx * temp);
    sogi->b2 = float2ctrl(-osgx * temp);
    sogi->a1 = float2ctrl((2.0f * (4.0f - osgy)) * temp);
    sogi->a2 = float2ctrl((osgx - osgy - 4.0f) * temp);

    parameter_gt qb0_f = (k_damp * osgy) * temp;
    sogi->qb0 = float2ctrl(qb0_f);
    sogi->qb1 = float2ctrl(qb0_f * (2.0f));
    sogi->qb2 = sogi->qb0;
}

void ctl_init_discrete_sogi_dc(discrete_sogi_dc_t* sogi_dc, parameter_gt k_damp, parameter_gt k_dc, parameter_gt fn,
                               parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    // 1. Initialize Standard SOGI Core
    ctl_init_discrete_sogi(&sogi_dc->core, k_damp, fn, fs);

    // 2. Clear States
    ctl_clear_discrete_sogi_dc(sogi_dc);

    // 3. Pre-calculate DC Integrator Gain
    // Gain = Ts * omega0 * k_dc
    // Ts = 1/fs
    // omega0 = 2 * pi * fn
    parameter_gt omega0 = CTL_PARAM_CONST_2PI * fn;
    parameter_gt ts = 1.0f / fs;

    // We store this as a fixed gain to use in the step function
    sogi_dc->dc_integ_gain = float2ctrl(ts * omega0 * k_dc);
}
