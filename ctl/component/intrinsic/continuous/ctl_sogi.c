//////////////////////////////////////////////////////////////////////////
// SOGI controller

#include <ctl/component/intrinsic/continuous/sogi.h>

void ctl_init_sogi_controller(
    // controller handle
    ctl_sogi_t* sogi,
    // gain of this controller
    parameter_gt gain,
    // resonant frequency
    parameter_gt freq_r,
    // cut frequency
    parameter_gt freq_c,
    // controller frequency
    parameter_gt freq_ctrl)
{
    sogi->k_damp = float2ctrl(2 * freq_c / freq_r);
    sogi->k_r = float2ctrl(CTL_PARAM_CONST_2PI * freq_r / freq_ctrl);
    sogi->gain = float2ctrl(gain);

    sogi->integrate_reference = 0;
    sogi->d_integrate = 0;
    sogi->q_integrate = 0;
}

void ctl_init_sogi_controller_with_damp(
    // controller handle
    ctl_sogi_t* sogi,
    // gain of this controller
    parameter_gt gain,
    // resonant frequency
    parameter_gt freq_r,
    // cut frequency, generally 1.414 is a great choice
    parameter_gt damp,
    // controller frequency
    parameter_gt freq_ctrl)
{
    sogi->k_damp = float2ctrl(damp);
    sogi->k_r = float2ctrl(CTL_PARAM_CONST_2PI * freq_r / freq_ctrl);
    sogi->gain = float2ctrl(gain);

    sogi->integrate_reference = 0;
    sogi->d_integrate = 0;
    sogi->q_integrate = 0;
}

//////////////////////////////////////////////////////////////////////////
// SOGI controller

#include <ctl/component/intrinsic/continuous/sogi.h>

void ctl_init_sogi(ctl_sogi_t* sogi, parameter_gt gain, parameter_gt freq_r, parameter_gt damp, parameter_gt fs)
{
    // Calculate the sampling period
    parameter_gt Ts = 1.0f / fs;
    // Calculate the resonant angular frequency
    parameter_gt omega_r = 2.0f * CTL_PARAM_CONST_PI * freq_r;

    // Set the controller parameters
    sogi->gain = float2ctrl(gain);
    sogi->k_damp = float2ctrl(damp);
    // Pre-calculate the resonant frequency gain for the discrete implementation
    sogi->k_r = float2ctrl(omega_r * Ts);

    // Clear all state variables to ensure a clean start
    ctl_clear_sogi(sogi);
}
