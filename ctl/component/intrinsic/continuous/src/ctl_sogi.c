#include <ctl/math_block/gmp_math.h>


//////////////////////////////////////////////////////////////////////////
// SOGI controller

#include <ctl/component/intrinsic/continuous/sogi.h>

void ctl_init_sogi(ctl_sogi_t* sogi, parameter_gt gain, parameter_gt freq_r, parameter_gt damp, parameter_gt fs)
{
    // Error prevention engineering: guard against division by zero
    gmp_ctl_assert(fs > 0.0f);

    // Calculate the sampling period
    parameter_gt Ts = 1.0f / fs;
    // Calculate the resonant angular frequency
    parameter_gt omega_r = 2.0f * CTL_PARAM_CONST_PI * freq_r;

    // Set the controller parameters
    sogi->gain = real2ctrl(gain);
    sogi->k_damp = real2ctrl(damp);
    // Pre-calculate the resonant frequency gain for the discrete implementation
    sogi->k_r = real2ctrl(omega_r * Ts);

    // Clear all state variables to ensure a clean start
    ctl_clear_sogi(sogi);
}
