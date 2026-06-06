/**
 * @file ctl_common_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Filter IIR2

#include <ctl/component/intrinsic/discrete/discrete_filter.h>

void ctl_init_lp_filter(ctl_low_pass_filter_t* lpf, parameter_gt fs, parameter_gt fc)
{
    gmp_base_assert(fs > 0.0f);

    lpf->out = 0;
    lpf->a = ctl_helper_lp_filter(fs, fc);
}

void ctl_init_filter_iir1_lpf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(fc < fs / 2.0f); // 奈奎斯特极限保护

    parameter_gt K = tanf(CTL_PARAM_CONST_PI * fc / fs);
    parameter_gt norm = 1.0f / (K + 1.0f);
    obj->b0 = K * norm;
    obj->b1 = obj->b0;
    obj->a1 = (K - 1.0f) * norm;
    ctl_clear_filter_iir1(obj);
}

void ctl_init_filter_iir1_hpf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(fc < fs / 2.0f);

    parameter_gt K = tanf(CTL_PARAM_CONST_PI * fc / fs);
    parameter_gt norm = 1.0f / (K + 1.0f);
    obj->b0 = 1.0f * norm;
    obj->b1 = -obj->b0;
    obj->a1 = (K - 1.0f) * norm;
    ctl_clear_filter_iir1(obj);
}

void ctl_init_filter_iir1_apf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(fc < fs / 2.0f);

    parameter_gt K = tanf(CTL_PARAM_CONST_PI * fc / fs);
    parameter_gt norm = 1.0f / (K + 1.0f);
    obj->b0 = (1.0f - K) * norm; // Note: b0 is negative of a1
    obj->b1 = 1.0f;
    obj->a1 = (K - 1.0f) * norm;
    ctl_clear_filter_iir1(obj);
}

/**
 * @brief Initializes the 1st-order IIR filter as a simple first-order lag (exponential smoothing) filter.
 * @details This initializer configures the IIR1 filter to emulate a standard discrete-time 
 * first-order low-pass filter with the transfer function: y[n] = a*x[n] + (1-a)*y[n-1].
 * Unlike the bilinear transform method, this approach does not introduce a transmission zero 
 * at the Nyquist frequency, making it highly robust for filtering high-frequency switching 
 * signals such as hysteresis control outputs or PWM ripples.
 *
 * @param[out] obj Pointer to the IIR filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 */
void ctl_init_filter_iir1_lag(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(fc >= 0.0f);

    // Calculate the filter coefficient a = 2 * pi * fc / fs
    parameter_gt a_val = fc * CTL_PARAM_CONST_2PI / fs;

    // Safety clamping to prevent pole overflow and divergence
    if (a_val > 1.0f)
    {
        a_val = 1.0f;
    }
    else if (a_val < 0.0f)
    {
        a_val = 0.0f;
    }

    // Map coefficients to the general IIR1 structure
    // y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1] -> y[n] = a*x[n] + 0*x[n-1] - (a-1)*y[n-1]
    obj->b0 = float2ctrl(a_val);
    obj->b1 = float2ctrl(0.0f);         // Eliminates the x[n-1] term
    obj->a1 = float2ctrl(a_val - 1.0f); // Sets the feedback pole

    ctl_clear_filter_iir1(obj);
}

parameter_gt ctl_get_filter_iir1_phase_lag(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);

    parameter_gt num_real = obj->b0 + obj->b1 * cos_w;
    parameter_gt num_imag = -obj->b1 * sin_w;
    parameter_gt den_real = 1.0f + obj->a1 * cos_w;
    parameter_gt den_imag = -obj->a1 * sin_w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

parameter_gt ctl_get_filter_iir1_gain(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);

    parameter_gt num_real = obj->b0 + obj->b1 * cos_w;
    parameter_gt num_imag = -obj->b1 * sin_w;
    parameter_gt den_real = 1.0f + obj->a1 * cos_w;
    parameter_gt den_imag = -obj->a1 * sin_w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    if (mag_den < float2ctrl(0.000001))
        return 0.0f;
    return mag_num / mag_den;
}
