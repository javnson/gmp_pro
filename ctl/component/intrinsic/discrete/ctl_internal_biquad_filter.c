#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Biquad filter

#include <ctl/component/intrinsic/discrete/biquad_filter.h>

void ctl_init_biquad_lpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = (1.0f - cos_w0) / 2.0f * a0_inv;
    obj->b[1] = (1.0f - cos_w0) * a0_inv;
    obj->b[2] = obj->b[0];
    obj->a[0] = -2.0f * cos_w0 * a0_inv;
    obj->a[1] = (1.0f - alpha) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_hpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = (1.0f + cos_w0) / 2.0f * a0_inv;
    obj->b[1] = -(1.0f + cos_w0) * a0_inv;
    obj->b[2] = obj->b[0];
    obj->a[0] = -2.0f * cos_w0 * a0_inv;
    obj->a[1] = (1.0f - alpha) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_bpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = alpha * a0_inv;
    obj->b[1] = 0.0f;
    obj->b[2] = -alpha * a0_inv;
    obj->a[0] = -2.0f * cos_w0 * a0_inv;
    obj->a[1] = (1.0f - alpha) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_notch(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = 1.0f * a0_inv;
    obj->b[1] = -2.0f * cos_w0 * a0_inv;
    obj->b[2] = 1.0f * a0_inv;
    obj->a[0] = -2.0f * cos_w0 * a0_inv;
    obj->a[1] = (1.0f - alpha) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_allpass(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = (1.0f - alpha) * a0_inv;
    obj->b[1] = -2.0f * cos_w0 * a0_inv;
    obj->b[2] = (1.0f + alpha) * a0_inv;
    obj->a[0] = -2.0f * cos_w0 * a0_inv;
    obj->a[1] = (1.0f - alpha) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_peaking_eq(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                                parameter_gt gain_db)
{
    parameter_gt V0 = powf(10.0f, gain_db / 20.0f);
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha / V0);

    obj->b[0] = (1.0f + alpha * V0) * a0_inv;
    obj->b[1] = -2.0f * cos_w0 * a0_inv;
    obj->b[2] = (1.0f - alpha * V0) * a0_inv;
    obj->a[0] = -2.0f * cos_w0 * a0_inv;
    obj->a[1] = (1.0f - alpha / V0) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_lowshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                              parameter_gt gain_db)
{
    parameter_gt V0 = powf(10.0f, gain_db / 20.0f);
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);
    parameter_gt beta = 2.0f * sqrtf(V0) * alpha;

    parameter_gt a0_inv = 1.0f / ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 + beta);

    obj->b[0] = V0 * ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 + beta) * a0_inv;
    obj->b[1] = 2.0f * V0 * ((V0 - 1.0f) - (V0 + 1.0f) * cos_w0) * a0_inv;
    obj->b[2] = V0 * ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 - beta) * a0_inv;
    obj->a[0] = -2.0f * ((V0 - 1.0f) + (V0 + 1.0f) * cos_w0) * a0_inv;
    obj->a[1] = ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 - beta) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_highshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                               parameter_gt gain_db)
{
    parameter_gt V0 = powf(10.0f, gain_db / 20.0f);
    parameter_gt omega = 2.0f * CTL_PARAM_CONST_PI * fc / fs;
    parameter_gt cos_w0 = cosf(omega);
    parameter_gt alpha = sinf(omega) / (2.0f * Q);
    parameter_gt beta = 2.0f * sqrtf(V0) * alpha;

    parameter_gt a0_inv = 1.0f / ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 + beta);

    obj->b[0] = V0 * ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 + beta) * a0_inv;
    obj->b[1] = -2.0f * V0 * ((V0 - 1.0f) + (V0 + 1.0f) * cos_w0) * a0_inv;
    obj->b[2] = V0 * ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 - beta) * a0_inv;
    obj->a[0] = 2.0f * ((V0 - 1.0f) - (V0 + 1.0f) * cos_w0) * a0_inv;
    obj->a[1] = ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 - beta) * a0_inv;

    ctl_clear_biquad_filter(obj);
}

parameter_gt ctl_get_biquad_phase_lag(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f)
{
    // 1. Calculate normalized angular frequency
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;

    // Pre-calculate cosine and sine terms
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2.0f * w);
    parameter_gt sin_2w = sinf(2.0f * w);

    // 2. Evaluate the complex numerator N(w)
    parameter_gt num_real = obj->b[0] + obj->b[1] * cos_w + obj->b[2] * cos_2w;
    parameter_gt num_imag = -obj->b[1] * sin_w - obj->b[2] * sin_2w;

    // 3. Evaluate the complex denominator D(w)
    // Note: The transfer function is 1 + a1*z^-1 + a2*z^-2, so we use +a1 and +a2 here.
    // The step function uses -a1 and -a2, which is correct for the difference equation.
    parameter_gt den_real = 1.0f + obj->a[0] * cos_w + obj->a[1] * cos_2w;
    parameter_gt den_imag = -obj->a[0] * sin_w - obj->a[1] * sin_2w;

    // 4. Calculate the phase of the numerator and denominator
    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    // 5. Total phase = phase(N) - phase(D)
    parameter_gt total_phase = phase_num - phase_den;

    // 6. Phase lag = -Total phase
    return -total_phase;
}

parameter_gt ctl_get_biquad_gain(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f)
{
    // 1. Calculate normalized angular frequency
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;

    // Pre-calculate cosine and sine terms
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2.0f * w);
    parameter_gt sin_2w = sinf(2.0f * w);

    // 2. Evaluate the complex numerator and denominator
    parameter_gt num_real = obj->b[0] + obj->b[1] * cos_w + obj->b[2] * cos_2w;
    parameter_gt num_imag = -obj->b[1] * sin_w - obj->b[2] * sin_2w;
    parameter_gt den_real = 1.0f + obj->a[0] * cos_w + obj->a[1] * cos_2w;
    parameter_gt den_imag = -obj->a[0] * sin_w - obj->a[1] * sin_2w;

    // 3. Calculate the magnitude of the numerator
    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);

    // 4. Calculate the magnitude of the denominator
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    // Avoid division by zero
    if (mag_den < 1e-9f)
    {
        return 0.0f;
    }

    // 5. Total gain = |N(¦Ø)| / |D(¦Ø)|
    return mag_num / mag_den;
}
