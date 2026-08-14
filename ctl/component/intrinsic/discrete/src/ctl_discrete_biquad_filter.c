#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// Biquad filter

#include <ctl/component/intrinsic/discrete/biquad_filter.h>

void ctl_init_biquad_lpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    // Quantize final coefficients through the standard control-domain conversion.
    obj->b[0] = real2ctrl((1.0f - cos_w0) / 2.0f * a0_inv);
    obj->b[1] = real2ctrl((1.0f - cos_w0) * a0_inv);
    obj->b[2] = obj->b[0]; // Reuse the already converted symmetric coefficient.
    obj->a[0] = real2ctrl(-2.0f * cos_w0 * a0_inv);
    obj->a[1] = real2ctrl((1.0f - alpha) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_hpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = real2ctrl((1.0f + cos_w0) / 2.0f * a0_inv);
    obj->b[1] = real2ctrl(-(1.0f + cos_w0) * a0_inv);
    obj->b[2] = obj->b[0];
    obj->a[0] = real2ctrl(-2.0f * cos_w0 * a0_inv);
    obj->a[1] = real2ctrl((1.0f - alpha) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_bpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = real2ctrl(alpha * a0_inv);
    obj->b[1] = CTL_CTRL_CONST_ZERO;
    obj->b[2] = real2ctrl(-alpha * a0_inv);
    obj->a[0] = real2ctrl(-2.0f * cos_w0 * a0_inv);
    obj->a[1] = real2ctrl((1.0f - alpha) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_notch(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha);

    obj->b[0] = real2ctrl(1.0f * a0_inv);
    obj->b[1] = real2ctrl(-2.0f * cos_w0 * a0_inv);
    obj->b[2] = real2ctrl(1.0f * a0_inv);
    obj->a[0] = obj->b[1]; // The notch numerator and denominator share this coefficient.
    obj->a[1] = real2ctrl((1.0f - alpha) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_peaking_eq(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                                parameter_gt gain_db)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    // The RBJ cookbook defines A as 10^(dB/40).
    parameter_gt V0 = param_pow(10.0f, gain_db / 40.0f);
    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);

    parameter_gt a0_inv = 1.0f / (1.0f + alpha / V0);

    obj->b[0] = real2ctrl((1.0f + alpha * V0) * a0_inv);
    obj->b[1] = real2ctrl(-2.0f * cos_w0 * a0_inv);
    obj->b[2] = real2ctrl((1.0f - alpha * V0) * a0_inv);
    obj->a[0] = real2ctrl(-2.0f * cos_w0 * a0_inv);
    obj->a[1] = real2ctrl((1.0f - alpha / V0) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_lowshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                              parameter_gt gain_db)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    // The RBJ cookbook defines A as 10^(dB/40).
    parameter_gt V0 = param_pow(10.0f, gain_db / 40.0f);
    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);
    parameter_gt beta = 2.0f * param_sqrt(V0) * alpha;

    parameter_gt a0_inv = 1.0f / ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 + beta);

    obj->b[0] = real2ctrl(V0 * ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 + beta) * a0_inv);
    obj->b[1] = real2ctrl(2.0f * V0 * ((V0 - 1.0f) - (V0 + 1.0f) * cos_w0) * a0_inv);
    obj->b[2] = real2ctrl(V0 * ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 - beta) * a0_inv);
    obj->a[0] = real2ctrl(-2.0f * ((V0 - 1.0f) + (V0 + 1.0f) * cos_w0) * a0_inv);
    obj->a[1] = real2ctrl(((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 - beta) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

void ctl_init_biquad_highshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                               parameter_gt gain_db)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(Q > 0.0f);

    // The RBJ cookbook defines A as 10^(dB/40).
    parameter_gt V0 = param_pow(10.0f, gain_db / 40.0f);
    parameter_gt omega = CTL_PARAM_CONST_2PI * fc / fs;
    parameter_gt cos_w0 = param_cos(omega);
    parameter_gt alpha = param_sin(omega) / (2.0f * Q);
    parameter_gt beta = 2.0f * param_sqrt(V0) * alpha;

    parameter_gt a0_inv = 1.0f / ((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 + beta);

    obj->b[0] = real2ctrl(V0 * ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 + beta) * a0_inv);
    obj->b[1] = real2ctrl(-2.0f * V0 * ((V0 - 1.0f) + (V0 + 1.0f) * cos_w0) * a0_inv);
    obj->b[2] = real2ctrl(V0 * ((V0 + 1.0f) + (V0 - 1.0f) * cos_w0 - beta) * a0_inv);
    obj->a[0] = real2ctrl(2.0f * ((V0 - 1.0f) - (V0 + 1.0f) * cos_w0) * a0_inv);
    obj->a[1] = real2ctrl(((V0 + 1.0f) - (V0 - 1.0f) * cos_w0 - beta) * a0_inv);

    ctl_clear_biquad_filter(obj);
}

parameter_gt ctl_get_biquad_phase_lag(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f)
{
    // 1. Calculate normalized angular frequency
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;

    // Pre-calculate cosine and sine terms
    parameter_gt cos_w = param_cos(w);
    parameter_gt sin_w = param_sin(w);
    parameter_gt cos_2w = param_cos(2.0f * w);
    parameter_gt sin_2w = param_sin(2.0f * w);

    // 2. Evaluate the complex numerator N(w)
    parameter_gt num_real = obj->b[0] + obj->b[1] * cos_w + obj->b[2] * cos_2w;
    parameter_gt num_imag = -obj->b[1] * sin_w - obj->b[2] * sin_2w;

    // 3. Evaluate the complex denominator D(w)
    // Note: The transfer function is 1 + a1*z^-1 + a2*z^-2, so we use +a1 and +a2 here.
    // The step function uses -a1 and -a2, which is correct for the difference equation.
    parameter_gt den_real = 1.0f + obj->a[0] * cos_w + obj->a[1] * cos_2w;
    parameter_gt den_imag = -obj->a[0] * sin_w - obj->a[1] * sin_2w;

    // 4. Calculate the phase of the numerator and denominator
    parameter_gt phase_num = param_atan2(num_imag, num_real);
    parameter_gt phase_den = param_atan2(den_imag, den_real);

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
    parameter_gt cos_w = param_cos(w);
    parameter_gt sin_w = param_sin(w);
    parameter_gt cos_2w = param_cos(2.0f * w);
    parameter_gt sin_2w = param_sin(2.0f * w);

    // 2. Evaluate the complex numerator and denominator
    parameter_gt num_real = obj->b[0] + obj->b[1] * cos_w + obj->b[2] * cos_2w;
    parameter_gt num_imag = -obj->b[1] * sin_w - obj->b[2] * sin_2w;
    parameter_gt den_real = 1.0f + obj->a[0] * cos_w + obj->a[1] * cos_2w;
    parameter_gt den_imag = -obj->a[0] * sin_w - obj->a[1] * sin_2w;

    // 3. Calculate the magnitude of the numerator
    parameter_gt mag_num = param_sqrt(num_real * num_real + num_imag * num_imag);

    // 4. Calculate the magnitude of the denominator
    parameter_gt mag_den = param_sqrt(den_real * den_real + den_imag * den_imag);

    // Avoid division by zero
    if (mag_den < 1e-9f)
    {
        return 0.0f;
    }

    // 5. Total gain = |N(w)| / |D(w)|.
    return mag_num / mag_den;
}
