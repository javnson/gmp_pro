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
#include <math.h> // support for sinf and cosf

void ctl_init_lp_filter(ctl_low_pass_filter_t* lpf, parameter_gt fs, parameter_gt fc)
{
    lpf->out = 0;
    lpf->a = ctl_helper_lp_filter(fs, fc);
}

void ctl_init_filter_iir1_lpf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    parameter_gt K = tanf(CTL_PARAM_CONST_PI * fc / fs);
    parameter_gt norm = 1.0f / (K + 1.0f);
    obj->b0 = K * norm;
    obj->b1 = obj->b0;
    obj->a1 = (K - 1.0f) * norm;
    ctl_clear_filter_iir1(obj);
}

void ctl_init_filter_iir1_hpf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    parameter_gt K = tanf(CTL_PARAM_CONST_PI * fc / fs);
    parameter_gt norm = 1.0f / (K + 1.0f);
    obj->b0 = 1.0f * norm;
    obj->b1 = -obj->b0;
    obj->a1 = (K - 1.0f) * norm;
    ctl_clear_filter_iir1(obj);
}

void ctl_init_filter_iir1_apf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc)
{
    parameter_gt K = tanf(CTL_PARAM_CONST_PI * fc / fs);
    parameter_gt norm = 1.0f / (K + 1.0f);
    obj->b0 = (1.0f - K) * norm; // Note: b0 is negative of a1
    obj->b1 = 1.0f;
    obj->a1 = (K - 1.0f) * norm;
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

    // 5. Total gain = |N(ω)| / |D(ω)|
    return mag_num / mag_den;
}

//////////////////////////////////////////////////////////////////////////
// FIR Filter
#include <ctl/component/intrinsic/discrete/fir_filter.h>
#include <stdlib.h> // Required for malloc and free

fast_gt ctl_init_fir_filter(ctl_fir_filter_t* fir, uint32_t order, const ctrl_gt* coeffs)
{
    fir->order = order;
    fir->coeffs = coeffs;
    fir->output = 0.0f;
    fir->buffer_index = 0;

    // 动态分配状态缓冲区
    // 滤波器的状态（即过去的输入样本）需要一个缓冲区。其大小等于滤波器的阶数。
    // 使用动态内存分配可以使模块适应任意阶数的滤波器。
    fir->buffer = (ctrl_gt*)malloc(order * sizeof(ctrl_gt));
    if (fir->buffer == NULL)
    {
        return 0; // 内存分配失败
    }

    // 将缓冲区初始化为零
    ctl_clear_fir_filter(fir);

    return 1; // 成功
}

//////////////////////////////////////////////////////////////////////////
// Direct Form controller
#include <ctl/component/intrinsic/discrete/direct_form.h>

/*---------------------------------------------------------------------------*/
/* DF11 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df11(ctl_df11_t* df, parameter_gt b0, parameter_gt b1, parameter_gt a1)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->a1 = float2ctrl(a1);
    ctl_clear_df11(df);
}

parameter_gt ctl_get_df11_gain(ctl_df11_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w;
    parameter_gt num_imag = -df->b1 * sin_w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w;
    parameter_gt den_imag = -df->a1 * sin_w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df11_phase_lag(ctl_df11_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w;
    parameter_gt num_imag = -df->b1 * sin_w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w;
    parameter_gt den_imag = -df->a1 * sin_w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

/*---------------------------------------------------------------------------*/
/* DF22 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df22(ctl_df22_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt a1, parameter_gt a2)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    ctl_clear_df22(df);
}

parameter_gt ctl_get_df22_gain(ctl_df22_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df22_phase_lag(ctl_df22_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

/*---------------------------------------------------------------------------*/
/* DF13 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df13(ctl_df13_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt b3, parameter_gt a1,
                   parameter_gt a2, parameter_gt a3)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->b3 = float2ctrl(b3);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    df->a3 = float2ctrl(a3);
    ctl_clear_df13(df);
}

parameter_gt ctl_get_df13_gain(ctl_df13_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df13_phase_lag(ctl_df13_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

/*---------------------------------------------------------------------------*/
/* DF23 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df23(ctl_df23_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt b3, parameter_gt a1,
                   parameter_gt a2, parameter_gt a3)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->b3 = float2ctrl(b3);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    df->a3 = float2ctrl(a3);
    ctl_clear_df23(df);
}

parameter_gt ctl_get_df23_gain(ctl_df23_t* df, parameter_gt fs, parameter_gt f)
{
    // Transfer function is identical to DF13
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df23_phase_lag(ctl_df23_t* df, parameter_gt fs, parameter_gt f)
{
    // Transfer function is identical to DF13
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

////////////////////////////////////////////////////////////////////////////
//// PLL module
//
//#include <ctl/component/intrinsic/discrete/pll.h>
//
//void ctl_init_pll(
//    // PLL Controller Object
//    ctl_pll_t* pll,
//    // PID parameter
//    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
//    // PID output limit
//    ctrl_gt out_min, ctrl_gt out_max,
//    // cutoff frequency
//    parameter_gt fc,
//    // Sample frequency
//    parameter_gt fs)
//{
//    ctl_init_pid(&pll->pid, kp, Ti, Td, fs);
//    ctl_set_pid_limit(&pll->pid, out_max, out_min);
//    ctl_init_lp_filter(&pll->filter, fs, fc);
//}

//////////////////////////////////////////////////////////////////////////
// Signal Generator

#include <ctl/component/intrinsic/discrete/signal_generator.h>

void ctl_init_sine_generator(ctl_sine_generator_t* sg,
                             parameter_gt init_angle, // pu
                             parameter_gt step_angle) // pu
{
    sg->ph_cos = float2ctrl(cos(init_angle));
    sg->ph_sin = float2ctrl(sin(init_angle));

    sg->ph_sin_delta = float2ctrl(sin(step_angle));
    sg->ph_cos_delta = float2ctrl(cos(step_angle));
}

void ctl_init_ramp_generator(ctl_ramp_generator_t* _rg, ctrl_gt slope, parameter_gt amp_pos, parameter_gt amp_neg)
{
    _rg->current = float2ctrl(0);

    _rg->maximum = float2ctrl(amp_pos);
    _rg->minimum = float2ctrl(amp_neg);

    _rg->slope = slope;
}

void ctl_init_ramp_generator_via_freq(
    // pointer to ramp generator object
    ctl_ramp_generator_t* _rg,
    // isr frequency, unit Hz
    parameter_gt isr_freq,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // ramp range
    parameter_gt amp_pos, parameter_gt amp_neg)
{
    _rg->current = float2ctrl(0);

    _rg->maximum = float2ctrl(amp_pos);
    _rg->minimum = float2ctrl(amp_neg);

    float a = isr_freq / target_freq;
    float b = amp_pos - amp_neg;

    // _rg->slope = float2ctrl((amp_pos - amp_neg) / (isr_freq / target_freq));
    _rg->slope = float2ctrl(b / a);
}

void ctl_init_square_wave_generator(ctl_square_wave_generator_t* sq, parameter_gt fs, parameter_gt target_freq,
                                    parameter_gt amplitude, parameter_gt offset)
{
    sq->high_level = offset + amplitude;
    sq->low_level = offset - amplitude;
    sq->phase = 0.0f;
    sq->phase_step = 2.0f * CTL_PARAM_CONST_PI * target_freq / fs;
    sq->output = sq->high_level;
}

void ctl_init_triangle_wave_generator(ctl_triangle_wave_generator_t* tri, parameter_gt fs, parameter_gt target_freq,
                                      parameter_gt pos_peak, parameter_gt neg_peak)
{
    tri->pos_peak = pos_peak;
    tri->neg_peak = neg_peak;
    // The total peak-to-peak amplitude is traversed twice per period (up and down).
    // So, the time for one ramp (neg to pos) is T/2.
    // Slope = Amplitude / Time = (pos_peak - neg_peak) / ( (1/target_freq) / 2 )
    // Slope per sample = Slope / fs
    tri->slope = 2.0f * (pos_peak - neg_peak) * target_freq / fs;
    tri->output = neg_peak;
}

//////////////////////////////////////////////////////////////////////////
// Discrete PID controller

#include <ctl/component/intrinsic/discrete/discrete_pid.h>
#ifdef _USE_DEBUG_DISCRETE_PID
void ctl_init_discrete_pid(
    // pointer to pid object
    discrete_pid_t* pid,
    // gain of the pid controller
    parameter_gt kp,
    // Time constant for integral and differential part, unit Hz
    parameter_gt Ti, parameter_gt Td,
    // sample frequency, unit Hz
    parameter_gt fs)
{
    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    parameter_gt ki = kp / Ti;
    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / 2.0f / fs - 2.0f * kd * fs;
    parameter_gt b0 = kd * fs + ki / 2.0f / fs;

    pid->kp = float2ctrl(kp);

    pid->b2 = float2ctrl(b2);
    pid->b1 = float2ctrl(b1);
    pid->b0 = float2ctrl(b0);

    pid->output_max = float2ctrl(1.0);
    pid->output_min = float2ctrl(-1.0);
}
#else // _USE_DEBUG_DISCRETE_PID
void ctl_init_discrete_pid(
    // pointer to pid object
    discrete_pid_t* pid,
    // gain of the pid controller
    parameter_gt kp,
    // Time constant for integral and differential part, unit Hz
    parameter_gt Ti, parameter_gt Td,
    // sample frequency, unit Hz
    parameter_gt fs)
{
    pid->input = 0;
    pid->input_1 = 0;
    pid->input_2 = 0;
    pid->output = 0;
    pid->output_1 = 0;

    parameter_gt ki = kp / Ti;
    parameter_gt kd = kp * Td;

    parameter_gt b2 = kd * fs;
    parameter_gt b1 = ki / 2.0f / fs - kp - 2.0f * kd * fs;
    parameter_gt b0 = kp + kd * fs + ki / 2.0f / fs;

    pid->b2 = float2ctrl(b2);
    pid->b1 = float2ctrl(b1);
    pid->b0 = float2ctrl(b0);

    pid->output_max = float2ctrl(1.0);
    pid->output_min = float2ctrl(-1.0);
}

#endif // _USE_DEBUG_DISCRETE_PID

//////////////////////////////////////////////////////////////////////////
// Discrete track pid

#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>

void ctl_init_tracking_pid(
    // pointer to track pid object
    ctl_tracking_discrete_pid_t* tp,
    // pid parameters, unit sec
    parameter_gt kp, parameter_gt Ti, parameter_gt Td,
    // saturation limit
    ctrl_gt sat_max, ctrl_gt sat_min,
    // slope limit, unit: p.u./sec
    parameter_gt slope_max, parameter_gt slope_min,
    // division factor:
    uint32_t division,
    // controller frequency, unit Hz
    parameter_gt fs)
{
    ctl_init_slope_limiter(&tp->traj, slope_max, slope_min, fs);
    ctl_init_divider(&tp->div, division);

    ctl_init_discrete_pid(&tp->pid, kp, Ti, Td, fs);
    ctl_set_discrete_pid_limit(&tp->pid, sat_max, sat_min);
}

//////////////////////////////////////////////////////////////////////////
// Pole Zero controller
#include <ctl/component/intrinsic/discrete/pole_zero.h>

// Helper function to multiply two first-order polynomials: (b0 + b1*z^-1) * (c0 + c1*z^-1) -> out[0] + out[1]*z^-1 + out[2]*z^-2
//static void _multiply_poly1_poly1(const parameter_gt b[2], const parameter_gt c[2], parameter_gt out[3])
//{
//    out[0] = b[0] * c[0];
//    out[1] = b[0] * c[1] + b[1] * c[0];
//    out[2] = b[1] * c[1];
//}

// Helper function to multiply a second-order and a first-order polynomial
//static void _multiply_poly2_poly1(const parameter_gt b[3], const parameter_gt c[2], parameter_gt out[4])
//{
//    out[0] = b[0] * c[0];
//    out[1] = b[0] * c[1] + b[1] * c[0];
//    out[2] = b[1] * c[1] + b[2] * c[0];
//    out[3] = b[2] * c[1];
//}

// Helper function to multiply a second-order numerator and a first-order numerator polynomial
static void _multiply_num_poly2_poly1(const parameter_gt b[3], const parameter_gt c[2], parameter_gt out[4])
{
    out[0] = b[0] * c[0];
    out[1] = b[0] * c[1] + b[1] * c[0];
    out[2] = b[1] * c[1] + b[2] * c[0];
    out[3] = b[2] * c[1];
}

// Helper function to multiply a 1st-order denominator (1 + a1*z^-1) and a 2nd-order denominator (1 + b1*z^-1 + b2*z^-2)
// The output corresponds to the coefficients of the resulting 3rd-order denominator: 1 + c1*z^-1 + c2*z^-2 + c3*z^-3
static void _multiply_den_poly2_poly1(const parameter_gt p2[2], const parameter_gt p1[1], parameter_gt out_a[3])
{
    // (1 + p2[0]z^-1 + p2[1]z^-2) * (1 + p1[0]z^-1)
    // = 1 + p1[0]z^-1 + p2[0]z^-1 + p1[0]p2[0]z^-2 + p2[1]z^-2 + p1[0]p2[1]z^-3
    // = 1 + (p1[0] + p2[0])z^-1 + (p2[1] + p1[0]p2[0])z^-2 + (p1[0]p2[1])z^-3
    out_a[0] = p1[0] + p2[0];
    out_a[1] = p2[1] + p1[0] * p2[0];
    out_a[2] = p1[0] * p2[1];
}

// Helper to calculate the z-domain polynomial coefficients from s-plane roots.
// Can handle two real roots (r1_hz, r2_hz) or a complex conjugate pair (real_hz, imag_hz).
// The s-plane polynomial is assumed to be s^2 + c1*s + c0.
// The output is the unnormalized z-domain polynomial: coeffs[0] + coeffs[1]*z^-1 + coeffs[2]*z^-2
static void _calc_poly2_coeffs(parameter_gt r1_hz, parameter_gt r2_hz, int is_complex, parameter_gt fs,
                               parameter_gt coeffs[3])
{
    parameter_gt c0, c1; // s^2 + c1*s + c0
    if (is_complex)
    {
        // s = -sigma +/- j*wd, where sigma = 2*pi*r1_hz, wd = 2*pi*r2_hz
        parameter_gt sigma = 2.0f * CTL_PARAM_CONST_PI * r1_hz;
        parameter_gt wd = 2.0f * CTL_PARAM_CONST_PI * r2_hz;
        c1 = 2.0f * sigma;
        c0 = sigma * sigma + wd * wd;
    }
    else
    {
        // (s + w1)(s + w2) = s^2 + (w1+w2)s + w1*w2
        parameter_gt w1 = 2.0f * CTL_PARAM_CONST_PI * r1_hz;
        parameter_gt w2 = 2.0f * CTL_PARAM_CONST_PI * r2_hz;
        c1 = w1 + w2;
        c0 = w1 * w2;
    }

    parameter_gt k = 2.0f * fs;
    parameter_gt k2 = k * k;
    parameter_gt den = k2 + c1 * k + c0;
    if (den < 1e-9f)
    {
        den = 1e-9f;
    }

    coeffs[0] = (k2 + c1 * k + c0) / den;      // z^0 term
    coeffs[1] = (2.0f * c0 - 2.0f * k2) / den; // z^-1 term
    coeffs[2] = (k2 - c1 * k + c0) / den;      // z^-2 term
}

/*---------------------------------------------------------------------------*/
/* 1P1Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_1p1z(ctrl_1p1z_t* c, parameter_gt gain, parameter_gt f_z, parameter_gt f_p, parameter_gt fs)
{
    parameter_gt Kz = tanf(CTL_PARAM_CONST_PI * f_z / fs);
    parameter_gt Kp = tanf(CTL_PARAM_CONST_PI * f_p / fs);
    parameter_gt den_norm = Kp + 1.0f;
    if (den_norm < 1e-9f)
    {
        den_norm = 1e-9f;
    }

    parameter_gt b0 = (Kz + 1.0f) / den_norm;
    parameter_gt b1 = (Kz - 1.0f) / den_norm;
    parameter_gt a1 = (Kp - 1.0f) / den_norm;

    parameter_gt dc_gain_comp = (f_p > 1e-9f && f_z > 1e-9f) ? (f_p / f_z) : 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    c->coef_b[0] = float2ctrl(b0 * final_gain);
    c->coef_b[1] = float2ctrl(b1 * final_gain);
    c->coef_a[0] = float2ctrl(-a1); // Standard form H(z) = ... / (1 + a1*z^-1)

    ctl_clear_1p1z(c);
}

/*---------------------------------------------------------------------------*/
/* 2P2Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_2p2z_real(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_p1,
                        parameter_gt f_p2, parameter_gt fs)
{
    parameter_gt num_poly_z[3], den_poly_z[3];
    _calc_poly2_coeffs(f_z1, f_z2, 0, fs, num_poly_z);
    _calc_poly2_coeffs(f_p1, f_p2, 0, fs, den_poly_z);

    parameter_gt norm = 1.0f / den_poly_z[0];
    parameter_gt a1 = den_poly_z[1] * norm;
    parameter_gt a2 = den_poly_z[2] * norm;

    parameter_gt dc_gain_comp = (f_p1 * f_p2) / (f_z1 * f_z2);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    c->coef_b[0] = float2ctrl(num_poly_z[0] * norm * final_gain);
    c->coef_b[1] = float2ctrl(num_poly_z[1] * norm * final_gain);
    c->coef_b[2] = float2ctrl(num_poly_z[2] * norm * final_gain);
    c->coef_a[0] = float2ctrl(a1);
    c->coef_a[1] = float2ctrl(a2);

    ctl_clear_2p2z(c);
}

void ctl_init_2p2z_complex_zeros(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_p1, parameter_gt f_p2, parameter_gt fs)
{
    parameter_gt num_poly_z[3], den_poly_z[3];
    _calc_poly2_coeffs(f_czr, f_czi, 1, fs, num_poly_z);
    _calc_poly2_coeffs(f_p1, f_p2, 0, fs, den_poly_z);

    parameter_gt norm = 1.0f / den_poly_z[0];
    parameter_gt a1 = den_poly_z[1] * norm;
    parameter_gt a2 = den_poly_z[2] * norm;

    parameter_gt dc_gain_comp = (f_p1 * f_p2) / (f_czr * f_czr + f_czi * f_czi);
    if (f_czr < 1e-9f && f_czi < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    c->coef_b[0] = float2ctrl(num_poly_z[0] * norm * final_gain);
    c->coef_b[1] = float2ctrl(num_poly_z[1] * norm * final_gain);
    c->coef_b[2] = float2ctrl(num_poly_z[2] * norm * final_gain);
    c->coef_a[0] = float2ctrl(a1);
    c->coef_a[1] = float2ctrl(a2);

    ctl_clear_2p2z(c);
}

/*---------------------------------------------------------------------------*/
/* 3P3Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_3p3z_real(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_z3,
                        parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3, parameter_gt fs)
{
    int i;

    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;

    ctl_init_2p2z_real(&sec1, 1.0, f_z1, f_z2, f_p1, f_p2, fs);
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    parameter_gt b2[3] = {sec1.coef_b[0], sec1.coef_b[1], sec1.coef_b[2]};
    parameter_gt c2[2] = {sec2.coef_b[0], sec2.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {sec1.coef_a[0], sec1.coef_a[1]};
    parameter_gt a1[1] = {sec2.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp = (f_p1 * f_p2 * f_p3) / (f_z1 * f_z2 * f_z3);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f || f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_zeros(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_z3, parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3,
                                 parameter_gt fs)
{
    int i;

    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;
    ctl_init_2p2z_complex_zeros(&sec1, 1.0, f_czr, f_czi, f_p1, f_p2, fs);
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    parameter_gt b2[3] = {sec1.coef_b[0], sec1.coef_b[1], sec1.coef_b[2]};
    parameter_gt c2[2] = {sec2.coef_b[0], sec2.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {sec1.coef_a[0], sec1.coef_a[1]};
    parameter_gt a1[1] = {sec2.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp = (f_p1 * f_p2 * f_p3) / ((f_czr * f_czr + f_czi * f_czi) * f_z3);
    if (f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_poles(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2,
                                 parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                 parameter_gt fs)
{
    int i;

    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;

    // Build the 2P2Z section with two real zeros and one complex pole pair
    parameter_gt num_poly[3], den_poly[3];
    _calc_poly2_coeffs(f_z1, f_z2, 0, fs, num_poly);
    _calc_poly2_coeffs(f_cpr, f_cpi, 1, fs, den_poly);

    parameter_gt norm = 1.0f / den_poly[0];
    sec1.coef_a[0] = den_poly[1] * norm;
    sec1.coef_a[1] = den_poly[2] * norm;
    sec1.coef_b[0] = num_poly[0] * norm;
    sec1.coef_b[1] = num_poly[1] * norm;
    sec1.coef_b[2] = num_poly[2] * norm;

    // Build the 1P1Z section with the remaining real pole and zero
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    // Multiply the polynomials
    parameter_gt b2[3] = {sec1.coef_b[0], sec1.coef_b[1], sec1.coef_b[2]};
    parameter_gt c2[2] = {sec2.coef_b[0], sec2.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {sec1.coef_a[0], sec1.coef_a[1]};
    parameter_gt a1[1] = {sec2.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp = ((f_cpr * f_cpr + f_cpi * f_cpi) * f_p3) / (f_z1 * f_z2 * f_z3);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f || f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_pair(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                parameter_gt fs)
{
    int i;

    ctrl_2p2z_t complex_sec;
    ctrl_1p1z_t real_sec;

    // Build the 2P2Z section from the complex pairs
    parameter_gt num_poly[3], den_poly[3];
    _calc_poly2_coeffs(f_czr, f_czi, 1, fs, num_poly);
    _calc_poly2_coeffs(f_cpr, f_cpi, 1, fs, den_poly);

    parameter_gt norm = 1.0f / den_poly[0];
    complex_sec.coef_a[0] = den_poly[1] * norm;
    complex_sec.coef_a[1] = den_poly[2] * norm;
    complex_sec.coef_b[0] = num_poly[0] * norm;
    complex_sec.coef_b[1] = num_poly[1] * norm;
    complex_sec.coef_b[2] = num_poly[2] * norm;

    // Build the 1P1Z section from the real pair
    ctl_init_1p1z(&real_sec, 1.0, f_z3, f_p3, fs);

    // Multiply the polynomials
    parameter_gt b2[3] = {complex_sec.coef_b[0], complex_sec.coef_b[1], complex_sec.coef_b[2]};
    parameter_gt c2[2] = {real_sec.coef_b[0], real_sec.coef_b[1]};
    parameter_gt b3[4];
    _multiply_num_poly2_poly1(b2, c2, b3);

    parameter_gt a2[2] = {complex_sec.coef_a[0], complex_sec.coef_a[1]};
    parameter_gt a1[1] = {real_sec.coef_a[0]};
    parameter_gt a3[3];
    _multiply_den_poly2_poly1(a2, a1, a3);

    parameter_gt dc_gain_comp_c = (f_cpr * f_cpr + f_cpi * f_cpi) / (f_czr * f_czr + f_czi * f_czi);
    parameter_gt dc_gain_comp_r = (f_p3 / f_z3);
    if (f_z3 < 1e-9f)
        dc_gain_comp_r = 1.0f;
    parameter_gt final_gain = gain / (dc_gain_comp_c * dc_gain_comp_r);

    for (i = 0; i < 4; ++i)
        c->coef_b[i] = float2ctrl(b3[i] * final_gain);
    for (i = 0; i < 3; ++i)
        c->coef_a[i] = float2ctrl(a3[i]);
    ctl_clear_3p3z(c);
}

/*---------------------------------------------------------------------------*/
/* Analysis Functions                                                        */
/*---------------------------------------------------------------------------*/
parameter_gt ctl_get_2p2z_gain(ctrl_2p2z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_2p2z_phase_lag(ctrl_2p2z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

parameter_gt ctl_get_3p3z_gain(ctrl_3p3z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w + c->coef_b[3] * cos_3w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w - c->coef_b[3] * sin_3w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w + c->coef_a[2] * cos_3w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w - c->coef_a[2] * sin_3w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_3p3z_phase_lag(ctrl_3p3z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w + c->coef_b[2] * cos_2w + c->coef_b[3] * cos_3w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w - c->coef_b[2] * sin_2w - c->coef_b[3] * sin_3w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w + c->coef_a[1] * cos_2w + c->coef_a[2] * cos_3w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w - c->coef_a[1] * sin_2w - c->coef_a[2] * sin_3w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

//////////////////////////////////////////////////////////////////////////
// PR / QPR controller

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wr_sq_T_sq = wr * wr * T * T;

    // Based on the bilinear transformation of G(s) = kr * (2s) / (s^2 + wr^2)
    // The resulting difference equation is:
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    parameter_gt den = wr_sq_T_sq + 4.0f;
    parameter_gt inv_den = 1.0f / den;

    r->b0 = float2ctrl(kr * 2.0f * T * inv_den);
    r->b2 = float2ctrl(-kr * 2.0f * T * inv_den);
    r->a1 = float2ctrl(2.0f * (4.0f - wr_sq_T_sq) * inv_den);
    r->a2 = float2ctrl(-1.0f); // This simplifies from -(4 + T^2*wr^2 - 8)/(4+T^2*wr^2) if no damping

    // In the original code, the denominator of a1 was different.
    // The standard discretization of an ideal resonant controller leads to a2 = -1.
    // The original code's coefficients seem to be from a slightly different form.
    // Let's stick to the standard bilinear transform result.
    // Original code had: r->kr = float2ctrl(2.0f * (4.0f * fs * fs - wr * wr) / (4.0f * fs * fs + wr * wr));
    // and r->krg = float2ctrl(kr * 4.0f * fs / (4.0f * fs * fs + wr * wr));
    // which corresponds to a different difference equation form.
    // The new form is u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)

    ctl_clear_resonant_controller(r);
}

void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs)
{
    pr->kp = kp;
    ctl_init_resonant_controller(&pr->resonant_part, kr, freq_resonant, fs);
}

void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs)
{
    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;

    // Based on the bilinear transformation of G(s) = kr * (2*wc*s) / (s^2 + 2*wc*s + wr^2)
    // The resulting difference equation is:
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    parameter_gt den = 4.0f + 4.0f * wc * T + wr * wr * T * T;
    parameter_gt inv_den = 1.0f / den;

    qr->b0 = float2ctrl(kr * 4.0f * wc * T * inv_den);
    qr->b2 = float2ctrl(-kr * 4.0f * wc * T * inv_den);
    qr->a1 = float2ctrl((8.0f - 2.0f * wr * wr * T * T) * inv_den);
    qr->a2 = float2ctrl((-4.0f + 4.0f * wc * T - wr * wr * T * T) * inv_den);

    ctl_clear_qr_controller(qr);
}

void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs)
{
    qpr->kp = kp;
    ctl_init_qr_controller(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}

//void ctl_init_resonant_controller(
//    // handle of PR controller
//    resonant_ctrl_t* r,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // controller frequency
//    parameter_gt fs)
//{
//    // resonant frequency, unit rad/s
//    parameter_gt omega_r = CTL_PARAM_CONST_2PI * freq_resonant;
//
//    r->krg = float2ctrl(kr * (4 * fs) / (4 * fs * fs + omega_r * omega_r));
//    r->kr = float2ctrl(2 * (4 * fs * fs - omega_r * omega_r) / (4 * fs * fs + omega_r * omega_r));
//
//    // clear temp variables
//    ctl_clear_resonant_controller(r);
//}
//
//void ctl_init_pr_controller(
//    // handle of PR controller
//    pr_ctrl_t *pr,
//    // Kp
//    parameter_gt kp,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // controller frequency
//    parameter_gt fs)
//{
//    // resonant frequency, unit rad/s
//    parameter_gt omega_r = CTL_PARAM_CONST_2PI * freq_resonant;
//
//    pr->kpg = float2ctrl(kp);
//    pr->krg = float2ctrl(kr * (4 * fs) / (4 * fs * fs + omega_r * omega_r));
//    pr->kr = float2ctrl(2 * (4 * fs * fs - omega_r * omega_r) / (4 * fs * fs + omega_r * omega_r));
//
//    // clear temp variables
//    ctl_clear_pr_controller(pr);
//}
//
//void ctl_init_qr_controller(
//    // handle of QR controller
//    qr_ctrl_t* qr,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // cut frequency, unit Hz
//    parameter_gt freq_cut,
//    // controller frequency
//    parameter_gt fs)
//{
//    ctl_clear_qr_controller(qr);
//
//    parameter_gt omega_r_sqr = 4.0f * PI * PI * freq_resonant * freq_resonant;
//    parameter_gt omega_c_fs = 4.0f * 2.0f * PI * freq_cut * fs;
//
//    parameter_gt kr_suffix = 4.0f * freq_cut * fs / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    parameter_gt b1 = 2.0f * (4.0f * fs * fs - omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//    parameter_gt b2 = (4.0f * fs * fs - omega_c_fs + omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    // Discrete parameters
//    qr->a0 = float2ctrl(kr_suffix);
//    qr->b1 = float2ctrl(b1);
//    qr->b2 = float2ctrl(b2);
//    qr->kr = float2ctrl(kr);
//}
//
//void ctl_init_qpr_controller(
//    // handle of QPR controller
//    qpr_ctrl_t *qpr,
//    // Kp
//    parameter_gt kp,
//    // gain of resonant frequency
//    parameter_gt kr,
//    // resonant frequency, unit Hz
//    parameter_gt freq_resonant,
//    // cut frequency, unit Hz
//    parameter_gt freq_cut,
//    // controller frequency
//    parameter_gt fs)
//{
//    ctl_clear_qpr_controller(qpr);
//
//    parameter_gt omega_r_sqr = 4.0f * PI * PI * freq_resonant * freq_resonant;
//    parameter_gt omega_c_fs = 4.0f * 2.0f * PI * freq_cut * fs;
//
//    parameter_gt kr_suffix = 4.0f * freq_cut * fs / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    parameter_gt b1 = 2.0f * (4.0f * fs * fs - omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//    parameter_gt b2 = (4.0f * fs * fs - omega_c_fs + omega_r_sqr) / (4.0f * fs * fs + omega_c_fs + omega_r_sqr);
//
//    // Discrete parameters
//    qpr->a0 = float2ctrl(kr_suffix);
//    qpr->b1 = float2ctrl(b1);
//    qpr->b2 = float2ctrl(b2);
//
//    qpr->kp = float2ctrl(kp);
//    qpr->kr = float2ctrl(kr);
//}

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
    ctl_clear_discrete_sogi(sogi);

    parameter_gt osgx, osgy, temp, wn, delta_t;
    delta_t = 1.0f / fs;
    wn = fn * CTL_PARAM_CONST_2PI;
    // wn = fn * GMP_CONST_2_PI;

    osgx = (2.0f * k_damp * wn * delta_t);
    osgy = (parameter_gt)(wn * delta_t * wn * delta_t);
    temp = (parameter_gt)1.0 / (osgx + osgy + 4.0f);

    sogi->b0 = float2ctrl(osgx * temp);
    sogi->b2 = -sogi->b0;
    sogi->a1 = float2ctrl((2.0f * (4.0f - osgy)) * temp);
    sogi->a2 = float2ctrl((osgx - osgy - 4.0f) * temp);
    sogi->qb0 = float2ctrl((k_damp * osgy) * temp);
    sogi->qb1 = float2ctrl(sogi->qb0 * (2.0f));
    sogi->qb2 = sogi->qb0;
}

//////////////////////////////////////////////////////////////////////////
// Lead Lag controller

#include "ctl/component/intrinsic/discrete/lead_lag.h"

void ctl_init_lead(ctrl_lead_t* obj, parameter_gt K_D, parameter_gt tau_D, parameter_gt fs)
{
    // Sampling period
    parameter_gt T = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_D + T)
    parameter_gt den = 2.0f * tau_D + T;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*tau_D - T) / (2*tau_D + T)
    obj->a1 = float2ctrl((2.0f * tau_D - T) * inv_den);

    // b0 = (2*tau_D + 2*K_D + T) / (2*tau_D + T)
    obj->b0 = float2ctrl((2.0f * tau_D + 2.0f * K_D + T) * inv_den);

    // b1 = (T - 2*tau_D - 2*K_D) / (2*tau_D + T)
    obj->b1 = float2ctrl((T - 2.0f * tau_D - 2.0f * K_D) * inv_den);

    // Clear initial states
    ctl_clear_lead(obj);
}

void ctl_init_lead_form2(ctrl_lead_t* obj, parameter_gt alpha, parameter_gt T, parameter_gt fs)
{
    // Sampling period
    parameter_gt Ts = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_D + T)
    parameter_gt den = 2.0f * T + Ts;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*T - Ts) / (2*T + Ts)
    obj->a1 = float2ctrl((2.0f * T - Ts) * inv_den);

    // b0 = (2*alpha*T + T) / (2*T + Ts)
    obj->b0 = float2ctrl((2.0f * alpha * T + Ts) * inv_den);

    // b1 = (T - 2*alpha*T) / (2*T + Ts)
    obj->b1 = float2ctrl((Ts - 2.0f * alpha * T) * inv_den);

    // Clear initial states
    ctl_clear_lead(obj);
}

void ctl_init_lead_form3(ctrl_lead_t* obj, parameter_gt theta_rad, parameter_gt fc, parameter_gt fs)
{
    parameter_gt alpha;
    //parameter_gt theta_rad = angle_deg * (CTL_PARAM_CONST_PI / 180.0f); // 确保输入是弧度或进行转换

    // 1. 计算 Alpha (分频比)
    parameter_gt sin_val = sinf(theta_rad);
    alpha = (1.0f + sin_val) / (1.0f - sin_val);

    // 2. 正确计算时间常数 T
    // T = 1 / (omega_c * sqrt(alpha))
    parameter_gt omega_c = 2.0f * CTL_PARAM_CONST_PI * fc;
    parameter_gt T = 1.0f / (omega_c * sqrtf(alpha));

    // 3. 调用 Form2 进行离散化
    ctl_init_lead_form2(obj, alpha, T, fs);
}

//void ctl_init_lead_form3(ctrl_lead_t* obj, parameter_gt angle, parameter_gt fc, parameter_gt fs)
//{
//    parameter_gt alpha;
//
//    alpha = (1 + sinf(angle)) / (1 - sinf(angle));
//
//    ctl_init_lead_form2(obj, alpha, 1 / fc, fs);
//}

void ctl_init_lag(ctrl_lag_t* obj, parameter_gt tau_L, parameter_gt tau_P, parameter_gt fs)
{
    // Sampling period
    parameter_gt T = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_P + T)
    parameter_gt den = 2.0f * tau_P + T;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*tau_P - T) / (2*tau_P + T)
    obj->a1 = float2ctrl((2.0f * tau_P - T) * inv_den);

    // b0 = (2*tau_L + T) / (2*tau_P + T)
    obj->b0 = float2ctrl((2.0f * tau_L + T) * inv_den);

    // b1 = (T - 2*tau_L) / (2*tau_P + T)
    obj->b1 = float2ctrl((T - 2.0f * tau_L) * inv_den);

    // Clear initial states
    ctl_clear_lag(obj);
}

// Note: Implementations for ctl_init_2p2z and other generic pole-zero
// initializers would go here if they were defined.

//////////////////////////////////////////////////////////////////////////
// Z transfer function

#include <ctl/component/intrinsic/discrete/z_function.h>

void ctl_init_z_function(ctl_z_function_t* obj, int32_t num_order, const ctrl_gt* num_coeffs, int32_t den_order,
                         const ctrl_gt* den_coeffs, ctrl_gt* input_buffer, ctrl_gt* output_buffer)
{
    // Assign orders
    obj->num_order = num_order;
    obj->den_order = den_order;

    // Assign pointers to coefficients and state buffers
    obj->num_coeffs = num_coeffs;
    obj->den_coeffs = den_coeffs;
    obj->input_buffer = input_buffer;
    obj->output_buffer = output_buffer;

    // Clear all state buffers to ensure a clean start
    ctl_clear_z_function(obj);
}
