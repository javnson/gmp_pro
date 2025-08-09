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

    if (mag_den < 1e-9)
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
    if (mag_den < 1e-9)
    {
        return 0.0f;
    }

    // 5. Total gain = |N(ω)| / |D(ω)|
    return mag_num / mag_den;
}

//////////////////////////////////////////////////////////////////////////
// FIR Filter
#include <ctl/component/intrinsic/discrete/fir_filter.h>

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

void ctl_init_df11(ctl_df11_t* df, parameter_gt b0, parameter_gt b1, parameter_gt a1)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->a1 = float2ctrl(a1);
    ctl_clear_df11(df);
}

void ctl_init_df22(ctl_df22_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt a1, parameter_gt a2)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    ctl_clear_df22(df);
}

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

void ctl_init_ramp_generator(ctl_ramp_generator_t* rg, ctrl_gt slope, parameter_gt amp_pos, parameter_gt amp_neg)
{
    rg->current = float2ctrl(0);

    rg->maximum = float2ctrl(amp_pos);
    rg->minimum = float2ctrl(amp_neg);

    rg->slope = slope;
}

void ctl_init_ramp_generator_via_freq(
    // pointer to ramp generator object
    ctl_ramp_generator_t* rg,
    // isr frequency, unit Hz
    parameter_gt isr_freq,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // ramp range
    parameter_gt amp_pos, parameter_gt amp_neg)
{
    rg->current = float2ctrl(0);

    rg->maximum = float2ctrl(amp_pos);
    rg->minimum = float2ctrl(amp_neg);

    float a = isr_freq / target_freq;
    float b = amp_pos - amp_neg;

    // rg->slope = float2ctrl((amp_pos - amp_neg) / (isr_freq / target_freq));
    rg->slope = float2ctrl(b / a);
}

void ctl_init_square_wave_generator(ctl_square_wave_generator_t* sq, parameter_gt fs, parameter_gt target_freq,
                                    parameter_gt amplitude, parameter_gt offset)
{
    sq->high_level = offset + amplitude;
    sq->low_level = offset - amplitude;
    sq->phase = 0.0f;
    sq->phase_step = 2.0f * PI * target_freq / fs;
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
// Pole-Zero Compensator

#include <ctl/component/intrinsic/discrete/pole_zero.h>

// Helper function to multiply two first-order polynomials: (b0 + b1*z^-1) * (c0 + c1*z^-1) -> out[0] + out[1]*z^-1 + out[2]*z^-2
static void _multiply_poly1_poly1(const ctrl_gt b[2], const ctrl_gt c[2], ctrl_gt out[3])
{
    out[0] = b[0] * c[0];
    out[1] = b[0] * c[1] + b[1] * c[0];
    out[2] = b[1] * c[1];
}

// Helper function to multiply a second-order and a first-order polynomial
static void _multiply_poly2_poly1(const ctrl_gt b[3], const ctrl_gt c[2], ctrl_gt out[4])
{
    out[0] = b[0] * c[0];
    out[1] = b[0] * c[1] + b[1] * c[0];
    out[2] = b[1] * c[1] + b[2] * c[0];
    out[3] = b[2] * c[1];
}

// Helper to calculate coefficients for a single real pole/zero section from s-plane frequencies
static void _calc_real_section_coeffs(parameter_gt f_z, parameter_gt f_p, parameter_gt fs, ctrl_gt b[2], ctrl_gt a[1])
{
    parameter_gt Kz = tanf(PI * f_z / fs);
    parameter_gt Kp = tanf(PI * f_p / fs);
    parameter_gt den_norm = Kp + 1.0f;
    if (den_norm < 1e-9f)
    {
        den_norm = 1e-9f;
    }

    b[0] = (Kz + 1.0f) / den_norm;
    b[1] = (Kz - 1.0f) / den_norm;
    a[0] = (Kp - 1.0f) / den_norm;
}

// Helper to calculate coefficients for a complex pole/zero pair section from s-plane locations
static void _calc_complex_section_coeffs(parameter_gt f_real, parameter_gt f_imag, parameter_gt fs, ctrl_gt out[3])
{
    parameter_gt sigma = 2.0f * CTL_PARAM_CONST_PI * f_real;
    parameter_gt wd = 2.0f * CTL_PARAM_CONST_PI * f_imag;
    parameter_gt two_fs = 2.0f * fs;

    parameter_gt den = 4 * fs * fs + 4 * fs * sigma + sigma * sigma + wd * wd;
    if (den < 1e-9f)
    {
        den = 1e-9f;
    }

    out[0] = (4 * fs * fs - 4 * fs * sigma + sigma * sigma + wd * wd) /
             den;                                                    // z^0 term (normalized to make z^-2 term = out[0])
    out[1] = (-8 * fs * fs + 2 * sigma * sigma + 2 * wd * wd) / den; // z^-1 term
    out[2] = 1.0f;                                                   // z^-2 term (by which we normalize)

    // Final normalization to make the z^0 term = 1 for the denominator
    parameter_gt norm = 1.0f / out[0];
    out[0] = 1.0f;
    out[1] *= norm;
    out[2] *= norm;
}

/*---------------------------------------------------------------------------*/
/* 1P1Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_1p1z(ctrl_1p1z_t* c, parameter_gt gain, parameter_gt f_z, parameter_gt f_p, parameter_gt fs)
{
    ctrl_gt b[2], a[1];
    _calc_real_section_coeffs(f_z, f_p, fs, b, a);

    // DC gain of H(s) = (s/wz+1)/(s/wp+1) is wp/wz. We need to scale by gain / (wp/wz).
    parameter_gt dc_gain_comp = (f_p > 1e-9f && f_z > 1e-9f) ? (f_p / f_z) : 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    c->coef_b[0] = b[0] * final_gain;
    c->coef_b[1] = b[1] * final_gain;
    c->coef_a[0] = a[0];

    ctl_clear_1p1z(c);
}

parameter_gt ctl_get_1p1z_phase_lag(ctrl_1p1z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w;

    return -(atan2f(num_imag, num_real) - atan2f(den_imag, den_real));
}

parameter_gt ctl_get_1p1z_gain(ctrl_1p1z_t* c, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);

    parameter_gt num_real = c->coef_b[0] + c->coef_b[1] * cos_w;
    parameter_gt num_imag = -c->coef_b[1] * sin_w;
    parameter_gt den_real = 1.0f + c->coef_a[0] * cos_w;
    parameter_gt den_imag = -c->coef_a[0] * sin_w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

/*---------------------------------------------------------------------------*/
/* 2P2Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_2p2z_real(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_p1,
                        parameter_gt f_p2, parameter_gt fs)
{
    ctrl_gt bz1[2], ap1[1], bz2[2], ap2[1];
    _calc_real_section_coeffs(f_z1, f_p1, fs, bz1, ap1);
    _calc_real_section_coeffs(f_z2, f_p2, fs, bz2, ap2);

    _multiply_poly1_poly1(bz1, bz2, c->coef_b);

    ctrl_gt den_poly1[2] = {1.0f, ap1[0]};
    ctrl_gt den_poly2[2] = {1.0f, ap2[0]};
    ctrl_gt den_final[3];
    _multiply_poly1_poly1(den_poly1, den_poly2, den_final);
    c->coef_a[0] = den_final[1];
    c->coef_a[1] = den_final[2];

    parameter_gt dc_gain_comp = (f_p1 * f_p2) / (f_z1 * f_z2);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (int i = 0; i < 3; ++i)
        c->coef_b[i] *= final_gain;
    ctl_clear_2p2z(c);
}

void ctl_init_2p2z_complex_zeros(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_p1, parameter_gt f_p2, parameter_gt fs)
{
    ctrl_gt ap1[1], ap2[1], b_dummy[2];
    _calc_complex_section_coeffs(f_czr, f_czi, fs, c->coef_b);

    _calc_real_section_coeffs(f_p1, f_p1, fs, b_dummy, ap1);
    _calc_real_section_coeffs(f_p2, f_p2, fs, b_dummy, ap2);

    ctrl_gt den_poly1[2] = {1.0f, ap1[0]};
    ctrl_gt den_poly2[2] = {1.0f, ap2[0]};
    ctrl_gt den_final[3];
    _multiply_poly1_poly1(den_poly1, den_poly2, den_final);
    c->coef_a[0] = den_final[1];
    c->coef_a[1] = den_final[2];

    parameter_gt dc_gain_comp = (f_p1 * f_p2) / (f_czr * f_czr + f_czi * f_czi);
    if (f_czr < 1e-9f && f_czi < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (int i = 0; i < 3; ++i)
        c->coef_b[i] *= final_gain;
    ctl_clear_2p2z(c);
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

    return -(atan2f(num_imag, num_real) - atan2f(den_imag, den_real));
}

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

/*---------------------------------------------------------------------------*/
/* 3P3Z Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_3p3z_real(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_z3,
                        parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3, parameter_gt fs)
{
    ctrl_2p2z_t sec1;
    ctrl_1p1z_t sec2;
    // Decompose into a 2P2Z and a 1P1Z section, each with unity gain
    ctl_init_2p2z_real(&sec1, 1.0, f_z1, f_z2, f_p1, f_p2, fs);
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    // Multiply polynomials to get the final direct-form coefficients
    ctrl_gt den_sec2_poly[2] = {1.0f, sec2.coef_a[0]};
    _multiply_poly2_poly1(sec1.coef_b, sec2.coef_b, c->coef_b);

    ctrl_gt den_sec1_poly[3] = {1.0f, sec1.coef_a[0], sec1.coef_a[1]};
    ctrl_gt den_final[4];
    _multiply_poly2_poly1(den_sec1_poly, den_sec2_poly, den_final);
    c->coef_a[0] = den_final[1];
    c->coef_a[1] = den_final[2];
    c->coef_a[2] = den_final[3];

    // Apply final gain scaling
    parameter_gt dc_gain_comp = (f_p1 * f_p2 * f_p3) / (f_z1 * f_z2 * f_z3);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f || f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (int i = 0; i < 4; ++i)
        c->coef_b[i] *= final_gain;
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_zeros(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_z3, parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3,
                                 parameter_gt fs)
{
    ctrl_2p2z_t sec1; // For the complex zero pair and two real poles
    ctrl_1p1z_t sec2; // For the remaining real zero and real pole
    ctl_init_2p2z_complex_zeros(&sec1, 1.0, f_czr, f_czi, f_p1, f_p2, fs);
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    ctrl_gt den_sec2_poly[2] = {1.0f, sec2.coef_a[0]};
    _multiply_poly2_poly1(sec1.coef_b, sec2.coef_b, c->coef_b);

    ctrl_gt den_sec1_poly[3] = {1.0f, sec1.coef_a[0], sec1.coef_a[1]};
    ctrl_gt den_final[4];
    _multiply_poly2_poly1(den_sec1_poly, den_sec2_poly, den_final);
    c->coef_a[0] = den_final[1];
    c->coef_a[1] = den_final[2];
    c->coef_a[2] = den_final[3];

    parameter_gt dc_gain_comp = (f_p1 * f_p2 * f_p3) / ((f_czr * f_czr + f_czi * f_czi) * f_z3);
    if (f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (int i = 0; i < 4; ++i)
        c->coef_b[i] *= final_gain;
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_poles(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2,
                                 parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                 parameter_gt fs)
{
    ctrl_2p2z_t sec1; // For two real zeros and the complex pole pair
    ctrl_1p1z_t sec2; // For the remaining real zero and real pole

    // Create a temporary 2P2Z with the complex poles
    ctrl_gt a_dummy[2], b_dummy[3];
    _calc_complex_section_coeffs(f_cpr, f_cpi, fs, b_dummy, sec1.coef_a);

    // Create the numerator from two real zeros
    ctrl_gt bz1[2], ap_dummy[1], bz2[2];
    _calc_real_section_coeffs(f_z1, f_z1, fs, bz1, ap_dummy);
    _calc_real_section_coeffs(f_z2, f_z2, fs, bz2, ap_dummy);
    _multiply_poly1_poly1(bz1, bz2, sec1.coef_b);

    // Create the second section from the remaining real pole and zero
    ctl_init_1p1z(&sec2, 1.0, f_z3, f_p3, fs);

    // Multiply the polynomials
    ctrl_gt den_sec2_poly[2] = {1.0f, sec2.coef_a[0]};
    _multiply_poly2_poly1(sec1.coef_b, sec2.coef_b, c->coef_b);

    ctrl_gt den_sec1_poly[3] = {1.0f, sec1.coef_a[0], sec1.coef_a[1]};
    ctrl_gt den_final[4];
    _multiply_poly2_poly1(den_sec1_poly, den_sec2_poly, den_final);
    c->coef_a[0] = den_final[1];
    c->coef_a[1] = den_final[2];
    c->coef_a[2] = den_final[3];

    parameter_gt dc_gain_comp = ((f_cpr * f_cpr + f_cpi * f_cpi) * f_p3) / (f_z1 * f_z2 * f_z3);
    if (f_z1 < 1e-9f || f_z2 < 1e-9f || f_z3 < 1e-9f)
        dc_gain_comp = 1.0f;
    parameter_gt final_gain = gain / dc_gain_comp;

    for (int i = 0; i < 4; ++i)
        c->coef_b[i] *= final_gain;
    ctl_clear_3p3z(c);
}

void ctl_init_3p3z_complex_pair(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                parameter_gt fs)
{
    ctrl_2p2z_t complex_sec; // For the complex zero and complex pole pair
    ctrl_1p1z_t real_sec;    // For the real zero and real pole

    // Create the 2P2Z section from the complex pairs
    _calc_complex_section_coeffs(f_czr, f_czi, fs, complex_sec.coef_b);
    _calc_complex_section_coeffs(f_cpr, f_cpi, fs, (ctrl_gt[3]){1.0, 0, 0}, complex_sec.coef_a);

    // Create the 1P1Z section from the real pair
    ctl_init_1p1z(&real_sec, 1.0, f_z3, f_p3, fs);

    // Multiply the polynomials
    ctrl_gt den_sec2_poly[2] = {1.0f, real_sec.coef_a[0]};
    _multiply_poly2_poly1(complex_sec.coef_b, real_sec.coef_b, c->coef_b);

    ctrl_gt den_sec1_poly[3] = {1.0f, complex_sec.coef_a[0], complex_sec.coef_a[1]};
    ctrl_gt den_final[4];
    _multiply_poly2_poly1(den_sec1_poly, den_sec2_poly, den_final);
    c->coef_a[0] = den_final[1];
    c->coef_a[1] = den_final[2];
    c->coef_a[2] = den_final[3];

    parameter_gt dc_gain_comp_c = (f_cpr * f_cpr + f_cpi * f_cpi) / (f_czr * f_czr + f_czi * f_czi);
    parameter_gt dc_gain_comp_r = (f_p3 / f_z3);
    if (f_z3 < 1e-9f)
        dc_gain_comp_r = 1.0f;
    parameter_gt final_gain = gain / (dc_gain_comp_c * dc_gain_comp_r);

    for (int i = 0; i < 4; ++i)
        c->coef_b[i] *= final_gain;
    ctl_clear_3p3z(c);
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

    return -(atan2f(num_imag, num_real) - atan2f(den_imag, den_real));
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

//
//void ctl_init_1p1z_from_freq(ctrl_1p1z_t* c, parameter_gt gain, parameter_gt f_z, parameter_gt f_p, parameter_gt fs)
//{
//    parameter_gt wz = 2.0f * CTL_PARAM_CONST_PI * f_z;
//    parameter_gt wp = 2.0f * CTL_PARAM_CONST_PI * f_p;
//
//    // The Bilinear Transform uses K = tan(w*T/2), but a direct algebraic substitution is clearer.
//    // H(z) = K_dc * (wp/wz) * (s+wz)/(s+wp) with s = 2*fs*(1-z^-1)/(1+z^-1)
//    parameter_gt two_fs = 2.0f * fs;
//    parameter_gt den_norm = two_fs + wp;
//
//    if (den_norm < 1e-9f)
//    { // Avoid division by zero
//        c->coef_b[0] = 0.0f;
//        c->coef_b[1] = 0.0f;
//        c->coef_a = 0.0f;
//        return;
//    }
//
//    // Gain correction factor
//    parameter_gt gain_corr = gain * wp / wz;
//    if (wz < 1e-9f)
//    { // Handle case where zero is at DC
//        gain_corr = gain;
//    }
//
//    // Calculate coefficients based on the derived difference equation
//    c->coef_b[0] = gain_corr * (two_fs + wz) / den_norm;
//    c->coef_b[1] = gain_corr * (wz - two_fs) / den_norm;
//    c->coef_a = (two_fs - wp) / den_norm; // This is the 'a1' for u(n) = a1*u(n-1) + ...
//
//    ctl_clear_1p1z(c);
//}
//
//
//// unit Hz
//void ctl_init_2p2z_from_freq(
//    // pointer to a 2p2z compensator
//    ctrl_2p2z_t* ctrl,
//    // gain of 2P2Z compensator
//    parameter_gt gain,
//    // two zero frequency, unit Hz
//    parameter_gt f_z0, parameter_gt f_z1,
//    // one pole frequency, unit Hz
//    parameter_gt f_p1,
//    // sample frequency
//    parameter_gt fs)
//{
//    parameter_gt z0 = f_z0 * CTL_PARAM_CONST_2PI;
//    parameter_gt z1 = f_z1 * CTL_PARAM_CONST_2PI;
//    parameter_gt p1 = f_p1 * CTL_PARAM_CONST_2PI;
//
//    // discrete controller parameter
//    parameter_gt gain_discrete = gain * (1.0f / 2.0f / fs / (p1 + 2.0f * fs));
//    parameter_gt b0 = (z1 + 2.0f * fs) * (z0 + 2.0f * fs);
//    parameter_gt b1 = ((z0 + 2.0f * fs) * (z1 - 2.0f * fs) + (z1 + 2.0f * fs) * (z0 - 2.0f * fs));
//    parameter_gt b2 = (z1 - 2.0f * fs) * (z0 - 2.0f * fs);
//    parameter_gt a1 = -(4.0f * fs / (p1 + 2.0f * fs));
//    parameter_gt a2 = (2.0f * fs - p1) / (2.0f * fs + p1);
//
//    ctrl->a1 = float2ctrl(a1);
//    ctrl->a2 = float2ctrl(a2);
//    ctrl->b0 = float2ctrl(b0);
//    ctrl->b1 = float2ctrl(b1);
//    ctrl->b2 = float2ctrl(b2);
//    ctrl->gain = float2ctrl(gain_discrete);
//
//    ctrl->out_max = float2ctrl(1.0);
//    ctrl->out_min = float2ctrl(-1.0);
//
//    ctrl->output_1 = 0;
//    ctrl->output_2 = 0;
//    ctrl->input_1 = 0;
//    ctrl->input_2 = 0;
//
//    ctrl->output = 0;
//    ctrl->input = 0;
//}
//
//void ctl_init_3p3z_from_freq(ctrl_3p3z_t* c, parameter_gt gain, const parameter_gt f_z[], const parameter_gt f_p[],
//                             parameter_gt fs)
//{
//    ctrl_1p1z_t sections[3];
//    parameter_gt total_dc_gain = 1.0f;
//
//    // Calculate coefficients for each of the three 1P1Z sections
//    for (int i = 0; i < 3; i++)
//    {
//        // --- Inlined logic from ctl_init_1p1z_from_freq ---
//        parameter_gt wz = 2.0f * CTL_PARAM_CONST_PI * f_z[i];
//        parameter_gt wp = 2.0f * CTL_PARAM_CONST_PI * f_p[i];
//        parameter_gt two_fs = 2.0f * fs;
//        parameter_gt den_norm = two_fs + wp;
//
//        if (den_norm < 1e-9f)
//        {
//            sections[i].coef_b[0] = 0.0f;
//            sections[i].coef_b[1] = 0.0f;
//            sections[i].coef_a = 0.0f;
//        }
//        else
//        {
//            // For each section, the gain is 1. The total gain is applied at the end.
//            parameter_gt gain_corr = 1.0f;
//            if (wz > 1e-9f)
//            {
//                gain_corr *= (wp / wz);
//            }
//            sections[i].coef_b[0] = gain_corr * (two_fs + wz) / den_norm;
//            sections[i].coef_b[1] = gain_corr * (wz - two_fs) / den_norm;
//            sections[i].coef_a = (two_fs - wp) / den_norm;
//        }
//    }
//
//    // Multiply the Z-domain polynomials of the three sections
//    // Numerator: B(z) = (b0_1 + b1_1*z^-1) * (b0_2 + b1_2*z^-1) * (b0_3 + b1_3*z^-1)
//    ctrl_gt b01 = sections[0].coef_b[0], b11 = sections[0].coef_b[1];
//    ctrl_gt b02 = sections[1].coef_b[0], b12 = sections[1].coef_b[1];
//    ctrl_gt b03 = sections[2].coef_b[0], b13 = sections[2].coef_b[1];
//
//    c->coef_b[0] = b01 * b02 * b03;
//    c->coef_b[1] = b01 * b02 * b13 + b01 * b12 * b03 + b11 * b02 * b03;
//    c->coef_b[2] = b01 * b12 * b13 + b11 * b02 * b13 + b11 * b12 * b03;
//    c->coef_b[3] = b11 * b12 * b13;
//
//    // Denominator: A(z) = (1 - a1_1*z^-1) * (1 - a1_2*z^-1) * (1 - a1_3*z^-1)
//    // A(z) = 1 - (a1+a2+a3)z^-1 + (a1a2+a1a3+a2a3)z^-2 - (a1a2a3)z^-3
//    ctrl_gt a11 = sections[0].coef_a, a12 = sections[1].coef_a, a13 = sections[2].coef_a;
//
//    c->coef_a[0] = a11 + a12 + a13;
//    c->coef_a[1] = -(a11 * a12 + a11 * a13 + a12 * a13);
//    c->coef_a[2] = a11 * a12 * a13;
//
//    // The DC gain of the combined filter with individual gains of 1 is 1.
//    // Simply apply the final desired gain to the numerator.
//    for (int i = 0; i < 4; i++)
//    {
//        c->coef_b[i] *= gain;
//    }
//
//    ctl_clear_3p3z(c);
//}


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
