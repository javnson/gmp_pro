/**
 * @file discrete_filter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a library of common discrete filters, focusing on Biquad implementations.
 * @version 0.5
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file contains implementations for a standard second-order IIR filter
 * (Biquad) and a set of helper functions to initialize it as various common
 * filter types (Low-pass, High-pass, Band-pass, Notch, etc.). The coefficient
 * calculations are based on the Audio EQ Cookbook formulas, derived from the
 * Bilinear Transform with frequency pre-warping.
 */

#ifndef _DISCRETE_FILTER_H_
#define _DISCRETE_FILTER_H_

#include <ctl/math_block/gmp_math.h>
#include <math.h> // Required for tanf, cosf, sinf, atan2f, powf, sqrtf

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup discrete_filter_api Discrete Filter Library
 * @brief A collection of common discrete filters for signal processing.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* 2nd-Order IIR General Filter (Biquad) Core Implementation                 */
/*---------------------------------------------------------------------------*/

/**
 * @brief A second-order infinite impulse response (IIR) general filter.
 * @details This module implements a standard biquad filter using the Direct Form I
 * structure. The specific filter type (e.g., low-pass, high-pass) is determined
 * by the 'a' and 'b' coefficients, which are calculated by the various
 * initialization functions provided below.
 *
 * The Z-domain transfer function is:
 * @f[
 * H(z) = \frac{b_0 + b_1z^{-1} + b_2z^{-2}}{1 + a_1z^{-1} + a_2z^{-2}}
 * @f]
 *
 * The corresponding difference equation is:
 * @f[
 * y(n) = b_0x(n) + b_1x(n-1) + b_2x(n-2) - a_1y(n-1) - a_2y(n-2)
 * @f]
 */
typedef struct _tag_biquad_filter_t
{
    // Historical inputs: x[0] stores x[n-1], x[1] stores x[n-2]
    ctrl_gt x[2];
    // Historical outputs: y[0] stores y[n-1], y[1] stores y[n-2]
    ctrl_gt y[2];

    // Denominator coefficients: a[0] stores a1, a[1] stores a2
    ctrl_gt a[2];
    // Numerator coefficients: b[0] stores b0, b[1] stores b1, b[2] stores b2
    ctrl_gt b[3];

    // Last calculated output
    ctrl_gt out;
} ctl_biquad_filter_t;

/**
 * @brief Alias for ctl_biquad_filter_t for backward compatibility.
 */
typedef ctl_biquad_filter_t ctl_filter_IIR2_t;

/**
 * @brief Executes one step of the 2nd-order IIR filter.
 * @param[in,out] obj Pointer to the biquad filter instance.
 * @param[in] input The current input sample, x[n].
 * @return ctrl_gt The calculated output sample, y[n].
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_biquad_filter(ctl_biquad_filter_t* obj, ctrl_gt input)
{
    obj->out = ctl_mul(obj->b[0], input) + ctl_mul(obj->b[1], obj->x[0]) + ctl_mul(obj->b[2], obj->x[1]) -
               ctl_mul(obj->a[0], obj->y[0]) - ctl_mul(obj->a[1], obj->y[1]);

    // Update historical inputs
    obj->x[1] = obj->x[0];
    obj->x[0] = input;

    // Update historical outputs
    obj->y[1] = obj->y[0];
    obj->y[0] = obj->out;

    return obj->out;
}

/**
 * @brief Clears all internal states of the 2nd-order IIR filter.
 * @param[out] obj Pointer to the biquad filter instance.
 */
GMP_STATIC_INLINE void ctl_clear_biquad_filter(ctl_biquad_filter_t* obj)
{
    obj->out = 0;
    obj->x[0] = 0;
    obj->x[1] = 0;
    obj->y[0] = 0;
    obj->y[1] = 0;
}

/**
 * @brief Gets the last calculated output from the IIR filter.
 * @param[in] obj Pointer to the biquad filter instance.
 * @return ctrl_gt The last output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_biquad_filter_output(ctl_biquad_filter_t* obj)
{
    return obj->out;
}

/*---------------------------------------------------------------------------*/
/* Biquad Filter Initializers                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the biquad filter as a Low-Pass Filter (LPF).
 * @details
 * Characteristics: Passes frequencies below the cutoff frequency `fc` and attenuates
 * frequencies above it.
 * S-Domain Transfer Function:
 * @f[ H(s) = \frac{\omega_0^2}{s^2 + \frac{\omega_0}{Q}s + \omega_0^2} @f]
 * Implementation Steps:
 * 1. Calculate the normalized angular frequency `omega`.
 * 2. Calculate intermediate variables `alpha` and `cos_w0`.
 * 3. Calculate the denominator coefficient `a0_inv` for normalization.
 * 4. Calculate the final `a` and `b` coefficients for the difference equation.
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 * @param[in] Q Quality factor. Controls the resonance peak at the cutoff frequency. A value of 0.707 gives a Butterworth response.
 */
void ctl_init_biquad_lpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as a High-Pass Filter (HPF).
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 * @param[in] Q Quality factor.
 */
void ctl_init_biquad_hpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as a Band-Pass Filter (BPF).
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency (Hz).
 * @param[in] Q Quality factor. Higher Q means a narrower bandwidth.
 */
void ctl_init_biquad_bpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as a Notch Filter.
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency to notch out (Hz).
 * @param[in] Q Quality factor. Higher Q means a narrower notch.
 */
void ctl_init_biquad_notch(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as an All-Pass Filter.
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency where phase shift is -180 degrees (Hz).
 * @param[in] Q Quality factor. Controls how quickly the phase changes around fc.
 */
void ctl_init_biquad_allpass(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q)
{
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as a Peaking EQ Filter.
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency (Hz).
 * @param[in] Q Quality factor.
 * @param[in] gain_db The desired gain or cut in decibels (dB). Positive for boost, negative for cut.
 */
void ctl_init_biquad_peaking_eq(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                                parameter_gt gain_db)
{
    parameter_gt V0 = powf(10.0f, gain_db / 20.0f);
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as a Low-Shelf Filter.
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Corner frequency (Hz).
 * @param[in] Q Quality factor (shelf slope).
 * @param[in] gain_db The desired gain or cut for the low frequencies in dB.
 */
void ctl_init_biquad_lowshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                              parameter_gt gain_db)
{
    parameter_gt V0 = powf(10.0f, gain_db / 20.0f);
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/**
 * @brief Initializes the biquad filter as a High-Shelf Filter.
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Corner frequency (Hz).
 * @param[in] Q Quality factor (shelf slope).
 * @param[in] gain_db The desired gain or cut for the high frequencies in dB.
 */
void ctl_init_biquad_highshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                               parameter_gt gain_db)
{
    parameter_gt V0 = powf(10.0f, gain_db / 20.0f);
    parameter_gt omega = 2.0f * PI * fc / fs;
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

/*---------------------------------------------------------------------------*/
/* Biquad Filter Analysis                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Calculates the phase lag of the biquad filter at a specific frequency.
 * @details
 * This function evaluates the filter's complex frequency response H(e^(j¦ØT)) at
 * the given frequency `f`.
 * Key Steps:
 * 1. Calculate the normalized angular frequency: ¦ØT = 2*pi*f / fs.
 * 2. Evaluate the complex numerator N(¦Ø) = b0 + b1*e^(-j¦ØT) + b2*e^(-j2¦ØT).
 * 3. Evaluate the complex denominator D(¦Ø) = 1 + a1*e^(-j¦ØT) + a2*e^(-j2¦ØT).
 * 4. Calculate the phase of the numerator and denominator using atan2.
 * 5. The total phase is phase(N) - phase(D).
 * 6. The phase lag is the negative of the total phase.
 * @param[in] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the phase lag (Hz).
 * @return parameter_gt The phase lag in radians. A positive value indicates lag.
 */
parameter_gt ctl_get_biquad_phase_lag(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f)
{
    // 1. Calculate normalized angular frequency
    parameter_gt w = 2.0f * PI * f / fs;

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

/**
 * @brief Calculates the linear gain (magnitude) of the biquad filter at a specific frequency.
 * @details
 * This function evaluates the filter's magnitude response |H(e^(j¦ØT))| at the
 * given frequency `f`. To convert the result to decibels (dB), use the formula:
 * Gain_dB = 20 * log10(linear_gain).
 * Key Steps:
 * 1. Calculate the normalized angular frequency: ¦ØT = 2*pi*f / fs.
 * 2. Evaluate the complex numerator N(¦Ø) and denominator D(¦Ø).
 * 3. Calculate the magnitude of the numerator: |N(¦Ø)| = sqrt(real(N)^2 + imag(N)^2).
 * 4. Calculate the magnitude of the denominator: |D(¦Ø)| = sqrt(real(D)^2 + imag(D)^2).
 * 5. The total gain is |N(¦Ø)| / |D(¦Ø)|.
 * @param[in] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the gain (Hz).
 * @return parameter_gt The linear gain (magnitude).
 */
parameter_gt ctl_get_biquad_gain(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f)
{
    // 1. Calculate normalized angular frequency
    parameter_gt w = 2.0f * PI * f / fs;

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

    // 5. Total gain = |N(¦Ø)| / |D(¦Ø)|
    return mag_num / mag_den;
}

/**
 * @}
 */ // end of discrete_filter_api group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _DISCRETE_FILTER_H_
