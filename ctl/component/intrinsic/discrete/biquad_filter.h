/**
 * @file biquad_filter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a library of common discrete filters, focusing on Biquad implementations.
 * @version 0.5
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _BIQUAD_FILTER_H_
#define _BIQUAD_FILTER_H_

#include <ctl/math_block/gmp_math.h>
#include <math.h> // Required for tanf, cosf, sinf, atan2f, powf, sqrtf

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup BIQUAD_filter_api Discrete Filter Library
 * @brief A collection of common discrete filters for signal processing.
 * @details This file contains implementations for a standard second-order IIR filter
 * (Biquad) and a set of helper functions to initialize it as various common
 * filter types (Low-pass, High-pass, Band-pass, Notch, etc.). The coefficient
 * calculations are based on the Audio EQ Cookbook formulas, derived from the
 * Bilinear Transform with frequency pre-warping.
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
void ctl_init_biquad_lpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q);

/**
 * @brief Initializes the biquad filter as a High-Pass Filter (HPF).
 * @details  
 * S-Domain Transfer Function:
 * @f[ H(s) = \frac{s^2}{s^2 + \frac{\omega_0}{Q}s + \omega_0^2} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 * @param[in] Q Quality factor.
 */
void ctl_init_biquad_hpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q);

/**
 * @brief Initializes the biquad filter as a Band-Pass Filter (BPF).
 * @details  
 * S-Domain Transfer Function:
 * @f[ H(s) = \frac{\frac{\omega_0}{Q} s}{s^2 + \frac{\omega_0}{Q}s + \omega_0^2} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency (Hz).
 * @param[in] Q Quality factor. Higher Q means a narrower bandwidth.
 */
void ctl_init_biquad_bpf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q);

/**
 * @brief Initializes the biquad filter as a Notch Filter.
 * @details  
 * S-Domain Transfer Function:
 * @f[ H(s) = \frac{s^2 + omega_0^2}{s^2 + \frac{\omega_0}{Q}s + \omega_0^2} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency to notch out (Hz).
 * @param[in] Q Quality factor. Higher Q means a narrower notch.
 */
void ctl_init_biquad_notch(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q);

/**
 * @brief Initializes the biquad filter as an All-Pass Filter.
 * @details  
 * S-Domain Transfer Function:
 * @f[ H(s) = \frac{s^2 - \frac{\omega_0}{Q}s + \omega_0^2}{s^2 + \frac{\omega_0}{Q}s + \omega_0^2} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency where phase shift is -180 degrees (Hz).
 * @param[in] Q Quality factor. Controls how quickly the phase changes around fc.
 */
void ctl_init_biquad_allpass(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q);

/**
 * @brief Initializes the biquad filter as a Peaking EQ Filter.
 * @details  
 * Peaking EQ is the most common type of equalization (EQ) used to enhance or attenuate 
 * a frequency band centered around a certain central frequency, with a frequency response curve 
 * that is bell shaped or peak shaped.
 * S-Domain Transfer Function:
 * @f[ H(s) = \frac{s^2 + \frac{A}{Q}s + \omega_0^2}{s^2 + \frac{1}{AQ}s + \omega_0^2} @f]
 * where @f[ A = 10^{gain_db} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency (Hz).
 * @param[in] Q Quality factor.
 * @param[in] gain_db The desired gain or cut in decibels (dB). Positive for boost, negative for cut.
 */
void ctl_init_biquad_peaking_eq(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                                parameter_gt gain_db);

/**
 * @brief Initializes the biquad filter as a Low-Shelf Filter.
 * @details  
 * The Low Self filter is used to enhance or attenuate all frequencies below the specified 
 * "inflection point frequency", and its frequency response curve is like a shelf.
 * S-Domain Transfer Function:
 * @f[ H(s) = A \cdot \frac{s^2 + \frac{\omega_0}{\sqrt{A}}s + \omega_0^2}{s^2 + \sqrt{A}\omega_0 s + \omega_0^2} @f]
 * where @f[ A = 10^{gain_db} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Corner frequency (Hz).
 * @param[in] Q Quality factor (shelf slope).
 * @param[in] gain_db The desired gain or cut for the low frequencies in dB.
 */
void ctl_init_biquad_lowshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                              parameter_gt gain_db);

/**
 * @brief Initializes the biquad filter as a High-Shelf Filter.
 * @details  
 * The High Self filter is the opposite of the Low Self filter, as it is used to enhance or attenuate 
 * all frequencies above a specified "inflection point frequency".
 * S-Domain Transfer Function:
 * @f[ H(s) = A \cdot \frac{s^2 + \frac{\sqrt{A}}{Q}\omega_0 s + \omega_0^2}{s^2 + \frac{1}{\sqrt{A}Q}\omega_0 s + \omega_0^2} @f]
 * where @f[ A = 10^{gain_db} @f]
 * 
 * @param[out] obj Pointer to the biquad filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Corner frequency (Hz).
 * @param[in] Q Quality factor (shelf slope).
 * @param[in] gain_db The desired gain or cut for the high frequencies in dB.
 */
void ctl_init_biquad_highshelf(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt fc, parameter_gt Q,
                               parameter_gt gain_db);

/*---------------------------------------------------------------------------*/
/* Biquad Filter Analysis                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Calculates the phase lag of the biquad filter at a specific frequency.
 * @details
 * 
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
parameter_gt ctl_get_biquad_phase_lag(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f);

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
parameter_gt ctl_get_biquad_gain(ctl_biquad_filter_t* obj, parameter_gt fs, parameter_gt f);

/**
 * @}
 */ // end of discrete_filter_api group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _DISCRETE_FILTER_H_
