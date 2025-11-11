/**
 * @file discrete_filter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a library of common discrete IIR and FIR filters.
 * @version 1.0
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _DISCRETE_FILTER_H_
#define _DISCRETE_FILTER_H_

#include <ctl/math_block/gmp_math.h>
#include <math.h> // Required for tanf, cosf, sinf, atan2f, sqrtf
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup discrete_filter_api Discrete Filter Library
 * @brief A collection of common discrete filters for signal processing.
 * @details This file contains implementations for several standard discrete filters
 * used in digital control and signal processing. It includes a generic first-order
 * and second-order IIR filter, along with various initializers. It also includes
 * a generic FIR filter module.

 * @{
 */

/*---------------------------------------------------------------------------*/
/* 1st-Order IIR Low-Pass Filter                                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief A first-order infinite impulse response (IIR) low-pass filter.
 * @details This filter is implemented using the following difference equation,
 * which is derived from the Z-transform of the continuous-time transfer function.
 *
 * Continuous-time transfer function:
 * @f[ H(s) = \frac{\omega_c}{s+\omega_c} @f]
 *
 * After discretization using a Zero-Order Hold (ZOH) equivalent, the Z-domain transfer function is:
 * @f[ H(z) = \frac{1-e^{-\omega_c T_s}}{1-e^{-\omega_c T_s} z^{-1}} @f]
 *
 * This leads to the difference equation:
 * @f[ y(n) = a \cdot x(n) + (1-a) \cdot y(n-1) @f]
 * where @f$ a = 1 - e^{-\omega_c T_s} @f$. For small @f$ \omega_c T_s @f$, this can be approximated as @f$ a \approx \omega_c T_s @f$.
 */
typedef struct _tag_low_pass_filter_t
{
    // parameters
    ctrl_gt a; //!< Filter coefficient, determines the cutoff frequency.

    // state
    ctrl_gt out; //!< Stores the previous output value, y[n-1].
} ctl_low_pass_filter_t;

/**
 * @brief Initializes a first-order low-pass filter object.
 * @param[out] lpf Pointer to the low-pass filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 */
void ctl_init_lp_filter(ctl_low_pass_filter_t* lpf, parameter_gt fs, parameter_gt fc);

/**
 * @brief Helper function to calculate the filter coefficient 'a'.
 * @details This calculates the approximate coefficient using @f$ a \approx \omega_c T_s = 2\pi f_c / f_s @f$.
 * @param fs Sample frequency (Hz).
 * @param fc Cutoff frequency (Hz).
 * @return ctrl_gt The calculated filter coefficient.
 */
GMP_STATIC_INLINE ctrl_gt ctl_helper_lp_filter(parameter_gt fs, parameter_gt fc)
{
    return float2ctrl(fc * CTL_PARAM_CONST_2PI / fs);
}

/**
 * @brief Calculates the approximate phase lag introduced by the filter at a given frequency.
 * @param fc Filter cutoff frequency (Hz).
 * @param finput Input signal frequency (Hz).
 * @return parameter_gt The phase lag in radians.
 */
GMP_STATIC_INLINE parameter_gt ctl_helper_get_lp_filter_lag_phase(parameter_gt fc, parameter_gt finput)
{
    return atanf(finput / fc);
}

/**
 * @brief Executes one step of the low-pass filter calculation.
 * @param[in,out] lpf Pointer to the low-pass filter instance.
 * @param[in] input The current input sample, x[n].
 * @return ctrl_gt The calculated output sample, y[n].
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_lowpass_filter(ctl_low_pass_filter_t* lpf, ctrl_gt input)
{
    // y[n] = a*x[n] + (1-a)*y[n-1]
    lpf->out = ctl_mul(input, lpf->a) + ctl_mul(lpf->out, CTL_CTRL_CONST_1 - lpf->a);
    return lpf->out;
}

/**
 * @brief Clears the internal state of the low-pass filter.
 * @details Resets the stored previous output to 0.
 * @param[out] lpf Pointer to the low-pass filter instance.
 */
GMP_STATIC_INLINE void ctl_clear_lowpass_filter(ctl_low_pass_filter_t* lpf)
{
    lpf->out = 0;
}

/**
 * @brief Gets the last calculated output from the filter.
 * @param[in] lpf Pointer to the low-pass filter instance.
 * @return ctrl_gt The last output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_lowpass_filter_result(ctl_low_pass_filter_t* lpf)
{
    return lpf->out;
}

/*---------------------------------------------------------------------------*/
/* 1st-Order IIR General Filter                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief A first-order infinite impulse response (IIR) general filter.
 * @details This module implements a standard first-order filter. The specific
 * filter type is determined by the coefficients calculated in the initializers.
 * The Z-domain transfer function is: 
 * @f[ H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1) @f]
 */
typedef struct _tag_filter_IIR1_t
{
    ctrl_gt x1;  //!< Stores the previous input, x[n-1].
    ctrl_gt y1;  //!< Stores the previous output, y[n-1].
    ctrl_gt a1;  //!< Denominator coefficient.
    ctrl_gt b0;  //!< Numerator coefficient.
    ctrl_gt b1;  //!< Numerator coefficient.
    ctrl_gt out; //!< Last calculated output.
} ctl_filter_IIR1_t;

/**
 * @brief Executes one step of the 1st-order IIR filter.
 * @param[in,out] obj Pointer to the IIR filter instance.
 * @param[in] input The current input sample, x[n].
 * @return ctrl_gt The calculated output sample, y[n].
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_filter_iir1(ctl_filter_IIR1_t* obj, ctrl_gt input)
{
    obj->out = (obj->b0 * input) + (obj->b1 * obj->x1) - (obj->a1 * obj->y1);
    obj->x1 = input;
    obj->y1 = obj->out;
    return obj->out;
}

/**
 * @brief Clears all internal states of the 1st-order IIR filter.
 * @param[out] obj Pointer to the IIR filter instance.
 */
GMP_STATIC_INLINE void ctl_clear_filter_iir1(ctl_filter_IIR1_t* obj)
{
    obj->out = 0;
    obj->x1 = 0;
    obj->y1 = 0;
}

/**
 * @brief Initializes the filter as a 1st-order Low-Pass Filter (LPF).
 * @details Based on the Tustin (trapezoidal) transformation of H(s) = wc / (s + wc).
 * @param[out] obj Pointer to the IIR filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 */
void ctl_init_filter_iir1_lpf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc);

/**
 * @brief Initializes the filter as a 1st-order High-Pass Filter (HPF).
 * @details Based on the Tustin transformation of H(s) = s / (s + wc).
 * @param[out] obj Pointer to the IIR filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Cutoff frequency (Hz).
 */
void ctl_init_filter_iir1_hpf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc);

/**
 * @brief Initializes the filter as a 1st-order All-Pass Filter.
 * @details Based on the Tustin transformation of H(s) = (s - wc) / (s + wc).
 * @param[out] obj Pointer to the IIR filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] fc Center frequency (Hz).
 */
void ctl_init_filter_iir1_apf(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt fc);

/**
 * @brief Calculates the phase lag of the 1st-order filter at a specific frequency.
 * @param[in] obj Pointer to the IIR filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the phase lag (Hz).
 * @return parameter_gt The phase lag in radians. A positive value indicates lag.
 */
parameter_gt ctl_get_filter_iir1_phase_lag(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt f);

/**
 * @brief Calculates the linear gain of the 1st-order filter at a specific frequency.
 * @param[in] obj Pointer to the IIR filter instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the gain (Hz).
 * @return parameter_gt The linear gain (magnitude).
 */
parameter_gt ctl_get_filter_iir1_gain(ctl_filter_IIR1_t* obj, parameter_gt fs, parameter_gt f);

/**
 * @}
 */ // end of discrete_filter_api group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _DISCRETE_FILTER_H_
