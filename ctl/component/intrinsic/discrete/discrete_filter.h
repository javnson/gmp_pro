/**
 * @file discrete_filter.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a library of common discrete filters.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file contains implementations for several standard discrete filters
 * used in digital control and signal processing. It includes a first-order
 * low-pass IIR filter, a generic second-order IIR filter configurable as
 * low-pass, high-pass, or band-pass, and a structure definition for a
 * general-purpose FIR filter.
 */

#ifndef _DISCRETE_FILTER_H_
#define _DISCRETE_FILTER_H_

#include <ctl/math_block/gmp_math.h>

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
/* 1st-Order IIR Low-Pass Filter                                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief A first-order infinite impulse response (IIR) low-pass filter.
 * @details This filter is implemented using the following difference equation,
 * which is derived from the Z-transform of the continuous-time transfer function.
 *
 * Continuous-time transfer function:
 * @f[
 * H(s) = \frac{\omega_c}{s+\omega_c}
 * @f]
 *
 * After discretization using a Zero-Order Hold (ZOH) equivalent, the Z-domain transfer function is:
 * @f[
 * H(z) = \frac{1-e^{-\omega_c T_s}}{1-e^{-\omega_c T_s} z^{-1}}
 * @f]
 *
 * This leads to the difference equation:
 * @f[
 * y(n) = a \cdot x(n) + (1-a) \cdot y(n-1)
 * @f]
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
    return float2ctrl(fc * 2 * PI / fs);
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
    lpf->out = ctl_mul(input, lpf->a) + ctl_mul(lpf->out, GMP_CONST_1 - lpf->a);
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
/* 2nd-Order IIR General Filter                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief A second-order infinite impulse response (IIR) general filter.
 * @details This module implements a standard biquad filter (Direct Form I).
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
typedef struct _tag_filter_IIR2_t
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
} ctl_filter_IIR2_t;

/**
 * @brief Executes one step of the 2nd-order IIR filter.
 * @param[in,out] obj Pointer to the IIR filter instance.
 * @param[in] input The current input sample, x[n].
 * @return ctrl_gt The calculated output sample, y[n].
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_filter_iir2(ctl_filter_IIR2_t* obj, ctrl_gt input)
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
 * @param[out] obj Pointer to the IIR filter instance.
 */
GMP_STATIC_INLINE void ctl_clear_filter_iir2(ctl_filter_IIR2_t* obj)
{
    obj->out = 0;
    obj->x[0] = 0;
    obj->x[1] = 0;
    obj->y[0] = 0;
    obj->y[1] = 0;
}

/**
 * @brief Gets the last calculated output from the IIR filter.
 * @param[in] obj Pointer to the IIR filter instance.
 * @return ctrl_gt The last output value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_filter_iir2_output(ctl_filter_IIR2_t* obj)
{
    return obj->out;
}

/**
 * @brief Enumeration for the supported 2nd-order IIR filter types.
 */
typedef enum _tag_filter_IIR2_type_t
{
    FILTER_IIR2_TYPE_LOWPASS = 0,  //!< Low-pass filter.
    FILTER_IIR2_TYPE_HIGHPASS = 1, //!< High-pass filter.
    FILTER_IIR2_TYPE_BANDPASS = 2, //!< Band-pass filter.
} filter_IIR2_type_t;

/**
 * @brief Setup structure for designing a 2nd-order IIR filter.
 */
typedef struct _tag_filter_IIR2_setup_t
{
    filter_IIR2_type_t filter_type; //!< The type of filter to design.
    parameter_gt fc;                //!< Center/cutoff frequency (Hz).
    parameter_gt fs;                //!< Sampling frequency (Hz).
    parameter_gt q;                 //!< Quality factor (determines bandwidth).
    parameter_gt gain;              //!< Desired gain at the passband.
} ctl_filter_IIR2_setup_t;

/**
 * @brief Designs and initializes a 2nd-order IIR filter based on design parameters.
 * @details This function calculates the 'a' and 'b' coefficients based on the
 * provided setup structure and populates the filter object.
 * @param[out] obj Pointer to the IIR filter instance to initialize.
 * @param[in] setup_obj Pointer to the setup structure with design parameters.
 */
void ctl_init_filter_iir2(ctl_filter_IIR2_t* obj, ctl_filter_IIR2_setup_t* setup_obj);

/*---------------------------------------------------------------------------*/
/* FIR Filter                                                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a Finite Impulse Response (FIR) filter.
 * @note This is a structure definition only. The associated functions for
 * initialization and stepping are not defined in this header.
 */
typedef struct _tag_filter_FIR_t
{
    ctrl_gt* parameters;  //!< Pointer to the array of filter coefficients (taps).
    ctrl_gt* data_buffer; //!< Pointer to the circular data buffer for input samples.
    size_gt order;        //!< The order of the FIR filter (number of taps - 1).
    size_gt cb_index;     //!< The current index for the circular buffer.
    ctrl_gt input;        //!< The last input value.
    ctrl_gt output;       //!< The last calculated output value.
} filter_fir_t;

/**
 * @}
 */ // end of discrete_filter_api group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _DISCRETE_FILTER_H_
