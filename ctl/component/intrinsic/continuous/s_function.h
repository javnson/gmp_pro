/**
 * @file s_transfer_function.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a generic S-domain transfer function implementation based on pole-zero placement.
 * @version 1.0
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a generic second-order continuous-time (S-domain)
 * transfer function. The user specifies the desired locations of poles and zeros
 * in the S-plane (as frequencies in Hz), and the module automatically discretizes
 * the system into a Z-domain difference equation using the Bilinear Transform.
 * This allows for intuitive design of controllers based on continuous-time methods.
 */

#ifndef _S_TRANSFER_FUNCTION_H_
#define _S_TRANSFER_FUNCTION_H_

#include <ctl/math_block/gmp_math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/**
 * @defgroup s_transfer_function Generic S-Domain Transfer Function
 * @brief An IIR filter module designed from S-domain pole-zero locations.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Generic S-Domain Transfer Function (2-Pole 2-Zero)                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 2-Pole-2-Zero S-domain transfer function.
 * @details This structure holds the discretized Z-domain coefficients and the
 * state variables needed to implement the difference equation. The underlying
 * continuous-time transfer function is of the form:
 * @f[
 * H(s) = K \frac{(s/\omega_{z1}+1)(s/\omega_{z2}+1)}{(s/\omega_{p1}+1)(s/\omega_{p2}+1)}
 * @f]
 * This is converted to a Z-domain transfer function:
 * @f[
 * H(z) = \frac{b_0 + b_1z^{-1} + b_2z^{-2}}{1 + a_1z^{-1} + a_2z^{-2}}
 * @f]
 */
typedef struct _tag_s_function_t
{
    // Z-domain coefficients
    ctrl_gt b0, b1, b2; //!< Numerator coefficients.
    ctrl_gt a1, a2;     //!< Denominator coefficients.

    // State variables
    ctrl_gt input_1;  //!< Previous input, x(n-1).
    ctrl_gt input_2;  //!< Input from two steps ago, x(n-2).
    ctrl_gt output_1; //!< Previous output, y(n-1).
    ctrl_gt output_2; //!< Output from two steps ago, y(n-2).
} ctl_s_function_t;

/**
 * @brief Initializes the S-domain transfer function from pole/zero frequencies.
 * @details This function takes the desired pole and zero locations (in Hz) on the
 * real axis of the S-plane, converts them to S-domain polynomial coefficients,
 * and then uses the bilinear transform to find the equivalent Z-domain
 * difference equation coefficients.
 * @param[out] obj Pointer to the s_function instance.
 * @param[in] gain The overall gain (K) of the transfer function.
 * @param[in] f_z1 Frequency of the first zero in Hz.
 * @param[in] f_z2 Frequency of the second zero in Hz.
 * @param[in] f_p1 Frequency of the first pole in Hz.
 * @param[in] f_p2 Frequency of the second pole in Hz.
 * @param[in] fs The sampling frequency in Hz.
 */
void ctl_init_s_function(ctl_s_function_t* obj, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2,
                         parameter_gt f_p1, parameter_gt f_p2, parameter_gt fs);

/**
 * @brief Clears the internal state buffers of the transfer function.
 * @param[out] obj Pointer to the s_function instance.
 */
GMP_STATIC_INLINE void ctl_clear_s_function(ctl_s_function_t* obj)
{
    obj->input_1 = 0;
    obj->input_2 = 0;
    obj->output_1 = 0;
    obj->output_2 = 0;
}

/**
 * @brief Executes one step of the transfer function calculation.
 * @details Implements the difference equation:
 * y(n) = -a1*y(n-1) - a2*y(n-2) + b0*x(n) + b1*x(n-1) + b2*x(n-2)
 * @param[in,out] obj Pointer to the s_function instance.
 * @param[in] input The current input value, x(n).
 * @return ctrl_gt The calculated output value, y(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_s_function(ctl_s_function_t* obj, ctrl_gt input)
{
    // Calculate the output using the difference equation
    ctrl_gt output = ctl_mul(obj->b0, input) + ctl_mul(obj->b1, obj->input_1) + ctl_mul(obj->b2, obj->input_2);
    output -= ctl_mul(obj->a1, obj->output_1);
    output -= ctl_mul(obj->a2, obj->output_2);

    // Update state buffers for the next iteration
    obj->input_2 = obj->input_1;
    obj->input_1 = input;
    obj->output_2 = obj->output_1;
    obj->output_1 = output;

    return output;
}

/**
 * @}
 */ // end of s_transfer_function group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _S_TRANSFER_FUNCTION_H_
