/**
 * @file z_transfer_function.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a generic discrete Z-domain transfer function implementation.
 * @version 1.0
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _Z_TRANSFER_FUNCTION_H_
#define _Z_TRANSFER_FUNCTION_H_

#include <stdint.h> 

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/**
 * @defgroup z_transfer_function Generic Z-Domain Transfer Function
 * @brief An IIR filter module to implement any given Z-domain transfer function.
 * @details This file implements a generic, discrete-time transfer function (IIR filter)
 * based on its Z-domain representation. It can be used to realize arbitrary
 * linear time-invariant controllers or filters by specifying their numerator and
 * denominator coefficients.
 * This structure implements the general difference equation for a transfer function:
 * @f[ H(z) = \frac{Y(z)}{X(z)} = \frac{b_0 + b_1z^{-1} + \dots + b_Mz^{-M}}{1 + a_1z^{-1} + \dots + a_Nz^{-N}} @f]
 * which corresponds to the difference equation:
 * @f[ y(n) = b_0x(n) + \sum_{i=1}^{M} b_i x(n-i) - \sum_{j=1}^{N} a_j y(n-j) @f]
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Generic Z-Domain Transfer Function                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a generic Z-domain transfer function.
 * @details This structure implements the general difference equation for a transfer function:
 * @f[ H(z) = \frac{Y(z)}{X(z)} = \frac{b_0 + b_1z^{-1} + \dots + b_Mz^{-M}}{1 + a_1z^{-1} + \dots + a_Nz^{-N}} @f]
 * which corresponds to the difference equation:
 * @f[ y(n) = b_0x(n) + \sum_{i=1}^{M} b_i x(n-i) - \sum_{j=1}^{N} a_j y(n-j) @f]
 */
typedef struct _tag_z_function_t
{
    // Configuration
    int32_t num_order; //!< The order of the numerator (M).
    int32_t den_order; //!< The order of the denominator (N).

    // Coefficient pointers
    const ctrl_gt* num_coeffs; //!< Pointer to numerator coefficients [b0, b1, ..., bM].
    const ctrl_gt* den_coeffs; //!< Pointer to denominator coefficients [a1, a2, ..., aN].

    // State variable buffers
    ctrl_gt* input_buffer;  //!< Buffer for past inputs [x(n-1), ..., x(n-M)].
    ctrl_gt* output_buffer; //!< Buffer for past outputs [y(n-1), ..., y(n-N)].

} ctl_z_function_t;

/**
 * @brief Initializes the Z-domain transfer function module.
 * @param[out] obj Pointer to the z_function instance.
 * @param[in] num_order The order of the numerator (M).
 * @param[in] num_coeffs Pointer to an array of M+1 numerator coefficients (b0 to bM).
 * @param[in] den_order The order of the denominator (N).
 * @param[in] den_coeffs Pointer to an array of N denominator coefficients (a1 to aN). Note: a0 is assumed to be 1.
 * @param[in] input_buffer A user-provided buffer of size M to store past inputs.
 * @param[in] output_buffer A user-provided buffer of size N to store past outputs.
 */
void ctl_init_z_function(ctl_z_function_t* obj, int32_t num_order, const ctrl_gt* num_coeffs, int32_t den_order,
                         const ctrl_gt* den_coeffs, ctrl_gt* input_buffer, ctrl_gt* output_buffer);

/**
 * @brief Clears the internal state buffers of the transfer function.
 * @param[out] obj Pointer to the z_function instance.
 */
GMP_STATIC_INLINE void ctl_clear_z_function(ctl_z_function_t* obj)
{
    int32_t i;

    for (i = 0; i < obj->num_order; ++i)
    {
        obj->input_buffer[i] = 0;
    }
    for (i = 0; i < obj->den_order; ++i)
    {
        obj->output_buffer[i] = 0;
    }
}

/**
 * @brief Executes one step of the Z-domain transfer function calculation.
 * @param[in,out] obj Pointer to the z_function instance.
 * @param[in] input The current input value, x(n).
 * @return ctrl_gt The calculated output value, y(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_z_function(ctl_z_function_t* obj, ctrl_gt input)
{
    int32_t i;
    ctrl_gt output = 0;

    // Calculate numerator part: b0*x(n) + b1*x(n-1) + ...
    output = ctl_mul(obj->num_coeffs[0], input);
    for (i = 0; i < obj->num_order; ++i)
    {
        output += ctl_mul(obj->num_coeffs[i + 1], obj->input_buffer[i]);
    }

    // Calculate denominator part: -a1*y(n-1) - a2*y(n-2) - ...
    for (i = 0; i < obj->den_order; ++i)
    {
        output -= ctl_mul(obj->den_coeffs[i], obj->output_buffer[i]);
    }

    // Update state buffers for the next iteration
    // Shift past outputs
    for (i = obj->den_order - 1; i > 0; --i)
    {
        obj->output_buffer[i] = obj->output_buffer[i - 1];
    }
    if (obj->den_order > 0)
    {
        obj->output_buffer[0] = output;
    }

    // Shift past inputs
    for (i = obj->num_order - 1; i > 0; --i)
    {
        obj->input_buffer[i] = obj->input_buffer[i - 1];
    }
    if (obj->num_order > 0)
    {
        obj->input_buffer[0] = input;
    }

    return output;
}

/**
 * @}
 */ // end of z_transfer_function group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _Z_TRANSFER_FUNCTION_H_
