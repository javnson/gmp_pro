/**
 * @file pole_zero.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides discrete pole-zero compensators (1P1Z, 2P2Z, 3P3Z).
 * @version 0.2
 * @date 2025-03-19
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file contains implementations for several discrete-time pole-zero
 * compensators, which are fundamental building blocks for digital controllers.
 * These are essentially IIR filters designed to shape the frequency response of a
 * control loop. Implementations for 1-pole-1-zero, 2-pole-2-zero, and
 * 3-pole-3-zero compensators are provided. Discretization from the s-domain to
 * the z-domain is typically achieved using the Bilinear Transform:
 * @f[
 * s = \frac{2}{T} \frac{1-z^{-1}}{1+z^{-1}}
 * @f]
 */

#ifndef _POLE_ZERO_H_
#define _POLE_ZERO_H_


#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/**
 * @defgroup pole_zero_compensators Pole-Zero Compensators
 * @brief A library of discrete IIR filters for control loop compensation.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* 1-Pole-1-Zero (1P1Z) Compensator                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 1-Pole-1-Zero compensator.
 * @details Implements a first-order IIR filter.
 *
 * The transfer function in the Z-domain is:
 * @f[
 * \frac{U(z)}{E(z)} = \frac{b_0 + b_1 z^{-1}}{1 - a_1 z^{-1}}
 * @f]
 *
 * This corresponds to the difference equation:
 * @f[
 * u(n) = a_1 u(n-1) + b_0 e(n) + b_1 e(n-1)
 * @f]
 */
typedef struct _tag_ctrl_1p1z_t
{
    ctrl_gt output;    //!< The current output of the compensator, u(n).
    ctrl_gt coef_a;    //!< The pole coefficient, a1.
    ctrl_gt coef_b[2]; //!< The zero coefficients: coef_b[0] is b0, coef_b[1] is b1.
    ctrl_gt resp;      //!< The previous output state, u(n-1).
    ctrl_gt exct;      //!< The previous input state, e(n-1).

    ctrl_gt out_max;     //!< The absolute maximum output value.
    ctrl_gt out_sto_min; //!< The minimum value for storing the response state (anti-windup).
    ctrl_gt out_min;     //!< The absolute minimum output value.
} ctrl_1p1z_t;

/**
 * @brief Executes one step of the 1P1Z compensator.
 * @note An initialization function for this module is not defined in this header.
 * The user is responsible for setting the coefficients directly.
 * @param[in,out] c Pointer to the 1P1Z compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_1p1z(ctrl_1p1z_t* c, ctrl_gt input)
{
    // u(n) = a1*u(n-1) + b1*e(n-1) + b0*e(n)
    c->output = ctl_mul(c->coef_a, c->resp) + ctl_mul(c->coef_b[1], c->exct) + ctl_mul(c->coef_b[0], input);

    // Update states for the next iteration
    c->exct = input;
    // The response state is saturated for anti-windup before being stored
    c->resp = ctl_sat(c->output, c->out_max, c->out_sto_min);

    // The final output is saturated to its absolute limits
    c->output = ctl_sat(c->output, c->out_max, c->out_min);

    return c->output;
}

/*---------------------------------------------------------------------------*/
/* 2-Poles-2-Zeros (2P2Z) Compensator                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 2-Poles-2-Zeros compensator.
 * @details Implements a second-order IIR filter.
 *
 * The transfer function in the Z-domain is:
 * @f[
 * \frac{U(z)}{E(z)} = K \cdot \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 - a_1 z^{-1} - a_2 z^{-2}}
 * @f]
 *
 * This corresponds to the difference equation:
 * @f[
 * u(n) = a_1 u(n-1) + a_2 u(n-2) + K \cdot (b_0 e(n) + b_1 e(n-1) + b_2 e(n-2))
 * @f]
 */
typedef struct _tag_ctrl_2p2z_t
{
    // Coefficients
    ctrl_gt b0, b1, b2; //!< Numerator (zero) coefficients.
    ctrl_gt a1, a2;     //!< Denominator (pole) coefficients.
    ctrl_gt gain;       //!< Overall gain K of the compensator.

    // State variables
    ctrl_gt input;    //!< Current input, e(n).
    ctrl_gt output;   //!< Current output, u(n).
    ctrl_gt input_1;  //!< Previous input, e(n-1).
    ctrl_gt input_2;  //!< Input from two steps ago, e(n-2).
    ctrl_gt output_1; //!< Previous output, u(n-1).
    ctrl_gt output_2; //!< Output from two steps ago, u(n-2).

    // Limits
    ctrl_gt out_max; //!< Maximum output value.
    ctrl_gt out_min; //!< Minimum output value.
} ctrl_2p2z_t;

/**
 * @brief Initializes a 2P2Z compensator from pole/zero frequencies.
 * @param[out] ctrl Pointer to the 2P2Z compensator instance.
 * @param[in] gain Overall gain K of the compensator.
 * @param[in] f_z0 Frequency of the first zero (Hz).
 * @param[in] f_z1 Frequency of the second zero (Hz).
 * @param[in] f_p1 Frequency of the first pole (Hz). The second pole is at the origin.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_2p2z(ctrl_2p2z_t* ctrl, parameter_gt gain, parameter_gt f_z0, parameter_gt f_z1, parameter_gt f_p1,
                   parameter_gt fs);

/**
 * @brief Sets the output limits for the 2P2Z compensator.
 * @param[out] ctrl Pointer to the 2P2Z compensator instance.
 * @param[in] limit_max The maximum output value.
 * @param[in] limit_min The minimum output value.
 */
GMP_STATIC_INLINE void ctl_set_2p2z_limit(ctrl_2p2z_t* ctrl, ctrl_gt limit_max, ctrl_gt limit_min)
{
    ctrl->out_max = limit_max;
    ctrl->out_min = limit_min;
}

/**
 * @brief Executes one step of the 2P2Z compensator.
 * @param[in,out] ctrl Pointer to the 2P2Z compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_2p2z(ctrl_2p2z_t* ctrl, ctrl_gt input)
{
    ctrl->input = input;

    // Note: The original implementation applies gain partway through the calculation.
    // A more standard approach would be to pre-scale the 'b' coefficients by the gain.
    // The calculation has been reordered to match the standard difference equation.
    ctrl_gt numerator_out =
        ctl_mul(ctrl->b0, ctrl->input) + ctl_mul(ctrl->b1, ctrl->input_1) + ctl_mul(ctrl->b2, ctrl->input_2);
    numerator_out = ctl_mul(numerator_out, ctrl->gain);

    ctrl->output = numerator_out + ctl_mul(ctrl->a1, ctrl->output_1) + ctl_mul(ctrl->a2, ctrl->output_2);

    // Saturation
    ctrl->output = ctl_sat(ctrl->output, ctrl->out_max, ctrl->out_min);

    // Update states for the next iteration
    ctrl->input_2 = ctrl->input_1;
    ctrl->input_1 = ctrl->input;
    ctrl->output_2 = ctrl->output_1;
    ctrl->output_1 = ctrl->output; // Use the saturated output for anti-windup

    return ctrl->output;
}

/*---------------------------------------------------------------------------*/
/* 3-Poles-3-Zeros (3P3Z) Compensator                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 3-Poles-3-Zeros compensator.
 * @details Implements a third-order IIR filter.
 *
 * The transfer function in the Z-domain is:
 * @f[
 * \frac{U(z)}{E(z)} = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2} + b_3 z^{-3}}{1 - a_1 z^{-1} - a_2 z^{-2} - a_3 z^{-3}}
 * @f]
 *
 * This corresponds to the difference equation:
 * @f[
 * u(n) = a_1 u(n-1) + a_2 u(n-2) + a_3 u(n-3) + b_0 e(n) + b_1 e(n-1) + b_2 e(n-2) + b_3 e(n-3)
 * @f]
 */
typedef struct _tag_ctrl_3p3z_t
{
    ctrl_gt output;    //!< The current output of the compensator, u(n).
    ctrl_gt coef_a[3]; //!< Pole coefficients: a1, a2, a3.
    ctrl_gt coef_b[4]; //!< Zero coefficients: b0, b1, b2, b3.
    ctrl_gt resp[3];   //!< Previous output states: u(n-1), u(n-2), u(n-3).
    ctrl_gt exct[3];   //!< Previous input states: e(n-1), e(n-2), e(n-3).

    ctrl_gt out_max;     //!< The absolute maximum output value.
    ctrl_gt out_sto_min; //!< The minimum value for storing the response state (anti-windup).
    ctrl_gt out_min;     //!< The absolute minimum output value.
} ctrl_3p3z_t;

/**
 * @brief Executes one step of the 3P3Z compensator.
 * @note An initialization function for this module is not defined in this header.
 * The user is responsible for setting the coefficients directly.
 * @param[in,out] c Pointer to the 3P3Z compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_3p3z(ctrl_3p3z_t* c, ctrl_gt input)
{
    // u(n) = a1*u(n-1) + a2*u(n-2) + a3*u(n-3) + b0*e(n) + ...
    c->output =
        ctl_mul(c->coef_a[0], c->resp[0]) + ctl_mul(c->coef_a[1], c->resp[1]) + ctl_mul(c->coef_a[2], c->resp[2]);
    c->output +=
        ctl_mul(c->coef_b[1], c->exct[0]) + ctl_mul(c->coef_b[2], c->exct[1]) + ctl_mul(c->coef_b[3], c->exct[2]);
    c->output += ctl_mul(c->coef_b[0], input);

    // Update input state buffer
    c->exct[2] = c->exct[1];
    c->exct[1] = c->exct[0];
    c->exct[0] = input;

    // The final output is saturated to its absolute limits
    c->output = ctl_sat(c->output, c->out_max, c->out_min);

    // Update response state buffer with saturated output for anti-windup
    c->resp[2] = c->resp[1];
    c->resp[1] = c->resp[0];
    c->resp[0] = c->output;

    return c->output;
}

/**
 * @}
 */ // end of pole_zero_compensators group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _POLE_ZERO_H_
