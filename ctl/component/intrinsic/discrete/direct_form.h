/**
 * @file direct_form.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides implementations for Direct Form (DF) controllers.
 * @version 0.5
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file contains implementations for various standard Direct Form
 * IIR filter structures, commonly used in digital control systems. This module
 * includes DF11, DF22, DF13 and DF23 structures.
 * reference TI DF controller
 */

#ifndef _DF_CONTROLLER_H_
#define _DF_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/**
 * @defgroup df_controllers Direct Form Controllers
 * @brief A library of standard direct form IIR filter implementations.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Direct Form I, 1st-Order (DF11) Controller                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the DF11 controller.
 * @details This structure implements a first-order IIR filter using the
 * Direct Form I topology.
 *
 * The transfer function is:
 * @f[ H(z) = \frac{b_0 + b_1 z^{-1}}{1 + a_1 z^{-1}} @f]
 */
typedef struct _tag_df11_controller_t
{
    // Coefficients
    ctrl_gt a1; //!< The pole coefficient a1. Note: The block diagram shows -a1, so the input should be the positive a1.
    ctrl_gt b0; //!< The zero coefficient b0.
    ctrl_gt b1; //!< The zero coefficient b1.

    // State variables (delay line)
    ctrl_gt d1; //!< Stores the previous input, e(k-1).
    ctrl_gt d2; //!< Stores the previous output, u(k-1).

    // Output
    ctrl_gt output; //!< The current controller output, u(k).
} ctl_df11_t;

/**
 * @brief Clears the internal state variables of the DF11 controller.
 * @param[out] df Pointer to the DF11 controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_df11(ctl_df11_t* df)
{
    df->d1 = 0.0f;
    df->d2 = 0.0f;
    df->output = 0.0f;
}

/**
 * @brief Initializes the DF11 controller.
 * @param[out] df Pointer to the DF11 controller instance.
 * @param[in] b0 Numerator coefficient b0.
 * @param[in] b1 Numerator coefficient b1.
 * @param[in] a1 Denominator coefficient a1 (from 1 + a1*z^-1).
 */
void ctl_init_df11(ctl_df11_t* df, parameter_gt b0, parameter_gt b1, parameter_gt a1);

/**
 * @brief Executes one step of the DF11 controller.
 * @param[in,out] df Pointer to the DF11 controller instance.
 * @param[in] input The current input error signal, e(k).
 * @return ctrl_gt The calculated output, u(k).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_df11(ctl_df11_t* df, ctrl_gt input)
{
    ctrl_gt v2 = ctl_mul(df->b0, input);
    ctrl_gt v3 = ctl_mul(df->b1, df->d1);
    ctrl_gt v6 = ctl_mul(-df->a1, df->d2);
    ctrl_gt v4 = v2 + v6;
    df->output = v4 + v3;

    df->d1 = input;
    df->d2 = df->output;

    return df->output;
}

/*---------------------------------------------------------------------------*/
/* Direct Form II, 2nd-Order (DF22) Controller                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the DF22 controller.
 * @details This structure implements a second-order IIR filter using the
 * Direct Form II topology. This form is canonical, meaning it uses the minimum
 * number of delay elements for a given transfer function.
 *
 * The transfer function is:
 * @f[ H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}} @f]
 */
typedef struct _tag_df22_controller_t
{
    // Coefficients
    ctrl_gt a1, a2;     //!< Denominator coefficients.
    ctrl_gt b0, b1, b2; //!< Numerator coefficients.

    // State variables (delay line for the intermediate variable 'x')
    ctrl_gt x1d; //!< Stores the intermediate state, x(k-1).
    ctrl_gt x2d; //!< Stores the intermediate state, x(k-2).

    // Output
    ctrl_gt output; //!< The current controller output, u(k).
} ctl_df22_t;

/**
 * @brief Clears the internal state variables of the DF22 controller.
 * @param[out] df Pointer to the DF22 controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_df22(ctl_df22_t* df)
{
    df->x1d = 0.0f;
    df->x2d = 0.0f;
    df->output = 0.0f;
}

/**
 * @brief Initializes the DF22 controller.
 * @param[out] df Pointer to the DF22 controller instance.
 * @param[in] b0, b1, b2 Numerator coefficients.
 * @param[in] a1, a2 Denominator coefficients.
 */
void ctl_init_df22(ctl_df22_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt a1, parameter_gt a2);

/**
 * @brief Executes one step of the DF22 controller.
 * @param[in,out] df Pointer to the DF22 controller instance.
 * @param[in] input The current input error signal, e(k).
 * @return ctrl_gt The calculated output, u(k).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_df22(ctl_df22_t* df, ctrl_gt input)
{
    ctrl_gt v3 = ctl_mul(-df->a1, df->x1d);
    ctrl_gt v4 = ctl_mul(-df->a2, df->x2d);
    ctrl_gt x_k = input + v3 + v4;

    ctrl_gt v0 = ctl_mul(df->b0, x_k);
    ctrl_gt v1 = ctl_mul(df->b1, df->x1d);
    ctrl_gt v2 = ctl_mul(df->b2, df->x2d);
    df->output = v0 + v1 + v2;

    df->x2d = df->x1d;
    df->x1d = x_k;

    return df->output;
}

/*---------------------------------------------------------------------------*/
/* Direct Form I, 3rd-Order (DF13) Controller                                */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the DF13 controller.
 * @details This structure implements a third-order IIR filter using the
 * Direct Form I topology.
 *
 * The transfer function is:
 * @f[ H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2} + b_3 z^{-3}}{1 + a_1 z^{-1} + a_2 z^{-2} + a_3 z^{-3}} @f]
 */
typedef struct _tag_df13_controller_t
{
    // Coefficients
    ctrl_gt a1, a2, a3;     //!< Denominator coefficients.
    ctrl_gt b0, b1, b2, b3; //!< Numerator coefficients.

    // State variables (delay lines)
    ctrl_gt d1, d2, d3; //!< Stores past inputs: e(k-1), e(k-2), e(k-3).
    ctrl_gt d5, d6, d7; //!< Stores past outputs: u(k-1), u(k-2), u(k-3).

    // Output
    ctrl_gt output; //!< The current controller output, u(k).
} ctl_df13_t;

/**
 * @brief Clears the internal state variables of the DF13 controller.
 * @param[out] df Pointer to the DF13 controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_df13(ctl_df13_t* df)
{
    df->d1 = 0.0f;
    df->d2 = 0.0f;
    df->d3 = 0.0f;
    df->d5 = 0.0f;
    df->d6 = 0.0f;
    df->d7 = 0.0f;
    df->output = 0.0f;
}

/**
 * @brief Initializes the DF13 controller.
 * @param[out] df Pointer to the DF13 controller instance.
 * @param[in] b0, b1, b2, b3 Numerator coefficients.
 * @param[in] a1, a2, a3 Denominator coefficients.
 */
void ctl_init_df13(ctl_df13_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt b3, parameter_gt a1,
                   parameter_gt a2, parameter_gt a3);

/**
 * @brief Executes one step of the DF13 controller.
 * @param[in,out] df Pointer to the DF13 controller instance.
 * @param[in] input The current input error signal, e(k).
 * @return ctrl_gt The calculated output, u(k).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_df13(ctl_df13_t* df, ctrl_gt input)
{
    ctrl_gt v5 = ctl_mul(-df->a1, df->d5);
    ctrl_gt v6 = ctl_mul(-df->a2, df->d6);
    ctrl_gt v7 = ctl_mul(-df->a3, df->d7);

    ctrl_gt v0 = ctl_mul(df->b0, input);
    ctrl_gt v1 = ctl_mul(df->b1, df->d1);
    ctrl_gt v2 = ctl_mul(df->b2, df->d2);
    ctrl_gt v3 = ctl_mul(df->b3, df->d3);

    df->output = v0 + v1 + v2 + v3 + v5 + v6 + v7;

    df->d7 = df->d6;
    df->d6 = df->d5;
    df->d5 = df->output;
    df->d3 = df->d2;
    df->d2 = df->d1;
    df->d1 = input;

    return df->output;
}

/*---------------------------------------------------------------------------*/
/* Direct Form II Transposed, 3rd-Order (DF23) Controller                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the DF23 controller.
 * @details This structure implements a third-order IIR filter using the
 * Direct Form II Transposed topology. This form is canonical and can offer
 * better numerical stability in some fixed-point implementations.
 *
 * The transfer function is:
 * @f[ H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2} + b_3 z^{-3}}{1 + a_1 z^{-1} + a_2 z^{-2} + a_3 z^{-3}} @f]
 */
typedef struct _tag_df23_controller_t
{
    // Coefficients
    ctrl_gt a1, a2, a3;     //!< Denominator coefficients.
    ctrl_gt b0, b1, b2, b3; //!< Numerator coefficients.

    // State variables (delay line)
    ctrl_gt d1, d2, d3; //!< Stores the state variables of the transposed structure.

    // Output
    ctrl_gt output; //!< The current controller output, u(k).
} ctl_df23_t;

/**
 * @brief Clears the internal state variables of the DF23 controller.
 * @param[out] df Pointer to the DF23 controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_df23(ctl_df23_t* df)
{
    df->d1 = 0.0f;
    df->d2 = 0.0f;
    df->d3 = 0.0f;
    df->output = 0.0f;
}

/**
 * @brief Initializes the DF23 controller.
 * @param[out] df Pointer to the DF23 controller instance.
 * @param[in] b0, b1, b2, b3 Numerator coefficients.
 * @param[in] a1, a2, a3 Denominator coefficients.
 */
void ctl_init_df23(ctl_df23_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt b3, parameter_gt a1,
                   parameter_gt a2, parameter_gt a3);

/**
 * @brief Executes one step of the DF23 controller.
 * @param[in,out] df Pointer to the DF23 controller instance.
 * @param[in] input The current input error signal, e(k).
 * @return ctrl_gt The calculated output, u(k).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_df23(ctl_df23_t* df, ctrl_gt input)
{
    // Calculate output u(k)
    df->output = ctl_mul(df->b0, input) + df->d1;

    // Calculate new state variables
    ctrl_gt d1_new = ctl_mul(df->b1, input) - ctl_mul(df->a1, df->output) + df->d2;
    ctrl_gt d2_new = ctl_mul(df->b2, input) - ctl_mul(df->a2, df->output) + df->d3;
    ctrl_gt d3_new = ctl_mul(df->b3, input) - ctl_mul(df->a3, df->output);

    // Update state variables
    df->d1 = d1_new;
    df->d2 = d2_new;
    df->d3 = d3_new;

    return df->output;
}

/**
 * @}
 */ // end of df_controllers group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _DF_CONTROLLER_H_
