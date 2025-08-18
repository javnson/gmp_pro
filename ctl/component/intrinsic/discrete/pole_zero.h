/**
 * @file pole_zero.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides discrete pole-zero compensators (1P1Z, 2P2Z, 3P3Z) with frequency-based design.
 * @version 2.0
 * @date 2025-08-09
 *
 * @copyright Copyright GMP(c) 2024
 *
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
 * @details This file contains implementations for several discrete-time pole-zero
 * compensators. These are fundamental building blocks for digital controllers,
 * designed to shape the frequency response of a control loop. This version
 * provides initialization functions based on desired pole and zero frequencies,
 * including support for complex-conjugate pairs. Discretization from the s-domain
 * to the z-domain is achieved using the Bilinear Transform.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* 1-Pole-1-Zero (1P1Z) Compensator                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 1-Pole-1-Zero compensator.
 * @details Implements H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
 */
typedef struct _tag_ctrl_1p1z_t
{
    ctrl_gt coef_a[1]; //!< Pole coefficients: [a1].
    ctrl_gt coef_b[2]; //!< Zero coefficients: [b0, b1].
    ctrl_gt resp[1];   //!< Previous output states: [u(n-1)].
    ctrl_gt exct[1];   //!< Previous input states: [e(n-1)].
    ctrl_gt output;    //!< The current output of the compensator, u(n).
} ctrl_1p1z_t;

/**
 * @brief Clears the internal states of the 1P1Z compensator.
 * @param[out] c Pointer to the 1P1Z compensator instance.
 */
GMP_STATIC_INLINE void ctl_clear_1p1z(ctrl_1p1z_t* c)
{
    c->output = 0.0f;
    c->resp[0] = 0.0f;
    c->exct[0] = 0.0f;
}

/**
 * @brief Initializes a 1P1Z compensator from a real pole and a real zero frequency.
 * @param[out] c Pointer to the 1P1Z compensator instance.
 * @param[in] gain The desired DC gain of the compensator.
 * @param[in] f_z The frequency of the zero in Hz.
 * @param[in] f_p The frequency of the pole in Hz.
 * @param[in] fs The sampling frequency in Hz.
 */
void ctl_init_1p1z(ctrl_1p1z_t* c, parameter_gt gain, parameter_gt f_z, parameter_gt f_p, parameter_gt fs);

/**
 * @brief Executes one step of the 1P1Z compensator.
 * @param[in,out] c Pointer to the 1P1Z compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_1p1z(ctrl_1p1z_t* c, ctrl_gt input)
{
    c->output = ctl_mul(c->coef_b[0], input) + ctl_mul(c->coef_b[1], c->exct[0]) - ctl_mul(c->coef_a[0], c->resp[0]);
    c->exct[0] = input;
    c->resp[0] = c->output;
    return c->output;
}

/**
 * @brief Calculates the phase lag of the 1P1Z compensator at a specific frequency.
 * @param[in] c Pointer to the 1P1Z compensator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the phase lag (Hz).
 * @return parameter_gt The phase lag in radians. A positive value indicates lag.
 */
parameter_gt ctl_get_1p1z_phase_lag(ctrl_1p1z_t* c, parameter_gt fs, parameter_gt f);

/**
 * @brief Calculates the linear gain of the 1P1Z compensator at a specific frequency.
 * @param[in] c Pointer to the 1P1Z compensator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the gain (Hz).
 * @return parameter_gt The linear gain (magnitude).
 */
parameter_gt ctl_get_1p1z_gain(ctrl_1p1z_t* c, parameter_gt fs, parameter_gt f);

/*---------------------------------------------------------------------------*/
/* 2-Poles-2-Zeros (2P2Z) Compensator                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 2-Poles-2-Zeros compensator.
 */
typedef struct _tag_ctrl_2p2z_t
{
    ctrl_gt coef_a[2]; //!< Pole coefficients: [a1, a2].
    ctrl_gt coef_b[3]; //!< Zero coefficients: [b0, b1, b2].
    ctrl_gt resp[2];   //!< Previous output states: [u(n-1), u(n-2)].
    ctrl_gt exct[2];   //!< Previous input states: [e(n-1), e(n-2)].
    ctrl_gt output;    //!< The current output of the compensator, u(n).
} ctrl_2p2z_t;

/**
 * @brief Clears the internal states of the 2P2Z compensator.
 * @param[out] c Pointer to the 2P2Z compensator instance.
 */
GMP_STATIC_INLINE void ctl_clear_2p2z(ctrl_2p2z_t* c)
{
    c->output = 0.0f;
    c->resp[0] = 0.0f;
    c->resp[1] = 0.0f;
    c->exct[0] = 0.0f;
    c->exct[1] = 0.0f;
}

/**
 * @brief Initializes a 2P2Z compensator with two real poles and two real zeros.
 * @param[out] c Pointer to the 2P2Z compensator instance.
 * @param[in] gain The desired DC gain.
 * @param[in] f_z1 Frequency of the first zero (Hz).
 * @param[in] f_z2 Frequency of the second zero (Hz).
 * @param[in] f_p1 Frequency of the first pole (Hz).
 * @param[in] f_p2 Frequency of the second pole (Hz).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_2p2z_real(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_p1,
                        parameter_gt f_p2, parameter_gt fs);

/**
 * @brief Initializes a 2P2Z compensator with two real poles and a pair of complex-conjugate zeros.
 * @param[out] c Pointer to the 2P2Z compensator instance.
 * @param[in] gain The desired DC gain.
 * @param[in] f_czr Real part of the complex zero location in the s-plane (-sigma, in Hz).
 * @param[in] f_czi Imaginary part of the complex zero location in the s-plane (wd, in Hz).
 * @param[in] f_p1 Frequency of the first pole (Hz).
 * @param[in] f_p2 Frequency of the second pole (Hz).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_2p2z_complex_zeros(ctrl_2p2z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_p1, parameter_gt f_p2, parameter_gt fs);

/**
 * @brief Executes one step of the 2P2Z compensator.
 * @param[in,out] c Pointer to the 2P2Z compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_2p2z(ctrl_2p2z_t* c, ctrl_gt input)
{
    c->output = ctl_mul(c->coef_b[0], input) + ctl_mul(c->coef_b[1], c->exct[0]) + ctl_mul(c->coef_b[2], c->exct[1]) -
                (ctl_mul(c->coef_a[0], c->resp[0]) + ctl_mul(c->coef_a[1], c->resp[1]));

    c->exct[1] = c->exct[0];
    c->exct[0] = input;
    c->resp[1] = c->resp[0];
    c->resp[0] = c->output;

    return c->output;
}

/**
 * @brief Calculates the phase lag of the 2P2Z compensator at a specific frequency.
 * @param[in] c Pointer to the 2P2Z compensator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the phase lag (Hz).
 * @return parameter_gt The phase lag in radians.
 */
parameter_gt ctl_get_2p2z_phase_lag(ctrl_2p2z_t* c, parameter_gt fs, parameter_gt f);

/**
 * @brief Calculates the linear gain of the 2P2Z compensator at a specific frequency.
 * @param[in] c Pointer to the 2P2Z compensator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the gain (Hz).
 * @return parameter_gt The linear gain (magnitude).
 */
parameter_gt ctl_get_2p2z_gain(ctrl_2p2z_t* c, parameter_gt fs, parameter_gt f);

/*---------------------------------------------------------------------------*/
/* 3-Poles-3-Zeros (3P3Z) Compensator                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a 3-Poles-3-Zeros compensator.
 */
typedef struct _tag_ctrl_3p3z_t
{
    ctrl_gt coef_a[3]; //!< Pole coefficients: [a1, a2, a3].
    ctrl_gt coef_b[4]; //!< Zero coefficients: [b0, b1, b2, b3].
    ctrl_gt resp[3];   //!< Previous output states: [u(n-1), u(n-2), u(n-3)].
    ctrl_gt exct[3];   //!< Previous input states: [e(n-1), e(n-2), e(n-3)].
    ctrl_gt output;    //!< The current output of the compensator, u(n).
} ctrl_3p3z_t;

/**
 * @brief Clears the internal states of the 3P3Z compensator.
 * @param[out] c Pointer to the 3P3Z compensator instance.
 */
GMP_STATIC_INLINE void ctl_clear_3p3z(ctrl_3p3z_t* c)
{
    c->output = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        c->resp[i] = 0.0f;
        c->exct[i] = 0.0f;
    }
}

/**
 * @brief Initializes a 3P3Z compensator with three real poles and three real zeros.
 * @param[out] c Pointer to the 3P3Z compensator instance.
 * @param[in] gain The desired overall DC gain.
 * @param[in] f_z1, f_z2, f_z3 Frequencies of the three real zeros (Hz).
 * @param[in] f_p1, f_p2, f_p3 Frequencies of the three real poles (Hz).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_3p3z_real(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2, parameter_gt f_z3,
                        parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3, parameter_gt fs);

/**
 * @brief Initializes a 3P3Z compensator with one complex-conjugate zero pair and one real zero.
 * @param[out] c Pointer to the 3P3Z compensator instance.
 * @param[in] gain The desired overall DC gain.
 * @param[in] f_czr, f_czi Real and imaginary parts of the complex zero (Hz).
 * @param[in] f_z3 Frequency of the real zero (Hz).
 * @param[in] f_p1, f_p2, f_p3 Frequencies of the three real poles (Hz).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_3p3z_complex_zeros(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                 parameter_gt f_z3, parameter_gt f_p1, parameter_gt f_p2, parameter_gt f_p3,
                                 parameter_gt fs);

/**
 * @brief Initializes a 3P3Z compensator with one complex-conjugate pole pair and one real pole.
 * @param[out] c Pointer to the 3P3Z compensator instance.
 * @param[in] gain The desired overall DC gain.
 * @param[in] f_z1, f_z2, f_z3 Frequencies of the three real zeros (Hz).
 * @param[in] f_cpr, f_cpi Real and imaginary parts of the complex pole (Hz).
 * @param[in] f_p3 Frequency of the real pole (Hz).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_3p3z_complex_poles(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_z1, parameter_gt f_z2,
                                 parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                 parameter_gt fs);

/**
 * @brief Initializes a 3P3Z compensator with one complex-conjugate pole pair and one complex-conjugate zero pair.
 * @param[out] c Pointer to the 3P3Z compensator instance.
 * @param[in] gain The desired overall DC gain.
 * @param[in] f_czr, f_czi Real and imaginary parts of the complex zero (Hz).
 * @param[in] f_z3 Frequency of the real zero (Hz).
 * @param[in] f_cpr, f_cpi Real and imaginary parts of the complex pole (Hz).
 * @param[in] f_p3 Frequency of the real pole (Hz).
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_3p3z_complex_pair(ctrl_3p3z_t* c, parameter_gt gain, parameter_gt f_czr, parameter_gt f_czi,
                                parameter_gt f_z3, parameter_gt f_cpr, parameter_gt f_cpi, parameter_gt f_p3,
                                parameter_gt fs);

/**
 * @brief Executes one step of the 3P3Z compensator.
 * @param[in,out] c Pointer to the 3P3Z compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_3p3z(ctrl_3p3z_t* c, ctrl_gt input)
{
    c->output =
        ctl_mul(c->coef_b[0], input) + ctl_mul(c->coef_b[1], c->exct[0]) + ctl_mul(c->coef_b[2], c->exct[1]) +
        ctl_mul(c->coef_b[3], c->exct[2]) -
        (ctl_mul(c->coef_a[0], c->resp[0]) + ctl_mul(c->coef_a[1], c->resp[1]) + ctl_mul(c->coef_a[2], c->resp[2]));

    c->exct[2] = c->exct[1];
    c->exct[1] = c->exct[0];
    c->exct[0] = input;
    c->resp[2] = c->resp[1];
    c->resp[1] = c->resp[0];
    c->resp[0] = c->output;

    return c->output;
}

/**
 * @brief Calculates the phase lag of the 3P3Z compensator at a specific frequency.
 * @param[in] c Pointer to the 3P3Z compensator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the phase lag (Hz).
 * @return parameter_gt The phase lag in radians.
 */
parameter_gt ctl_get_3p3z_phase_lag(ctrl_3p3z_t* c, parameter_gt fs, parameter_gt f);

/**
 * @brief Calculates the linear gain of the 3P3Z compensator at a specific frequency.
 * @param[in] c Pointer to the 3P3Z compensator instance.
 * @param[in] fs Sampling frequency (Hz).
 * @param[in] f The frequency at which to calculate the gain (Hz).
 * @return parameter_gt The linear gain (magnitude).
 */
parameter_gt ctl_get_3p3z_gain(ctrl_3p3z_t* c, parameter_gt fs, parameter_gt f);

/**
 * @}
 */ // end of pole_zero_compensators group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _POLE_ZERO_H_
