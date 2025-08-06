/**
 * @file lead_lag.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides discrete Lead and Lag compensators for control systems.
 * @version 0.3
 * @date 2025-03-20
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file contains implementations for discrete-time lead and lag
 * compensators. These are first-order IIR filters used to improve the stability
 * (phase margin) and steady-state performance of a control loop. The continuous-time
 * transfer functions are discretized using the Bilinear Transform:
 * @f[
 * s = \frac{2}{T} \frac{1-z^{-1}}{1+z^{-1}}
 * @f]
 */

#ifndef _LEAD_LAG_H_
#define _LEAD_LAG_H_

#include <ctl/math_block/gmp_math.h> // Assuming this is needed for types like ctrl_gt and parameter_gt

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/**
 * @defgroup lead_lag_compensators Lead-Lag Compensators
 * @brief A library of discrete IIR filters for control loop compensation.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Lead Compensator                                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a Lead compensator.
 * @details Implements a lead network, which adds phase margin to a system,
 * improving its transient response. The continuous-time transfer function is:
 * @f[
 * H(s) = \frac{(\tau_D + K_D)s + 1}{\tau_D s + 1}
 * @f]
 * After discretization, it becomes a 1-Pole-1-Zero (1P1Z) filter.
 */
typedef struct _tag_ctrl_lead_t
{
    ctrl_gt b0, b1;   //!< Numerator (zero) coefficients.
    ctrl_gt a1;       //!< Denominator (pole) coefficient.
    ctrl_gt input_1;  //!< Previous input, e(n-1).
    ctrl_gt output_1; //!< Previous output, u(n-1).
} ctrl_lead_t;

/**
 * @brief Initializes a lead compensator from its continuous-time parameters.
 * @param[out] obj Pointer to the lead compensator instance.
 * @param[in] K_D Proportional gain of the derivative term.
 * @param[in] tau_D Time constant of the derivative term's pole.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_lead(ctrl_lead_t* obj, parameter_gt K_D, parameter_gt tau_D, parameter_gt fs);

/**
 * @brief Clears the internal states of the lead compensator.
 * @param[out] obj Pointer to the lead compensator instance.
 */
GMP_STATIC_INLINE void ctl_clear_lead(ctrl_lead_t* obj)
{
    obj->input_1 = 0;
    obj->output_1 = 0;
}

/**
 * @brief Executes one step of the lead compensator.
 * @details Implements the difference equation: u(n) = a1*u(n-1) + b0*e(n) + b1*e(n-1).
 * @param[in,out] obj Pointer to the lead compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_lead(ctrl_lead_t* obj, ctrl_gt input)
{
    ctrl_gt output = ctl_mul(obj->a1, obj->output_1) + ctl_mul(obj->b0, input) + ctl_mul(obj->b1, obj->input_1);

    // Update states for the next iteration
    obj->input_1 = input;
    obj->output_1 = output;

    return output;
}

/*---------------------------------------------------------------------------*/
/* Lag Compensator                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a Lag compensator.
 * @details Implements a lag network, which improves steady-state error by
 * increasing low-frequency gain. The continuous-time transfer function is:
 * @f[
 * H(s) = \frac{\tau_L s + 1}{\tau_P s + 1}, \quad \text{where } \tau_P > \tau_L
 * @f]
 * After discretization, it becomes a 1-Pole-1-Zero (1P1Z) filter.
 */
typedef struct _tag_ctrl_lag_t
{
    ctrl_gt b0, b1;   //!< Numerator (zero) coefficients.
    ctrl_gt a1;       //!< Denominator (pole) coefficient.
    ctrl_gt input_1;  //!< Previous input, e(n-1).
    ctrl_gt output_1; //!< Previous output, u(n-1).
} ctrl_lag_t;

/**
 * @brief Initializes a lag compensator from its continuous-time parameters.
 * @param[out] obj Pointer to the lag compensator instance.
 * @param[in] tau_L Time constant of the lag network's zero.
 * @param[in] tau_P Time constant of the lag network's pole.
 * @param[in] fs Sampling frequency (Hz).
 */
void ctl_init_lag(ctrl_lag_t* obj, parameter_gt tau_L, parameter_gt tau_P, parameter_gt fs);

/**
 * @brief Clears the internal states of the lag compensator.
 * @param[out] obj Pointer to the lag compensator instance.
 */
GMP_STATIC_INLINE void ctl_clear_lag(ctrl_lag_t* obj)
{
    obj->input_1 = 0;
    obj->output_1 = 0;
}

/**
 * @brief Executes one step of the lag compensator.
 * @details Implements the difference equation: u(n) = a1*u(n-1) + b0*e(n) + b1*e(n-1).
 * @param[in,out] obj Pointer to the lag compensator instance.
 * @param[in] input The current input to the compensator, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_lag(ctrl_lag_t* obj, ctrl_gt input)
{
    ctrl_gt output = ctl_mul(obj->a1, obj->output_1) + ctl_mul(obj->b0, input) + ctl_mul(obj->b1, obj->input_1);

    // Update states for the next iteration
    obj->input_1 = input;
    obj->output_1 = output;

    return output;
}

/**
 * @}
 */ // end of lead_lag_compensators group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _LEAD_LAG_H_
