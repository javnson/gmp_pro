/**
 * @file sliding_mode_controller.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides a Sliding Mode Controller (SMC).
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @details This file implements a Sliding Mode Controller (SMC), a type of
 * nonlinear controller known for its robustness to parameter uncertainties and
 * external disturbances. The control law is designed to drive the system's
 * state trajectory onto a user-defined sliding surface and maintain it there.
 * This implementation uses a state-dependent gain structure.
 */
#ifndef _SLIDING_MODE_CONTROLLER_H_
#define _SLIDING_MODE_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup sliding_mode_controller Sliding Mode Controller (SMC)
 * @brief A nonlinear robust controller based on a sliding surface.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Sliding Mode Controller                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Sliding Mode Controller.
 * @details The controller takes two state variables as input: x1 (the controlled
 * variable) and x2 (its derivative).
 *
 * The sliding surface is defined as:
 * @f[
 * s = \lambda x_1 + x_2
 * @f]
 *
 * The control law is a form of reaching law:
 * @f[
 * u = u_{eq} + u_{sw} = (\eta_1 x_1 + \eta_2 x_2) + \rho \cdot \text{sgn}(s)
 * @f]
 * where the gains @f$ \eta_1 @f$ and @f$ \eta_2 @f$ are switched based on the
 * signs of the sliding surface and the state variables.
 */
typedef struct _tag_smc_t_
{
    // Parameters
    ctrl_gt eta11;  //!< Gain for x1 when s > 0 and x1 > 0.
    ctrl_gt eta12;  //!< Gain for x1 when s > 0 and x1 < 0 (or vice versa).
    ctrl_gt eta21;  //!< Gain for x2 when s > 0 and x2 > 0.
    ctrl_gt eta22;  //!< Gain for x2 when s > 0 and x2 < 0 (or vice versa).
    ctrl_gt rho;    //!< Switching gain for the sgn(s) term.
    ctrl_gt lambda; //!< Coefficient defining the slope of the sliding surface.

    // State variables
    ctrl_gt output; //!< The current controller output, u.
    ctrl_gt slide;  //!< The current value of the sliding surface, s.
} ctl_smc_t;

/**
 * @brief Initializes the Sliding Mode Controller.
 * @param[out] smc Pointer to the SMC instance.
 * @param[in] eta11 Gain parameter.
 * @param[in] eta12 Gain parameter.
 * @param[in] eta21 Gain parameter.
 * @param[in] eta22 Gain parameter.
 * @param[in] rho Switching gain.
 * @param[in] lambda Sliding surface slope.
 */
void ctl_init_smc(ctl_smc_t* smc, ctrl_gt eta11, ctrl_gt eta12, ctrl_gt eta21, ctrl_gt eta22, ctrl_gt rho,
                  ctrl_gt lambda);

/**
 * @brief Executes one step of the Sliding Mode Controller.
 * @param[in,out] smc Pointer to the SMC instance.
 * @param[in] input The primary state variable, x1.
 * @param[in] input_diff The derivative of the state variable, x2.
 * @return ctrl_gt The calculated control output, u.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_smc(ctl_smc_t* smc, ctrl_gt input, ctrl_gt input_diff)
{
    ctrl_gt etax1;
    ctrl_gt etax2;
    ctrl_gt rhos;

    // Calculate the sliding surface value: s = lambda*x1 + x2
    smc->slide = ctl_mul(smc->lambda, input) + input_diff;

    if (smc->slide > 0)
    {
        // Equivalent control part based on state signs
        etax1 = (input > 0) ? ctl_mul(smc->eta11, input) : ctl_mul(smc->eta12, input);
        etax2 = (input_diff > 0) ? ctl_mul(smc->eta21, input_diff) : ctl_mul(smc->eta22, input_diff);
        // Switching control part
        rhos = smc->rho;
    }
    else
    {
        // Equivalent control part based on state signs (gains are swapped)
        etax1 = (input > 0) ? ctl_mul(smc->eta12, input) : ctl_mul(smc->eta11, input);
        etax2 = (input_diff > 0) ? ctl_mul(smc->eta22, input_diff) : ctl_mul(smc->eta21, input_diff);
        // Switching control part
        rhos = -smc->rho;
    }

    // Total control output
    smc->output = etax1 + etax2 + rhos;

    return smc->output;
}

/**
 * @}
 */ // end of sliding_mode_controller group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _SLIDING_MODE_CONTROLLER_H_
