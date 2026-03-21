/**
 * @file backstepping_ctrl.h
 * @brief Implements a Backstepping controller for Single-Input Single-Output (SISO) systems.
 * @note **Typical Application Scenarios (典型应用场景):**
 * The Backstepping controller is highly effective in systems where linear controllers (like PI) 
 * struggle with parameter variations, nonlinear dynamics, or strict-feedback structures.
 * Typical applications in power electronics and motor drives include:
 * * 1. **High-Performance PMSM Speed/Position Control:** * Replacing the traditional PI speed loop. Backstepping guarantees global asymptotic stability 
 * under severe load torque steps and inertia variations, especially when combined with a 
 * disturbance observer (such as an Extended State Observer, ESO) providing `disturbance_est`.
 * * 2. **DC-DC Converter Voltage Regulation (Buck/Boost/Buck-Boost):** * Handling the non-minimum phase characteristics and severe nonlinearity of power converters. 
 * It provides much faster transient response to input voltage drops or load shedding 
 * compared to small-signal linearized PI controllers.
 * * 3. **Grid-Tied Inverter Current Control:** * Robustly tracking the reference current `y_ref` even when grid impedance parameters 
 * (represented by `tau_p` and `K_p`) vary, and effectively suppressing grid voltage harmonics 
 * fed as `disturbance_est`.
 * 
 * @version 1.1
 * @date 2025-08-07
 *
 */

#ifndef _FILE_BACKSTEPPING_CTRL_H_
#define _FILE_BACKSTEPPING_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Backstepping Controller for SISO Systems                                  */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup BACKSTEPPING_CONTROLLER Backstepping Controller
 * @brief A nonlinear controller based on systematic Lyapunov design.
 * @details This module provides a nonlinear controller based on the backstepping
 * design methodology. It is designed for systems that can be approximated by a
 * first-order model. The controller systematically constructs a control law
 * using a Lyapunov function to guarantee stability and tracking performance,
 * offering a robust alternative to traditional linear controllers like PI.
 * 
 *  The design is based on a generic first-order plant model: 
 *          @f[ \tau_p \dot{y} = -y + K_p u + d @f]
 *  where u is the control input, y is the output, and d is an external disturbance.
 * 
 *  Step 1: Define the tracking error: @f[ z_1 = y_{ref} - y @f]
 * 
 *  Step 2: Define a Lyapunov function @f[ V = \frac{1}{2}z_1^2 @f]. Its derivative is
 *          @f[ \dot{V} = z_1 \dot{z}_1 = z_1(\dot{y}_{ref} - \dot{y}) @f]
 * 
 *  Step 3: Substitute the system dynamics and design the control law for u
 *          such that @f[ \dot{V} = -k_1 z_1^2 \le 0 @f] to ensure stability. This yields:
 *          @f[ u^* = \frac{1}{K_p} ( \tau_p(\dot{y}_{ref} + k_1 z_1) + y - d ) @f]
 *
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Initialization parameters for the Backstepping controller module.
 * @details These parameters model the plant as a first-order system.
 */
typedef struct
{
    // --- Controller Gains ---
    parameter_gt k1; ///< The main gain for the tracking error dynamics.

    // --- Plant Model Parameters (SI units) ---
    parameter_gt K_p;   ///< Plant gain.
    parameter_gt tau_p; ///< Plant time constant (s).

} ctl_backstepping_init_t;

/**
 * @brief Main structure for the Backstepping controller.
 */
typedef struct
{
    // --- Output ---
    ctrl_gt u_out; ///< The calculated control output.

    // --- Controller Gain ---
    ctrl_gt k1; ///< Tracking error gain.

    // --- Model Parameters ---
    ctrl_gt inv_K_p; ///< Plant gain.
    ctrl_gt tau_p; ///< Plant time constant.

} ctl_backstepping_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the Backstepping controller module.
 * @param[out] bc   Pointer to the Backstepping controller structure.
 * @param[in]  init Pointer to the initialization parameters.
 */
void ctl_init_backstepping(ctl_backstepping_controller_t* bc, const ctl_backstepping_init_t* init);

/**
 * @brief Resets the internal states of the Backstepping controller.
 * @param[out] bc Pointer to the Backstepping controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_backstepping(ctl_backstepping_controller_t* bc)
{
    bc->u_out = float2ctrl(0.0f);
}

/**
 * @brief Executes one step of the Backstepping control algorithm.
 * @details This function calculates the required control input to
 * make the plant's output track the reference signal.
 * @param[out] bc              Pointer to the Backstepping controller structure.
 * @param[in]  y_ref           The reference command for the system.
 * @param[in]  y_ref_dot       The time derivative of the reference command.
 * @param[in]  y_actual        The measured output from the plant.
 * @param[in]  disturbance_est An estimate of an additive output disturbance.
 * @return The calculated control signal `u` to be sent to the plant.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_backstepping(ctl_backstepping_controller_t* bc, ctrl_gt y_ref, ctrl_gt y_ref_dot,
                                                ctrl_gt y_actual, ctrl_gt disturbance_est)
{
    // 1. Calculate the tracking error (z1)
    ctrl_gt error = y_ref - y_actual;

    // 2. Calculate the control law based on the backstepping design
    // u = (tau_p * (y_ref_dot + k1*error) + y_actual - disturbance_est) / K_p

    // 2. Term 1: Dynamics and tracking term: tau_p * (y_ref_dot + k1*error)
    ctrl_gt term1 = ctl_mul(bc->tau_p, (y_ref_dot + ctl_mul(bc->k1, error)));

    // 3. Term 2: State feedback and disturbance rejection: y_actual - disturbance_est
    ctrl_gt term2 = y_actual - disturbance_est;

    // 4. Multiply by inverse plant gain (Extreme fast execution, no division or branches)
    bc->u_out = ctl_mul(term1 + term2, bc->inv_K_p);

    return bc->u_out;
}

/**
 * @}
 */ // end of BACKSTEPPING_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BACKSTEPPING_CTRL_H_
