/**
 * @file virtual_imp.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a virtual RL impedance.
 * @version 1.0
 * @date 2026-01-26
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_BASIC_VIRTUAL_IMP Virtual Impedance module (Header)
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive three-phase inverter, including harmonic compensation, droop control,
 * and multiple operating modes.
 */

#ifndef _FILE_VIRTUAL_IMPEDANCE_
#define _FILE_VIRTUAL_IMPEDANCE_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/intrinsic/discrete/biquad_filter.h>

/**
 * @brief Virtual Impedance Module.
 * @details Implements @f[ Z(s) = R + sL @f].
 * Since pure 's' is noisy, we implement sL as:@f[ L \times (s / (s/wc + 1)) @f]
 */
typedef struct _tag_vir_imp
{
    // Parameters
    ctrl_gt gain_R; //!< Proportional gain (R).

    // Virtual Inductor Implementation
    // We use a biquad filter to implement the transfer function: 
    // @f[ H(s) = L * s / (tau*s + 1) @f]
    ctl_biquad_filter_t diff_filter;

    // Output
    ctrl_gt out; //!< Calculated voltage drop (V_inj).

} vir_imp_t;

/**
 * @brief Initializes the Virtual Impedance module.
 */
void ctl_init_vir_imp(vir_imp_t* imp, const parameter_gt R_vir, const parameter_gt L_vir, parameter_gt fs);

GMP_STATIC_INLINE void ctl_clear_vir_imp(vir_imp_t* imp)
{
    ctl_clear_biquad_filter(&imp->diff_filter);
}

/**
 * @brief Executes one step of the Virtual Impedance simulation.
 * @param imp Pointer to the module.
 * @param input_current Current (usually filtered) to apply impedance to.
 * @return Voltage injection value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_vir_imp(vir_imp_t* imp, ctrl_gt input_current)
{
    // 1. Resistive Part: V = I * R
    ctrl_gt v_r = ctl_mul(input_current, imp->gain_R);

    // 2. Inductive Part: V = L * di/dt
    // The biquad is configured as a band-limited differentiator with gain L.
    ctrl_gt v_l = ctl_step_biquad_filter(&imp->diff_filter, input_current);

    // 3. Total Impedance Drop
    imp->out = v_r + v_l;

    return imp->out;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_VIRTUAL_IMPEDANCE_

/**
 * @}
 */
