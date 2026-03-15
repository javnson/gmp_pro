/**
 * @file consultant_mech.h
 * @brief Implements the Mechanical System physical models (1-Mass and 2-Mass).
 *
 * @version 1.0
 * @date 2024-10-27
 */

#ifndef _FILE_CONSULTANT_MECH_H_
#define _FILE_CONSULTANT_MECH_H_

#include <ctl/math_block/gmp_math.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Consultant: Mechanical System Physical Models                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CONSULTANT_MECH Mechanical Model Consultant
 * @brief Manages the dynamic properties of the mechanical load.
 * @details Provides standardized models for simple rigid bodies (1-Mass) and 
 * complex flexible couplings (2-Mass). Derives critical frequencies for Notch 
 * Filter tuning and total inertia for LADRC/PI speed loop tuning.
 * @{
 */

//================================================================================
// 1-Mass Rigid Body Model (一阶刚体模型)
//================================================================================

/**
 * @brief Structure for a 1-Mass rigid mechanical system.
 */
typedef struct _tag_consultant_mech1
{
    // --- Physical Parameters ---
    parameter_gt J_total;   //!< Total system inertia (Motor + Load) [kg*m^2].
    parameter_gt B_viscous; //!< Viscous friction coefficient [Nm / (rad/s)].

    // --- Derived Intrinsic Properties ---
    parameter_gt tau_m; //!< Mechanical time constant (J / B) [seconds].

} ctl_consultant_mech1_t;

//================================================================================
// 2-Mass Flexible Body Model (双质量块柔性模型)
//================================================================================

/**
 * @brief Structure for a 2-Mass flexible mechanical system.
 */
typedef struct _tag_consultant_mech2
{
    // --- Physical Parameters ---
    parameter_gt J_motor; //!< Motor side inertia [kg*m^2].
    parameter_gt J_load;  //!< Load side inertia [kg*m^2].
    parameter_gt K_stiff; //!< Shaft torsional stiffness (Spring constant) [Nm/rad].
    parameter_gt C_damp;  //!< Shaft internal damping coefficient [Nm / (rad/s)].

    // --- Derived Intrinsic Properties ---
    parameter_gt J_total;     //!< Total equivalent inertia (Jm + Jl) [kg*m^2].
    parameter_gt w_res_rads;  //!< Resonance frequency [rad/s].
    parameter_gt w_ares_rads; //!< Anti-resonance frequency [rad/s].

    parameter_gt f_res_hz;  //!< Resonance frequency [Hz]. Directly used for Notch Filter!
    parameter_gt f_ares_hz; //!< Anti-resonance frequency [Hz].

} ctl_consultant_mech2_t;

//================================================================================
// Function Prototypes & Inline APIs
//================================================================================

/**
 * @brief Initializes the 1-Mass mechanical model.
 * @param[out] mech Pointer to the 1-Mass consultant instance.
 * @param[in]  j_tot Total inertia (kg*m^2).
 * @param[in]  b_vis Viscous friction (Nm/(rad/s)).
 */
void ctl_consultant_mech1_init(ctl_consultant_mech1_t* mech, parameter_gt j_tot, parameter_gt b_vis);

/**
 * @brief Initializes the 2-Mass mechanical model and derives resonant frequencies.
 * @param[out] mech Pointer to the 2-Mass consultant instance.
 * @param[in]  j_m  Motor inertia (kg*m^2).
 * @param[in]  j_l  Load inertia (kg*m^2).
 * @param[in]  k_s  Shaft stiffness (Nm/rad).
 * @param[in]  c_d  Shaft damping (Nm/(rad/s)).
 */
void ctl_consultant_mech2_init(ctl_consultant_mech2_t* mech, parameter_gt j_m, parameter_gt j_l, parameter_gt k_s,
                               parameter_gt c_d);

/**
 * @brief Calculates the acceleration feedforward torque for a 1-Mass system.
 * @details T_ff = J * alpha + B * omega
 * @param[in] mech Pointer to the 1-Mass consultant instance.
 * @param[in] target_vel Target velocity (rad/s).
 * @param[in] target_acc Target acceleration (rad/s^2).
 * @return parameter_gt Feedforward torque (Nm).
 */
GMP_STATIC_INLINE parameter_gt ctl_consultant_mech1_calc_ff_torque(const ctl_consultant_mech1_t* mech,
                                                                   parameter_gt target_vel, parameter_gt target_acc)
{
    return mech->J_total * target_acc + mech->B_viscous * target_vel;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONSULTANT_MECH_H_
