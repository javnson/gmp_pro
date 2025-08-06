/**
 * @file acm_consultant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a data structure for holding AC induction motor design parameters.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a "consultant" structure that acts as a container for all
 * key electrical and mechanical parameters of an AC induction motor.
 */

#ifndef _FILE_IM_CONSULTANT_H_
#define _FILE_IM_CONSULTANT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Induction Motor Design Consultant                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_IM_CONSULTANT Induction Motor Design Consultant
 * @ingroup MC_DEFINES
 * @brief A structure to hold all design parameters for an induction motor.
 * @{
 */

/**
 * @brief Data structure for holding AC Induction Motor (IM) design parameters.
 *
 * This structure aggregates all the necessary nameplate and design parameters
 * for an induction motor, which are required for simulation and advanced
* control algorithms.
 */
typedef struct _tag_im_dsn_consultant
{
    // Electrical Parameters
    uint16_t pole_pair; /**< @brief Number of motor pole pairs. */
    parameter_gt Rs;    /**< @brief Stator resistance in Ohms (Ω). */
    parameter_gt Ls;    /**< @brief Stator inductance in Henrys (H). */
    parameter_gt Rr;    /**< @brief Rotor resistance in Ohms (Ω). */
    parameter_gt Lr;    /**< @brief Rotor inductance in Henrys (H). */
    parameter_gt Lm;    /**< @brief Mutual inductance in Henrys (H). */

    // Mechanical Parameters
    parameter_gt inertia; /**< @brief Moment of inertia in kg·m². */
    parameter_gt damp;    /**< @brief Damping factor or friction coefficient. */

} im_dsn_consultant_t;

/**
 * @brief Initializes the induction motor design consultant structure with provided parameters.
 *
 * @param[out] im_dsn Pointer to the induction motor consultant structure to be initialized.
 * @param[in] pole_pair Number of motor pole pairs.
 * @param[in] Rs Stator resistance (Ω).
 * @param[in] Ls Stator inductance (H).
 * @param[in] Rr Rotor resistance (Ω).
 * @param[in] Lr Rotor inductance (H).
 * @param[in] Lm Mutual inductance (H).
 * @param[in] inertia Moment of inertia (kg·m²).
 * @param[in] damp Damping factor.
 */
void ctl_setup_im_dsn_consultant(im_dsn_consultant_t* im_dsn, uint16_t pole_pair, parameter_gt Rs, parameter_gt Ls,
                                 parameter_gt Rr, parameter_gt Lr, parameter_gt Lm, parameter_gt inertia,
                                 parameter_gt damp)
{
    im_dsn->pole_pair = pole_pair;
    im_dsn->Rs = Rs;
    im_dsn->Ls = Ls;
    im_dsn->Rr = Rr;
    im_dsn->Lr = Lr;
    im_dsn->Lm = Lm;
    im_dsn->inertia = inertia;
    im_dsn->damp = damp;
}

/** @} */ // end of MC_IM_CONSULTANT group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_CONSULTANT_H_
