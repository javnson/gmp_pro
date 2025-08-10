/**
 * @file pmsm_mtpv.h
 * @brief Implements a Maximum Torque Per Voltage (MTPV) / Field Weakening controller.
 * @details This module calculates the optimal d-q axis current references to achieve
 * the maximum possible torque when the motor operates above its base speed and
 * is limited by the available bus voltage. It injects a negative d-axis
 * current to counteract the back-EMF, thus "weakening" the field and allowing
 * for higher speed operation. This is commonly known as Field Weakening control.
 *
 * @version 0.1
 * @date 2025-08-06
 *
 * //tex:
 * // The MTPV algorithm operates on the voltage limit circle defined by:
 * // U_{max}^2 = v_d^2 + v_q^2 = (R_s i_d - \omega_e L_q i_q)^2 + (R_s i_q + \omega_e L_d i_d + \omega_e \psi_f)^2
 * // The optimal d-axis current (id) for field weakening is calculated as:
 * // i_d = \frac{\omega_e^2 L_d \psi_f - \sqrt{(\omega_e^2 L_d \psi_f)^2 - ((\omega_e L_d)^2 + R_s^2)((\omega_e \psi_f)^2 - U_{max}^2)}}{(\omega_e L_d)^2 + R_s^2}
 *
 */

#ifndef _FILE_PMSM_MTPV_H_
#define _FILE_PMSM_MTPV_H_

#include <math.h> // For sqrtf

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* MTPV (Field Weakening) Controller                                         */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MTPV_CONTROLLER MTPV / Field Weakening Controller
 * @brief Calculates optimal Id and Iq references for high-speed operation.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the MTPV module.
 */
typedef struct
{
    // --- Motor Parameters (SI units) ---
    parameter_gt Rs;    ///< Stator Resistance (Ohm).
    parameter_gt Ld;    ///< D-axis Inductance (H).
    parameter_gt Lq;    ///< Q-axis Inductance (H).
    parameter_gt psi_f; ///< Permanent magnet flux linkage (Wb).

} ctl_mtpv_init_t;

/**
 * @brief Main structure for the MTPV controller.
 */
typedef struct
{
    // --- Outputs ---
    ctrl_gt id_ref; ///< The calculated field-weakening d-axis current reference.
    ctrl_gt iq_ref; ///< The calculated q-axis current reference under voltage limit.

    // --- Pre-calculated Parameters ---
    ctrl_gt rs;    ///< Stator Resistance.
    ctrl_gt ld;    ///< D-axis Inductance.
    ctrl_gt lq;    ///< Q-axis Inductance.
    ctrl_gt psi_f; ///< Flux Linkage.
    ctrl_gt rs_sq; ///< R_s^2
    ctrl_gt ld_sq; ///< L_d^2
    ctrl_gt lq_sq; ///< L_q^2

} ctl_mtpv_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the MTPV controller with motor parameters.
 * @param[out] mtpv Pointer to the MTPV controller structure.
 * @param[in]  init Pointer to the initialization parameters structure.
 */
void ctl_init_mtpv(ctl_mtpv_controller_t* mtpv, const ctl_mtpv_init_t* init);

/**
 * @brief Executes one step of the MTPV (Field Weakening) calculation.
 * @details This function should be called when the commanded voltage from the
 * current loop exceeds the available DC bus voltage.
 * @param[out] mtpv      Pointer to the MTPV controller structure.
 * @param[in]  iq_cmd    The original q-axis current command from the speed loop.
 * @param[in]  u_max     The maximum available voltage magnitude.
 * @param[in]  omega_e   The current electrical speed (rad/s).
 */
GMP_STATIC_INLINE void ctl_step_mtpv(ctl_mtpv_controller_t* mtpv, ctrl_gt iq_cmd, ctrl_gt u_max, ctrl_gt omega_e)
{
    ctrl_gt we_sq = omega_e * omega_e;
    ctrl_gt u_max_sq = u_max * u_max;

    // --- Calculate terms for the quadratic equation of Id ---
    // The voltage equation can be rearranged into a quadratic form: A*id^2 + B*id + C = 0
    ctrl_gt A = we_sq * mtpv->ld_sq + mtpv->rs_sq;
    ctrl_gt B = 2.0f * we_sq * mtpv->ld * mtpv->psi_f;
    ctrl_gt C = we_sq * mtpv->psi_f * mtpv->psi_f + we_sq * mtpv->lq_sq * iq_cmd * iq_cmd +
                mtpv->rs_sq * iq_cmd * iq_cmd + 2.0f * mtpv->rs * omega_e * mtpv->psi_f * iq_cmd - u_max_sq;

    // --- Solve the quadratic equation for Id ---
    ctrl_gt discriminant = B * B - 4.0f * A * C;

    if (discriminant >= 0.0f)
    {
        // A valid solution exists. We choose the negative Id for field weakening.
        mtpv->id_ref = (-B + sqrtf(discriminant)) / (2.0f * A);
    }
    else
    {
        // No real solution, indicates extreme condition.
        // Clamp Id to a safe negative value based on flux.
        mtpv->id_ref = -mtpv->psi_f / mtpv->ld;
    }

    // The q-axis current is the original command from the speed loop.
    // In a full implementation, Iq might also be adjusted to stay within the current limit circle.
    mtpv->iq_ref = iq_cmd;
}

/**
 * @brief Gets the calculated d-axis current reference for field weakening.
 * @param[in] mtpv Pointer to the MTPV controller structure.
 * @return The optimal d-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpv_id_ref(const ctl_mtpv_controller_t* mtpv)
{
    return mtpv->id_ref;
}

/**
 * @brief Gets the q-axis current reference for field weakening.
 * @param[in] mtpv Pointer to the MTPV controller structure.
 * @return The optimal q-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpv_iq_ref(const ctl_mtpv_controller_t* mtpv)
{
    return mtpv->iq_ref;
}

/** @} */ // end of MTPV_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_MTPV_H_
