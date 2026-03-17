/**
 * @file pmsm_dbptc.h
 * @brief Implements an O(1) Deadbeat Predictive Torque Controller (DB-PTC) for PMSM.
 *
 * @version 1.0
 * @date 2024-10-26
 *
 */

#ifndef _FILE_PMSM_DBPTC_H_
#define _FILE_PMSM_DBPTC_H_

#include <ctl/component/motor_control/interface/motor_universal_interface.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* PMSM Deadbeat Predictive Torque Controller (DB-PTC)                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_DBPTC PMSM Deadbeat Predictive Controller
 * @brief Advanced O(1) predictive current controller for Permanent Magnet Motors.
 * @details Replaces the traditional cascaded PI controllers. Utilizes a 2-step 
 * delay-compensated discrete physical model to calculate the exact voltage vector 
 * required to reach the target current in one step. All math is rigorously 
 * pre-calculated into Per-Unit (PU) constants to ensure fixed-point safety.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the PMSM DB-PTC.
 */
typedef struct _tag_pmsm_dbptc_init
{
    parameter_gt fs;            //!< Controller execution frequency (Hz).
    parameter_gt v_bus;         //!< Nominal DC Bus voltage (V).
    parameter_gt v_phase_limit; //!< Phase voltage limitation (Vrms).
    parameter_gt v_base;        //!< Base voltage for PU conversion (V).
    parameter_gt i_base;        //!< Base current for PU conversion (A).
    parameter_gt spd_base;      //!< Nominal motor speed base (rpm).
    parameter_gt pole_pairs;    //!< Number of pole pairs.

    // --- PMSM Physical Parameters ---
    parameter_gt mtr_Rs;   //!< Stator resistance (Ohm).
    parameter_gt mtr_Ld;   //!< D-axis inductance (H).
    parameter_gt mtr_Lq;   //!< Q-axis inductance (H).
    parameter_gt mtr_Flux; //!< Permanent magnet flux linkage (Wb).

    // --- Safety Limits ---
    parameter_gt i_max_pu; //!< Absolute maximum allowed current vector magnitude (PU).

} pmsm_dbptc_init_t;

/**
 * @brief Main structure for the PMSM DB-PTC controller.
 */
typedef struct _tag_pmsm_dbptc_ctrl
{
    // --- Interfaces ---
    ctl_vector2_t* idq_meas; //!< Measured d-q axis current (PU).
    rotation_ift* pos_if;    //!< Rotor position interface.
    velocity_ift *spd_if;    //!< Rotor velocity interface.

    // --- Setpoints ---
    ctl_vector2_t idq_ref; //!< d-q current reference target (PU).

    // --- Pre-calculated Predictive Model Constants (PU Space) ---
    // Model: i(k+1) = A*i(k) + B*u(k) + E(w)
    ctrl_gt A11; //!< 1 - (Rs * Ts) / Ld
    ctrl_gt A22; //!< 1 - (Rs * Ts) / Lq

    ctrl_gt Kw_d; //!< Cross-coupling D: (Ts * W_base * Lq) / Ld
    ctrl_gt Kw_q; //!< Cross-coupling Q: (Ts * W_base * Ld) / Lq

    ctrl_gt B11; //!< Gain D: (Ts * V_base) / (Ld * I_base)
    ctrl_gt B22; //!< Gain Q: (Ts * V_base) / (Lq * I_base)

    ctrl_gt inv_B11; //!< Inverse Gain D (Avoids division in ISR)
    ctrl_gt inv_B22; //!< Inverse Gain Q (Avoids division in ISR)

    ctrl_gt K_emf; //!< Back-EMF constant: (Ts * W_base * Flux) / (Lq * I_base)

    // --- Protection Limits ---
    ctrl_gt v_max_pu;    //!< Voltage saturation radius (PU).
    ctrl_gt i_max_pu_sq; //!< Current saturation radius squared (PU^2).

    // --- Internal States (Crucial for 2-Step Prediction) ---
    ctrl_gt ud_prev; //!< Saturated D-axis voltage applied in the CURRENT cycle u(k)
    ctrl_gt uq_prev; //!< Saturated Q-axis voltage applied in the CURRENT cycle u(k)

    // --- Output Vectors ---
    ctl_vector2_t vdq_out; //!< Final voltage command for PWM.
    fast_gt flag_enable;   //!< Controller enable flag.

} pmsm_dbptc_ctrl_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_pmsm_dbptc(pmsm_dbptc_ctrl_t* mc, const pmsm_dbptc_init_t* init);

GMP_STATIC_INLINE void ctl_enable_pmsm_dbptc(pmsm_dbptc_ctrl_t* mc)
{
    mc->flag_enable = 1;
}
GMP_STATIC_INLINE void ctl_disable_pmsm_dbptc(pmsm_dbptc_ctrl_t* mc)
{
    mc->flag_enable = 0;
    mc->ud_prev = float2ctrl(0.0f);
    mc->uq_prev = float2ctrl(0.0f);
    ctl_vector2_clear(&mc->vdq_out);
}

/**
 * @brief Safely sets the current reference with strictly enforced circular limits.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_dbptc_ref(pmsm_dbptc_ctrl_t* mc, ctrl_gt id_ref, ctrl_gt iq_ref)
{
    // Point 2: Current Limit Protection (Circular Clamping)
    ctrl_gt i_sq = ctl_mul(id_ref, id_ref) + ctl_mul(iq_ref, iq_ref);
    if (i_sq > mc->i_max_pu_sq)
    {
        // Calculate scaling factor to bring vector back to the circular limit
        // using fast inverse square root approximation or standard sqrt.
        ctrl_gt i_mag = ctl_sqrt(i_sq);
        ctrl_gt scale = ctl_div(ctl_sqrt(mc->i_max_pu_sq), i_mag);
        mc->idq_ref.dat[0] = ctl_mul(id_ref, scale);
        mc->idq_ref.dat[1] = ctl_mul(iq_ref, scale);
    }
    else
    {
        mc->idq_ref.dat[0] = id_ref;
        mc->idq_ref.dat[1] = iq_ref;
    }
}

/**
 * @brief Executes one step of the Deadbeat Predictive Torque Controller.
 * @details Solves the inverse system equations in pure PU space. Includes 2-step 
 * delay compensation and strict voltage vector saturation for robust anti-windup.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_dbptc(pmsm_dbptc_ctrl_t* mc)
{
    if (!mc->flag_enable)
        return;

    // Read physical PU feedback
    ctrl_gt id_meas = mc->idq_meas->dat[0];
    ctrl_gt iq_meas = mc->idq_meas->dat[1];

    // Convert speed from Revs/s back to Electrical Rad/s (PU space)
    ctrl_gt we_pu = mc->spd_if->speed; // Assuming this provides PU electrical frequency

    // ========================================================================
    // Step 1: Delay Compensation (Predict current at k+1)
    // ========================================================================
    // Using the real SATURATED voltage (u_prev) that is currently acting on the motor
    // i_d(k+1) = A11*i_d(k) + We*Kw_d*i_q(k) + B11*u_d(k)
    ctrl_gt id_hat =
        ctl_mul(mc->A11, id_meas) + ctl_mul(mc->Kw_d, ctl_mul(we_pu, iq_meas)) + ctl_mul(mc->B11, mc->ud_prev);

    // i_q(k+1) = A22*i_q(k) - We*Kw_q*i_d(k) + B22*u_q(k) - We*K_emf
    ctrl_gt iq_hat = ctl_mul(mc->A22, iq_meas) - ctl_mul(mc->Kw_q, ctl_mul(we_pu, id_meas)) +
                     ctl_mul(mc->B22, mc->uq_prev) - ctl_mul(mc->K_emf, we_pu);

    // ========================================================================
    // Step 2: Deadbeat Voltage Calculation (Required u(k+1) to reach i_ref at k+2)
    // ========================================================================
    ctrl_gt id_ref = mc->idq_ref.dat[0];
    ctrl_gt iq_ref = mc->idq_ref.dat[1];

    // u_d(k+1) = (1/B11) * [ i_d_ref - A11*i_d(k+1) - We*Kw_d*i_q(k+1) ]
    ctrl_gt ud_req =
        ctl_mul(mc->inv_B11, id_ref - ctl_mul(mc->A11, id_hat) - ctl_mul(mc->Kw_d, ctl_mul(we_pu, iq_hat)));

    // u_q(k+1) = (1/B22) * [ i_q_ref - A22*i_q(k+1) + We*Kw_q*i_d(k+1) + We*K_emf ]
    ctrl_gt uq_req = ctl_mul(mc->inv_B22, iq_ref - ctl_mul(mc->A22, iq_hat) +
                                              ctl_mul(mc->Kw_q, ctl_mul(we_pu, id_hat)) + ctl_mul(mc->K_emf, we_pu));

    // ========================================================================
    // Step 3: Voltage Limit Protection & Anti-Windup Injection
    // ========================================================================
    // Circular voltage saturation: U_d^2 + U_q^2 <= V_max^2
    ctrl_gt v_sq = ctl_mul(ud_req, ud_req) + ctl_mul(uq_req, uq_req);
    ctrl_gt v_max_sq = ctl_mul(mc->v_max_pu, mc->v_max_pu);

    if (v_sq > v_max_sq)
    {
        ctrl_gt v_mag = ctl_sqrt(v_sq);
        ctrl_gt scale = ctl_div(mc->v_max_pu, v_mag);
        ud_req = ctl_mul(ud_req, scale);
        uq_req = ctl_mul(uq_req, scale);
    }

    // Save the strictly saturated voltage for NEXT cycle's prediction (Implicit Anti-Windup)
    mc->ud_prev = ud_req;
    mc->uq_prev = uq_req;

    // Output to modulator
    mc->vdq_out.dat[0] = ud_req;
    mc->vdq_out.dat[1] = uq_req;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_DBPTC_H_
