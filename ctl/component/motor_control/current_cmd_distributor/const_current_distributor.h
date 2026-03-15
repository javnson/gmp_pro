/**
 * @file const_current_distributor.h
 * @brief Implements a constant-angle current distributor with Field Weakening (FW).
 *
 * @version 0.1
 * @date 2024-10-25
 *
 */

#ifndef _FILE_CONST_CURRENT_DISTRIBUTOR_H_
#define _FILE_CONST_CURRENT_DISTRIBUTOR_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/gmp_math.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup CONST_CURRENT_DISTRIBUTOR Constant Angle Current Distributor
 * @brief Distributes current magnitude to d-q axis using a fixed advance angle, with Field Weakening support.
 * @details This module calculates the d-q axis current references using a predefined constant 
 * advance angle (alpha) for positive torque requests (e.g., alpha = 0 for id=0 control in SPM).
 * It features a built-in voltage-feedback Field Weakening (FW) PID controller to 
 * actively compensate the angle when the terminal voltage exceeds the DC bus limit.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the Constant Angle Current Distributor.
 * @details Uses standard parameter_gt (float) for offline configuration.
 */
typedef struct _tag_const_distributor_init
{
    parameter_gt fs;                 //!< Control loop execution frequency (Hz).
    parameter_gt v_lim;              //!< Absolute voltage limit for FW intervention (V).
    parameter_gt v_base;             //!< Base voltage for per-unit conversion (V).

    parameter_gt kp_fw;              //!< Proportional gain for FW PID controller.
    parameter_gt ki_fw;              //!< Integral gain for FW PID controller.
    parameter_gt alpha_lim_fw;       //!< Maximum allowed advance angle during FW (rad).

    parameter_gt alpha_const;        //!< Fixed advance angle used when Im >= 0 (rad).
    parameter_gt alpha_neg_torque;   //!< Fixed advance angle used when Im < 0 (braking/generator mode) (rad).

} ctl_const_distributor_init_t;

/**
 * @brief Main structure for the Constant Angle Distributor real-time execution.
 * @details Retains only internal states, configuration, and outputs.
 */
typedef struct _tag_const_distributor
{
    // --- Outputs ---
    ctrl_gt id;                      //!< Output d-axis current reference.
    ctrl_gt iq;                      //!< Output q-axis current reference.
    ctrl_gt alpha_out;               //!< The final calculated advance angle (for monitoring).

    // --- Internal States & Configurations ---
    ctrl_gt vs_limit;                //!< Per-unit voltage limit for FW triggering.
    ctrl_gt alpha_lim_fw;            //!< Maximum FW angle limit.
    ctrl_gt alpha_const;             //!< Fixed advance angle for positive torque.
    ctrl_gt alpha_neg_torque;        //!< Fixed advance angle for negative torque.
    
    fast_gt flag_enable_fw;          //!< Flag to enable (1) or disable (0) Field Weakening.

    // --- Sub-modules ---
    ctl_pid_t fw_pid;                //!< PID controller for Field Weakening.

} ctl_const_distributor_t;


//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Initializes the constant angle current distributor parameters.
 * @param[out] dist Pointer to the constant distributor controller structure.
 * @param[in]  init Pointer to the initialization configuration structure.
 */
void ctl_init_const_distributor(ctl_const_distributor_t* dist, const ctl_const_distributor_init_t* init);

/**
 * @brief Clears the internal states of the distributor (PID integrators, outputs, etc.).
 * @param[out] dist Pointer to the constant distributor controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_const_distributor(ctl_const_distributor_t* dist)
{
    dist->id = float2ctrl(0.0f);
    dist->iq = float2ctrl(0.0f);
    dist->alpha_out = float2ctrl(0.0f);

    ctl_clear_pid(&dist->fw_pid);
}

/**
 * @brief Executes one step of the constant angle current distribution loop.
 * @param[in,out] dist Pointer to the distributor structure.
 * @param[in]     im   Target current magnitude.
 * @param[in]     ud   Feedback d-axis voltage (used for FW).
 * @param[in]     uq   Feedback q-axis voltage (used for FW).
 */
GMP_STATIC_INLINE void ctl_step_const_distributor(ctl_const_distributor_t* dist, ctrl_gt im, ctrl_gt ud, ctrl_gt uq)
{
    ctrl_gt alpha_base;

    // 1. Determine base angle based on torque direction
    if (im < float2ctrl(0.0f))
    {
        // Negative torque mode (braking)
        alpha_base = dist->alpha_neg_torque;
    }
    else
    {
        // Positive torque mode: use predefined constant angle (e.g., 0 for id=0 control)
        alpha_base = dist->alpha_const;
    }

    // 2. Field Weakening Angle Compensation
    ctrl_gt delta_alpha = float2ctrl(0.0f);
    if (dist->flag_enable_fw)
    {
        // Calculate voltage amplitude: U_amp = sqrt(ud^2 + uq^2)
        ctrl_gt u_sq = ctl_mul(ud, ud) + ctl_mul(uq, uq);
        ctrl_gt u_amp = ctl_sqrt(u_sq);
        
        // Calculate error and step PID: err = U_amp - V_limit
        ctrl_gt fw_err = u_amp - dist->vs_limit;
        delta_alpha = ctl_step_pid_ser(&dist->fw_pid, fw_err);
    }

    // 3. Output Saturation
    // Total angle = Base angle + FW compensation
    dist->alpha_out = ctl_sat(alpha_base + delta_alpha, dist->alpha_lim_fw, float2ctrl(0.0f));

    // 4. Calculate id, iq components
    dist->id = ctl_mul(im, ctl_cos(dist->alpha_out));
    dist->iq = ctl_mul(im, ctl_sin(dist->alpha_out));
}

/**
 * @brief Gets the calculated d-axis current reference.
 * @param[in] dist Pointer to the distributor structure.
 * @return ctrl_gt The d-axis current reference (id).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_const_distributor_id_ref(const ctl_const_distributor_t* dist)
{
    return dist->id;
}

/**
 * @brief Gets the calculated q-axis current reference.
 * @param[in] dist Pointer to the distributor structure.
 * @return ctrl_gt The q-axis current reference (iq).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_const_distributor_iq_ref(const ctl_const_distributor_t* dist)
{
    return dist->iq;
}

/** *@} 
 */ // end of CONST_CURRENT_DISTRIBUTOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONST_CURRENT_DISTRIBUTOR_H_