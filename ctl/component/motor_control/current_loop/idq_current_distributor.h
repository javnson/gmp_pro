/**
 * @file idq_current_distributor.h
 * @brief Implements a unified current distributor with runtime-switchable modes and Angle-based Field Weakening.
 *
 * @version 2.0
 * @date 2024-10-25
 *
 */

#ifndef _FILE_IDQ_CURRENT_DISTRIBUTOR_H_
#define _FILE_IDQ_CURRENT_DISTRIBUTOR_H_


#include <ctl/component/intrinsic/advance/paired_lut1d.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup IDQ_CURRENT_DISTRIBUTOR Unified IDQ Current Distributor
 * @brief Distributes current magnitude to d-q axis with switchable angle sources and Angle-FW.
 * @details This module calculates d-q axis current references based on an input magnitude (Im).
 * The base advance angle can be dynamically switched at runtime between a CONSTANT value 
 * or a LOOKUP TABLE (LUT). Field weakening is achieved strictly by compensating the angle 
 * via a voltage-feedback PID controller.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Enumeration of available base angle calculation modes.
 */
typedef enum
{
    DIST_MODE_CONST_ALPHA = 0, //!< Use a constant predefined advance angle.
    DIST_MODE_LUT_LINEAR = 1   //!< Use the paired 1D Look-Up Table for the advance angle.
} ctl_dist_mode_e;

/**
 * @brief Initialization parameters for the IDQ Current Distributor.
 */
typedef struct _tag_idq_distributor_init
{
    parameter_gt fs;     //!< Control loop execution frequency (Hz).
    parameter_gt v_lim;  //!< Absolute voltage limit for FW intervention (V).
    parameter_gt v_base; //!< Base voltage for per-unit conversion (V).

    // --- Base Angle Parameters ---
    parameter_gt alpha_const;      //!< Fixed advance angle used in CONST_ALPHA mode (rad).
    parameter_gt alpha_neg_torque; //!< Fixed advance angle used when Im < 0 (braking mode) (rad).

    // --- LUT Parameters ---
    const ctl_lut1d_pair_t* lut_table; //!< Pointer to the {Im, alpha} calibration table.
    uint32_t lut_size;                 //!< The number of {Im, alpha} pairs.

    // --- Field Weakening Parameters ---
    parameter_gt kp_fw;        //!< Proportional gain for FW PID controller.
    parameter_gt ki_fw;        //!< Integral gain for FW PID controller.
    parameter_gt alpha_lim_fw; //!< Maximum allowed advance angle during FW (rad).

} ctl_idq_distributor_init_t;

/**
 * @brief Main structure for the IDQ Current Distributor real-time execution.
 */
typedef struct _tag_idq_distributor
{
    // --- Outputs ---
    ctrl_gt id;        //!< Output d-axis current reference.
    ctrl_gt iq;        //!< Output q-axis current reference.
    ctrl_gt alpha_out; //!< The final calculated advance angle (for monitoring).

    // --- Control Flags ---
    ctl_dist_mode_e mode;   //!< Active mode for base angle calculation (CONST or LUT).
    fast_gt flag_enable_fw; //!< Flag to enable (1) or disable (0) Field Weakening angle compensation.

    // --- Internal States & Configurations ---
    ctrl_gt vs_limit;         //!< Per-unit voltage limit for FW triggering.
    ctrl_gt alpha_lim_fw;     //!< Maximum FW angle limit.
    ctrl_gt alpha_const;      //!< Fixed advance angle for CONST mode.
    ctrl_gt alpha_neg_torque; //!< Fixed angle for negative torque.

    // --- Sub-modules ---
    ctl_pid_t fw_pid;          //!< PID controller for Field Weakening.
    ctl_paired_lut1d_t im_lut; //!< Paired 1D Look-Up Table object for Im -> alpha mapping.

} ctl_idq_distributor_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_idq_distributor(ctl_idq_distributor_t* dist, const ctl_idq_distributor_init_t* init);

GMP_STATIC_INLINE void ctl_clear_idq_distributor(ctl_idq_distributor_t* dist)
{
    dist->id = float2ctrl(0.0f);
    dist->iq = float2ctrl(0.0f);
    dist->alpha_out = float2ctrl(0.0f);

    ctl_clear_pid(&dist->fw_pid);
}

/**
 * @brief Sets the active mode for the distributor.
 * @param[in,out] dist Pointer to the distributor structure.
 * @param[in]     mode The target mode (CONST_ALPHA or LUT_LINEAR).
 */
GMP_STATIC_INLINE void ctl_idq_distributor_set_mode(ctl_idq_distributor_t* dist, ctl_dist_mode_e mode)
{
    dist->mode = mode;
}

/**
 * @brief Sets the active mode for the flux weakening.
 * @param[in,out] dist Pointer to the distributor structure.
 * @param[in]     fw_mode enable flux weakening or not.
 */
GMP_STATIC_INLINE void ctl_idq_distributor_set_fw_mode(ctl_idq_distributor_t* dist, fast_gt fw_mode)
{
    if (fw_mode == 1 && dist->flag_enable_fw == 0)
        ctl_clear_pid(&dist->fw_pid);
    
    dist->flag_enable_fw = fw_mode;
}

/**
 * @brief Executes one step of the IDQ current distribution loop.
 * @param[in,out] dist Pointer to the distributor structure.
 * @param[in]     im   Target current magnitude.
 * @param[in]     ud   Feedback d-axis voltage.
 * @param[in]     uq   Feedback q-axis voltage.
 */
GMP_STATIC_INLINE void ctl_step_idq_distributor(ctl_idq_distributor_t* dist, ctrl_gt im, ctrl_gt ud, ctrl_gt uq)
{
    ctrl_gt alpha_base;

    // 1. Calculate Base Angle
    if (im < float2ctrl(0.0f))
    {
        // Negative torque mode (braking/generator)
        alpha_base = dist->alpha_neg_torque;
    }
    else
    {
        // Positive torque mode: select source based on active mode
        if (dist->mode == DIST_MODE_CONST_ALPHA)
        {
            alpha_base = dist->alpha_const;
        }
        else // DIST_MODE_LUT_LINEAR
        {
            alpha_base = ctl_step_interpolate_paired_lut1d(&dist->im_lut, im);
        }
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

    // 3. Output Saturation & Final Angle
    dist->alpha_out = ctl_sat(alpha_base + delta_alpha, dist->alpha_lim_fw, float2ctrl(0.0f));

    // 4. Calculate id, iq components
    dist->id = ctl_mul(im, ctl_cos(dist->alpha_out));
    dist->iq = ctl_mul(im, ctl_sin(dist->alpha_out));
}

/**
 * @brief Gets the calculated d-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_idq_distributor_id_ref(const ctl_idq_distributor_t* dist)
{
    return dist->id;
}

/**
 * @brief Gets the calculated q-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_idq_distributor_iq_ref(const ctl_idq_distributor_t* dist)
{
    return dist->iq;
}

/** *@} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IDQ_CURRENT_DISTRIBUTOR_H_
