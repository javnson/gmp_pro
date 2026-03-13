/**
 * @file flux_weakening.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a flux weakening 
 * @version 0.2
 * @date 2026-02-06
 *
 * @copyright Copyright GMP(c) 2024
 */

#ifndef _FILE_FLUX_WEAKENING_H_
#define _FILE_FLUX_WEAKENING_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup FLUX_WEAKENING_CONTROLLER Basic Flux Weakening Controller
 * @brief Flux weakening controller
 * @details 
 * @{
 */

/**
 * @brief Flux Weakening Controller Structure
 */
typedef struct _tag_fw_controller
{
    // --- Config Parameters ---
    ctrl_gt v_ref_margin; //!< Voltage margin coefficient (e.g., 0.90 ~ 0.95).
    ctrl_gt i_smax;       //!< Maximum allowing motor current (A or PU).

    // --- Inputs (Updated each cycle) ---
    ctl_vector2_t vdq_curr; //!< Current d-q axis voltage output from current loop.
    ctrl_gt u_dc;           //!< Current DC bus voltage.

    // --- Internal State ---
    ctl_pid_t volt_ctrl; //!< The Voltage Loop PI controller.

    // --- Outputs ---
    ctrl_gt id_ref_fw;      //!< The output d-axis current reference (Negative value).
    ctrl_gt iq_max_dynamic; //!< The dynamic limit for q-axis current based on circle limitation.

    // --- Status ---
    fast_gt flag_in_fw; //!< Flag indicating if the system is in flux weakening region.

} ctl_fw_ctrl_t;

/**
 * @brief Initializes the Flux Weakening Controller.
 * @param[out] fw Pointer to the FW controller instance.
 * @param[in] kp Voltage loop Kp.
 * @param[in] ki Voltage loop Ki.
 * @param[in] margin Voltage reference margin (e.g., 0.95).
 * @param[in] fs Sampling frequency.
 */
void ctl_init_fw_ctrl(ctl_fw_ctrl_t* fw, ctrl_gt kp, ctrl_gt ki, ctrl_gt margin, parameter_gt fs)
{
    // 1. Initialize PI
    // Note: The output of this PI is id_ref.
    // Ideally, when V_err < 0 (Voltage too high), we want id to go Negative.
    // If we define Error = V_ref - V_s:
    //    V_s > V_ref -> Error < 0.
    //    We need negative output. So Kp should be Positive.
    ctl_init_pid(&fw->volt_ctrl, kp, ki, 0, fs);

    // 2. Set PI Limits
    // The FW controller only generates NEGATIVE id current.
    // So Max is 0, Min is -I_smax (will be updated in step if I_smax changes, but init here).
    ctl_set_pid_limit(&fw->volt_ctrl, 0, -1000.0f); // Temporary min, sets in step
    ctl_set_pid_int_limit(&fw->volt_ctrl, 0, -1000.0f);

    fw->v_ref_margin = margin;
    fw->i_smax = 0; // User needs to set this via setter or input
    fw->id_ref_fw = 0;
    fw->iq_max_dynamic = 0;
    fw->flag_in_fw = 0;

    ctl_vector2_clear(&fw->vdq_curr);
}

/**
 * @brief Executes one step of Flux Weakening Control.
 * @details Should be called BEFORE the Speed Loop and Current Loop.
 * @param[in,out] fw Pointer to the FW controller.
 * @param[in] vdq_out The voltage output from the *previous* current loop cycle.
 * @param[in] udc The current DC bus voltage.
 * @param[in] i_max The maximum current capability (Inverter/Motor limit).
 */
GMP_STATIC_INLINE void ctl_step_fw_ctrl(ctl_fw_ctrl_t* fw, ctl_vector2_t* vdq_out, ctrl_gt udc, ctrl_gt i_max)
{
    // Update inputs
    fw->i_smax = i_max;
    ctl_vector2_copy(&fw->vdq_curr, vdq_out);

    // --------------------------------------------------------
    // 1. Calculate Voltage Magnitude (Vs)
    // --------------------------------------------------------
    // Vs = sqrt(vd^2 + vq^2)
    // Optimization: If your CPU is slow, you can use Alpha/Beta voltages
    // or compare squared values to avoid sqrt, but for Linear PI control, sqrt is better.
    ctrl_gt v_s_sq = ctl_mul(fw->vdq_curr.dat[phase_d], fw->vdq_curr.dat[phase_d]) +
                     ctl_mul(fw->vdq_curr.dat[phase_q], fw->vdq_curr.dat[phase_q]);

    ctrl_gt v_s = ctl_sqrt(v_s_sq);

    // --------------------------------------------------------
    // 2. Calculate Voltage Limit (V_smax)
    // --------------------------------------------------------
    // V_smax = (Udc / sqrt(3)) * Margin
    // Note: If using SVPWM over-modulation, this limit can be higher.
    // Keeping it linear region: Udc * 0.57735 * Margin.
    ctrl_gt v_limit = ctl_mul(udc, ctl_mul(float2ctrl(0.5773502f), fw->v_ref_margin));

    // --------------------------------------------------------
    // 3. Voltage Loop PI
    // --------------------------------------------------------
    // Error = V_limit - V_s
    ctrl_gt v_err = v_limit - v_s;

    // Update PI Limits dynamically based on current I_max
    ctl_set_pid_limit(&fw->volt_ctrl, 0, -fw->i_smax);
    ctl_set_pid_int_limit(&fw->volt_ctrl, 0, -fw->i_smax);

    // Run PI
    // If V_s > V_limit, Error < 0, Output becomes negative (inject negative id).
    // If V_s < V_limit, Error > 0, Output tries to go positive but clamped to 0.
    fw->id_ref_fw = ctl_step_pid_par(&fw->volt_ctrl, v_err);

    // Check status
    fw->flag_in_fw = (fw->id_ref_fw < float2ctrl(-0.01f)) ? 1 : 0;

    // --------------------------------------------------------
    // 4. Calculate Iq Limit (Priority to Id)
    // --------------------------------------------------------
    // Iq_max = sqrt(I_smax^2 - Id_fw^2)
    ctrl_gt i_rem_sq = ctl_mul(fw->i_smax, fw->i_smax) - ctl_mul(fw->id_ref_fw, fw->id_ref_fw);

    if (i_rem_sq > 0)
    {
        fw->iq_max_dynamic = ctl_sqrt(i_rem_sq);
    }
    else
    {
        fw->iq_max_dynamic = 0;
    }
}



/** @} */ // end of POSITION_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_FLUX_WEAKENING_H_
