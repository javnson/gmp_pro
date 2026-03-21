/**
 * @file spm_fw_distributor.h
 * @brief Implements an id=0 current distributor with Angle-based Field Weakening for SPM.
 *
 * @version 1.2
 * @date 2024-10-25
 *
 */

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifndef _FILE_SPM_FW_DISTRIBUTOR_H_
#define _FILE_SPM_FW_DISTRIBUTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup SPM_FW_DISTRIBUTOR SPM id=0 FW Distributor
 * @brief Current distributor for Surface Mounted PMSM (id=0 base control) with Flux Weakening.
 * @details Uses Per-Unit (PU) angles where 1.0 represents 360 electrical degrees.
 * Normal positive torque is aligned at 0.25 PU (+q axis). FW compensates this angle towards 0.5 PU (-d axis).
 * * @par Example Usage:
 * @code
 * // 1. Instantiate structures
 * ctl_spm_fw_distributor_init_t fw_init;
 * ctl_spm_fw_distributor_t fw_ctrl;
 * * // 2. User provides essential hardware parameters
 * fw_init.fs = 10000.0f;       // 10kHz control loop
 * fw_init.v_base = 310.0f;     // Base voltage
 * fw_init.v_nom = 310.0f;      // Nominal achievable phase voltage
 * * // 3. Auto-tune the PI parameters and margins
 * ctl_autotuning_spm_fw_distributor(&fw_init);
 * * // 4. Initialize the controller
 * ctl_init_spm_fw_distributor(&fw_ctrl, &fw_init);
 * ctl_enable_spm_fw_distributor(&fw_ctrl); // Enable FW (Clears PI automatically)
 * * // 5. Real-time loop (Inside Main ISR)
 * void Main_ISR(void) {
 * // Inputs: target Im (from speed loop), and U_d/U_q feedbacks
 * ctl_step_spm_fw_distributor(&fw_ctrl, target_im, ud_fb, uq_fb);
 * * // Pass the vector directly to inner current loop
 * ctl_vector2_t* target_idq = ctl_get_spm_fw_distributor_idq_ref(&fw_ctrl);
 * }
 * * // 6. Safe shutdown (Preserves PI state, zeroes out output)
 * ctl_disable_spm_fw_distributor(&fw_ctrl);
 * @endcode
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the SPM FW Distributor.
 */
typedef struct _tag_spm_fw_distributor_init
{
    // --- User Provided Essential Parameters ---
    parameter_gt fs;     //!< Control loop execution frequency (Hz).
    parameter_gt v_base; //!< Base voltage for per-unit conversion (V).
    parameter_gt v_nom;  //!< Nominal/Max output phase voltage limit of the inverter (V).

    // --- Auto-tunable / Optional Parameters ---
    parameter_gt v_fw_margin;  //!< Voltage margin coefficient for FW triggering.
    parameter_gt kp_fw;        //!< Proportional gain for FW PID controller.
    parameter_gt ki_fw;        //!< Integral gain for FW PID controller (1/Ti for Series PI).
    parameter_gt alpha_max_fw; //!< Maximum allowed FW advance angle in PU.

} ctl_spm_fw_distributor_init_t;

/**
 * @brief Main structure for the SPM FW Distributor real-time execution.
 */
typedef struct _tag_spm_fw_distributor
{
    // --- Outputs ---
    ctl_vector2_t idq_ref;  //!< Output d-q axis current reference vector [id, iq]^T.
    ctrl_gt delta_alpha_pu; //!< The current FW compensation angle in PU.
    ctrl_gt alpha_out_pu;   //!< The final calculated angle in PU (for monitoring).

    // --- Internal States & Limits (All in PU) ---
    ctrl_gt vs_limit_pu;     //!< FW trigger voltage threshold in PU.
    ctrl_gt alpha_max_fw_pu; //!< Maximum FW angle compensation in PU.

    // --- Control Flags ---
    fast_gt flag_enable_fw; //!< Flag to enable (1) or disable (0) Field Weakening.

    // --- Sub-modules ---
    ctl_pid_t fw_pid; //!< PID controller for Field Weakening.

} ctl_spm_fw_distributor_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_spm_fw_distributor(ctl_spm_fw_distributor_init_t* init);
void ctl_init_spm_fw_distributor(ctl_spm_fw_distributor_t* dist, const ctl_spm_fw_distributor_init_t* init);

/**
 * @brief Clears all internal states and completely resets the distributor.
 */
GMP_STATIC_INLINE void ctl_clear_spm_fw_distributor(ctl_spm_fw_distributor_t* dist)
{
    ctl_vector2_clear(&dist->idq_ref);
    dist->delta_alpha_pu = float2ctrl(0.0f);
    dist->alpha_out_pu = float2ctrl(0.0f);
    ctl_clear_pid(&dist->fw_pid);
}

/**
 * @brief Enables the Field Weakening controller safely.
 * @details Clears the PI integrator prior to engagement to guarantee a bump-less start.
 */
GMP_STATIC_INLINE void ctl_enable_spm_fw_distributor(ctl_spm_fw_distributor_t* dist)
{
    ctl_clear_pid(&dist->fw_pid);
    dist->delta_alpha_pu = float2ctrl(0.0f);
    dist->flag_enable_fw = 1;
}

/**
 * @brief Disables the Field Weakening controller safely.
 * @details Sets the FW compensation angle to zero but preserves the PI controller 
 * internal states for telemetry and debugging purposes.
 */
GMP_STATIC_INLINE void ctl_disable_spm_fw_distributor(ctl_spm_fw_distributor_t* dist)
{
    dist->flag_enable_fw = 0;
    dist->delta_alpha_pu = float2ctrl(0.0f); // Cut off the output immediately
    // Notice: ctl_clear_pid is NOT called here to protect the scene.
}

/**
 * @brief Executes one step of the SPM current distribution loop.
 */
GMP_STATIC_INLINE void ctl_step_spm_fw_distributor(ctl_spm_fw_distributor_t* dist, ctrl_gt im, ctrl_gt ud, ctrl_gt uq)
{
    ctrl_gt alpha_base_pu;

    // 1. Determine base angle based on Torque Direction
    ctrl_gt im_mag = (im < float2ctrl(0.0f)) ? -im : im;

    if (im < float2ctrl(0.0f))
    {
        alpha_base_pu = float2ctrl(0.75f); // Negative torque: align to -q axis (-90 deg)
    }
    else
    {
        alpha_base_pu = float2ctrl(0.25f); // Positive torque: align to +q axis (+90 deg)
    }

    // 2. Field Weakening Angle Compensation
    if (dist->flag_enable_fw)
    {
        ctrl_gt u_sq = ctl_mul(ud, ud) + ctl_mul(uq, uq);
        ctrl_gt u_amp = ctl_sqrt(u_sq);

        // Error > 0 when Voltage exceeds Limit
        ctrl_gt fw_err = u_amp - dist->vs_limit_pu;

        // Update the preserved state variable
        dist->delta_alpha_pu = ctl_step_pid_ser(&dist->fw_pid, fw_err);
    }
    else
    {
        // Fail-safe: ensure delta is rigorously 0 when disabled
        dist->delta_alpha_pu = float2ctrl(0.0f);
    }

    // 3. Final Angle Calculation (Quadrant Safe)
    if (im < float2ctrl(0.0f))
    {
        // Subtract delta to push -q towards -d axis
        dist->alpha_out_pu = alpha_base_pu - dist->delta_alpha_pu;
    }
    else
    {
        // Add delta to push +q towards -d axis
        dist->alpha_out_pu = alpha_base_pu + dist->delta_alpha_pu;
    }

    // 4. Calculate id, iq components and store in the vector
    ctrl_gt alpha_rad = ctl_mul(dist->alpha_out_pu, float2ctrl(CTL_PARAM_CONST_2PI));
    dist->idq_ref.dat[phase_d] = ctl_mul(im_mag, ctl_cos(alpha_rad));
    dist->idq_ref.dat[phase_q] = ctl_mul(im_mag, ctl_sin(alpha_rad));
}

/**
 * @brief Returns a pointer to the calculated d-q axis current reference vector.
 */
GMP_STATIC_INLINE ctl_vector2_t* ctl_get_spm_fw_distributor_idq_ref(ctl_spm_fw_distributor_t* dist)
{
    return &dist->idq_ref;
}

/** *@} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SPM_FW_DISTRIBUTOR_H_
