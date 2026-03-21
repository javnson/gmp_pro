/**
 * @file lut_fw_distributor.h
 * @brief Implements a LUT-based current distributor with Field Weakening for PMASynRM / IPM.
 *
 * @version 1.0
 * @date 2024-10-25
 *
 */

#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/component/intrinsic/advance/paired_lut1d.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifndef _FILE_LUT_FW_DISTRIBUTOR_H_
#define _FILE_LUT_FW_DISTRIBUTOR_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup LUT_FW_DISTRIBUTOR LUT-based FW Distributor
 * @brief Current distributor using a 1D Lookup Table for base MTPA angle and PI for Flux Weakening.
 * @details Uses Per-Unit (PU) angles. The base angle is dynamically interpolated from a 
 * calibrated {Im, Alpha} lookup table. This is ideal for highly saturated IPMs or PMASynRMs.
 * Negative torque base angles are mirrored automatically across the d-axis (1.0 - alpha_pos).
 * Field weakening compensates this angle towards the -d axis (0.5 PU).
 * * @par Example Usage:
 * @code
 * // 1. Instantiate structures
 * ctl_lut_fw_distributor_init_t fw_init;
 * ctl_lut_fw_distributor_t fw_ctrl;
 * * // 2. User provides essential parameters & Calibration Data
 * fw_init.fs = 10000.0f;
 * fw_init.v_base = 310.0f;
 * fw_init.v_nom = 310.0f;
 * fw_init.lut_table = (const ctl_lut1d_pair_t*)MY_CALIBRATED_LUT; // 内存完全对齐，直接强转
 * fw_init.lut_size = MY_CALIBRATED_LUT_SIZE;
 * * // 3. Auto-tune the PI parameters and margins
 * ctl_autotuning_lut_fw_distributor(&fw_init);
 * * // 4. Initialize and enable the controller
 * ctl_init_lut_fw_distributor(&fw_ctrl, &fw_init);
 * ctl_enable_lut_fw_distributor(&fw_ctrl); 
 * * // 5. Real-time loop (Inside Main ISR)
 * void Main_ISR(void) {
 * ctl_step_lut_fw_distributor(&fw_ctrl, target_im, ud_fb, uq_fb);
 * ctl_vector2_t* target_idq = ctl_get_lut_fw_distributor_idq_ref(&fw_ctrl);
 * }
 * * // 6. Safe shutdown
 * ctl_disable_lut_fw_distributor(&fw_ctrl);
 * @endcode
 * @{
 */

#ifndef phase_d
#define phase_d 0
#define phase_q 1
#endif

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the LUT FW Distributor.
 */
typedef struct _tag_lut_fw_distributor_init
{
    // --- User Provided Essential Parameters ---
    parameter_gt fs;     //!< Control loop execution frequency (Hz).
    parameter_gt v_base; //!< Base voltage for per-unit conversion (V).
    parameter_gt v_nom;  //!< Nominal/Max output phase voltage limit of the inverter (V).

    // --- Calibration Table Parameters ---
    const ctl_lut1d_pair_t* lut_table; //!< Pointer to the {Im, alpha(PU)} calibration table.
    uint32_t lut_size;                 //!< The number of {Im, alpha} pairs.

    // --- Auto-tunable / Optional Parameters ---
    parameter_gt v_fw_margin;     //!< Voltage margin coefficient for FW triggering.
    parameter_gt kp_fw;           //!< Proportional gain for FW PID controller.
    parameter_gt ki_fw;           //!< Integral gain for FW PID controller (1/Ti).
    parameter_gt alpha_max_fw_pu; //!< Maximum FW angle compensation limit in PU.

} ctl_lut_fw_distributor_init_t;

/**
 * @brief Main structure for the LUT FW Distributor real-time execution.
 */
typedef struct _tag_lut_fw_distributor
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
    ctl_pid_t fw_pid;          //!< PID controller for Field Weakening.
    ctl_paired_lut1d_t im_lut; //!< Paired 1D Look-Up Table object for Im -> alpha_pu mapping.

} ctl_lut_fw_distributor_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_autotuning_lut_fw_distributor(ctl_lut_fw_distributor_init_t* init);
void ctl_init_lut_fw_distributor(ctl_lut_fw_distributor_t* dist, const ctl_lut_fw_distributor_init_t* init);

/**
 * @brief Clears all internal states and completely resets the distributor.
 */
GMP_STATIC_INLINE void ctl_clear_lut_fw_distributor(ctl_lut_fw_distributor_t* dist)
{
    ctl_vector2_clear(&dist->idq_ref);
    dist->delta_alpha_pu = float2ctrl(0.0f);
    dist->alpha_out_pu = float2ctrl(0.0f);
    ctl_clear_pid(&dist->fw_pid);
}

/**
 * @brief Enables the Field Weakening controller safely.
 */
GMP_STATIC_INLINE void ctl_enable_lut_fw_distributor(ctl_lut_fw_distributor_t* dist)
{
    ctl_clear_pid(&dist->fw_pid);
    dist->delta_alpha_pu = float2ctrl(0.0f);
    dist->flag_enable_fw = 1;
}

/**
 * @brief Disables the Field Weakening controller safely.
 */
GMP_STATIC_INLINE void ctl_disable_lut_fw_distributor(ctl_lut_fw_distributor_t* dist)
{
    dist->flag_enable_fw = 0;
    dist->delta_alpha_pu = float2ctrl(0.0f); // Cut off the output immediately
}

/**
 * @brief Executes one step of the LUT FW current distribution loop.
 */
GMP_STATIC_INLINE void ctl_step_lut_fw_distributor(ctl_lut_fw_distributor_t* dist, ctrl_gt im, ctrl_gt ud, ctrl_gt uq)
{
    ctrl_gt alpha_pos_base_pu;
    ctrl_gt alpha_base_pu;
    ctrl_gt im_mag = (im < float2ctrl(0.0f)) ? -im : im;

    // 1. Determine base angle via LUT Interpolation
    // Always interpolate using the absolute magnitude to get the 1st/2nd quadrant MTPA angle
    alpha_pos_base_pu = ctl_step_interpolate_paired_lut1d(&dist->im_lut, im_mag);

    // Map to negative torque quadrant if necessary
    if (im < float2ctrl(0.0f))
    {
        // Symmetrical mirroring across the d-axis for generation/braking mode
        alpha_base_pu = float2ctrl(1.0f) - alpha_pos_base_pu;
    }
    else
    {
        alpha_base_pu = alpha_pos_base_pu;
    }

    // 2. Field Weakening Angle Compensation
    if (dist->flag_enable_fw)
    {
        ctrl_gt u_sq = ctl_mul(ud, ud) + ctl_mul(uq, uq);
        ctrl_gt u_amp = ctl_sqrt(u_sq);

        ctrl_gt fw_err = u_amp - dist->vs_limit_pu;
        dist->delta_alpha_pu = ctl_step_pid_ser(&dist->fw_pid, fw_err);
    }
    else
    {
        dist->delta_alpha_pu = float2ctrl(0.0f);
    }

    // 3. Final Angle Calculation (Quadrant Safe)
    // To weaken the field, we always want to push the angle towards the -d axis (0.5 PU)
    if (im < float2ctrl(0.0f))
    {
        dist->alpha_out_pu = alpha_base_pu - dist->delta_alpha_pu;
    }
    else
    {
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
GMP_STATIC_INLINE ctl_vector2_t* ctl_get_lut_fw_distributor_idq_ref(ctl_lut_fw_distributor_t* dist)
{
    return &dist->idq_ref;
}

/** *@} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_LUT_FW_DISTRIBUTOR_H_
