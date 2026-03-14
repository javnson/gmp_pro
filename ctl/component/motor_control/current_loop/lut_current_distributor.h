/**
 * @file lut_current_distributor.h
 * @brief Implements a Paired-LUT based current distributor with Field Weakening (FW).
 *
 * @version 0.3
 * @date 2024-10-25
 *
 */

#ifndef _FILE_LUT_CURRENT_DISTRIBUTOR_H_
#define _FILE_LUT_CURRENT_DISTRIBUTOR_H_


#include <ctl/component/intrinsic/advance/paired_lut1d.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup LUT_CURRENT_DISTRIBUTOR LUT Current Distributor
 * @brief Distributes current magnitude to d-q axis using a paired lookup table {Im, alpha}.
 * @details Calculates the optimal advance angle (alpha) based on current magnitude (Im) 
 * using a Paired 1D Look-Up Table (typically used for MTPA in IPM motors).
 * Includes a built-in voltage-feedback Field Weakening (FW) PID controller.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Initialization parameters for the LUT Current Distributor.
 * @details Uses standard parameter_gt (float) for offline configuration.
 */
typedef struct _tag_lut_distributor_init
{
    parameter_gt fs;     //!< Control loop execution frequency (Hz).
    parameter_gt v_lim;  //!< Absolute voltage limit for FW intervention (V).
    parameter_gt v_base; //!< Base voltage for per-unit conversion (V).

    parameter_gt kp_fw;        //!< Proportional gain for FW PID controller.
    parameter_gt ki_fw;        //!< Integral gain for FW PID controller.
    parameter_gt alpha_lim_fw; //!< Maximum allowed advance angle during FW (rad).

    parameter_gt alpha_neg_torque; //!< Fixed advance angle used when Im < 0 (braking/generator mode) (rad).

    const ctl_lut1d_pair_t* current_alpha_table; //!< Pointer to the {Im, alpha} calibration table.
    uint32_t lut_size;                           //!< The number of {Im, alpha} pairs in the table.

} ctl_lut_distributor_init_t;

/**
 * @brief Main structure for the LUT Current Distributor real-time execution.
 * @details Retains only internal states, configuration, and outputs. Inputs are passed directly to the step function.
 */
typedef struct _tag_lut_distributor
{
    // --- Outputs ---
    ctrl_gt id;        //!< Output d-axis current reference.
    ctrl_gt iq;        //!< Output q-axis current reference.
    ctrl_gt alpha_out; //!< The final calculated advance angle (for monitoring).

    // --- Internal States & Configurations ---
    ctrl_gt vs_limit;         //!< Per-unit voltage limit for FW triggering.
    ctrl_gt alpha_lim_fw;     //!< Maximum FW angle limit.
    ctrl_gt alpha_neg_torque; //!< Fixed angle for negative torque.

    fast_gt flag_enable_fw; //!< Flag to enable (1) or disable (0) Field Weakening.

    // --- Sub-modules ---
    ctl_pid_t fw_pid; //!< PID controller for Field Weakening.

    // --- 核心更新：使用配对查找表实例 ---
    ctl_paired_lut1d_t im_lut; //!< Paired 1D Look-Up Table object for Im -> alpha mapping.

} ctl_lut_distributor_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

/**
 * @brief Initializes the LUT current distributor parameters.
 * @param[out] dist Pointer to the LUT distributor controller structure.
 * @param[in]  init Pointer to the initialization configuration structure.
 */
void ctl_init_lut_distributor(ctl_lut_distributor_t* dist, const ctl_lut_distributor_init_t* init);

/**
 * @brief Clears the internal states of the distributor (PID integrators, outputs, etc.).
 * @param[out] dist Pointer to the LUT distributor controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_lut_distributor(ctl_lut_distributor_t* dist)
{
    dist->id = float2ctrl(0.0f);
    dist->iq = float2ctrl(0.0f);
    dist->alpha_out = float2ctrl(0.0f);

    ctl_clear_pid(&dist->fw_pid);
}

/**
 * @brief Executes one step of the LUT current distribution loop.
 * @param[in,out] dist Pointer to the distributor structure.
 * @param[in]     im   Target current magnitude.
 * @param[in]     ud   Feedback d-axis voltage.
 * @param[in]     uq   Feedback q-axis voltage.
 */
GMP_STATIC_INLINE void ctl_step_lut_distributor(ctl_lut_distributor_t* dist, ctrl_gt im, ctrl_gt ud, ctrl_gt uq)
{
    ctrl_gt alpha_base;

    // 1. Calculate base angle (alpha_base)
    if (im < float2ctrl(0.0f))
    {
        // Negative torque mode (braking)
        alpha_base = dist->alpha_neg_torque;
    }
    else
    {
        // Positive torque mode: utilize Paired LUT for MTPA interpolation
        // --- 核心更新：插值调用变得极其干净 ---
        alpha_base = ctl_step_interpolate_paired_lut1d(&dist->im_lut, im);
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
GMP_STATIC_INLINE ctrl_gt ctl_get_lut_distributor_id_ref(const ctl_lut_distributor_t* dist)
{
    return dist->id;
}

/**
 * @brief Gets the calculated q-axis current reference.
 * @param[in] dist Pointer to the distributor structure.
 * @return ctrl_gt The q-axis current reference (iq).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_lut_distributor_iq_ref(const ctl_lut_distributor_t* dist)
{
    return dist->iq;
}

/**
 * example
 * 

 ctl_lut_distributor_init_t dist_init;
dist_init.fs = 10000.0f;
// ... 配置其他参数 ...

// 核心：直接强转您的自定义结构体数组！内存结构 {ctrl_gt, ctrl_gt} 完全对齐。
dist_init.current_alpha_table = (const ctl_lut1d_pair_t*)CURRENT_DISTRIBUTION_LUT;
dist_init.lut_size = CURRENT_DISTRIBUTION_LUT_SIZE;

// 初始化
ctl_init_lut_distributor(&my_distributor, &dist_init);

 */


/** *@} 
 */ // end of LUT_CURRENT_DISTRIBUTOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_LUT_CURRENT_DISTRIBUTOR_H_
