/**
 * @file online_rs_estimator.h
 * @author Javnson & Gemini
 * @brief Implements an online stator resistance (Rs) estimator using an MRAS scheme.
 * @version 1.0
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#ifndef _FILE_ONLINE_RS_ESTIMATOR_H_
#define _FILE_ONLINE_RS_ESTIMATOR_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup ONLINE_RS_ESTIMATOR Online Rs Estimator (MRAS)
 * @brief An online estimator for stator resistance based on the Model Reference Adaptive System principle.
 * @details This module continuously updates the estimated stator resistance (Rs) to track changes
 * due to temperature variations during motor operation. It uses the d-axis voltage equation
 * as the reference model and a PI controller as the adaptation mechanism.
 * @{
 */

//================================================================================
// 1. 主结构体定义 (Main Structure Definition)
//================================================================================

typedef struct
{
    // --- 内部模块 (Internal Modules) ---
    ctl_pid_t adapt_pi; /**< 用于自适应机制的PI控制器 */

    // --- 链接的电机参数 (Linked Motor Parameters) ---
    const ctl_pmsm_dsn_consultant_t* pmsm_params; /**< 指向包含Ld, Lq等参数的结构体 */

    // --- 状态变量 (State Variables) ---
    parameter_gt id_prev; /**< 存储上一时刻的d轴电流，用于计算微分 */
    parameter_gt Ts;      /**< 控制周期 (s) */

    // --- 输出 (Output) ---
    parameter_gt Rs_estimated; /**< 在线辨识出的电阻值 (Ohm) */

} ctl_online_rs_est_t;

//================================================================================
// 2. 核心函数 (Core Functions)
//================================================================================

/**
 * @brief 初始化在线电阻辨识器.
 * @param est Pointer to the online estimator structure.
 * @param pmsm Pointer to the motor design parameters (must contain initial Rs, Ld, Lq).
 * @param isr_freq_hz The frequency of the control ISR in Hz.
 * @param adapt_kp The proportional gain for the adaptation PI controller.
 * @param adapt_ki The integral gain for the adaptation PI controller.
 */
GMP_STATIC_INLINE void ctl_init_online_rs_est(ctl_online_rs_est_t* est, const ctl_pmsm_dsn_consultant_t* pmsm,
                                              parameter_gt isr_freq_hz, parameter_gt adapt_kp, parameter_gt adapt_ki)
{
    est->pmsm_params = pmsm;
    est->Ts = 1.0f / isr_freq_hz;
    est->id_prev = 0.0f;

    // 使用离线辨识的Rs作为初始值
    est->Rs_estimated = pmsm->Rs;

    // 初始化PI控制器
    // 输出限制可以根据电阻的预期变化范围来设定
    parameter_gt rs_max = pmsm->Rs * 2.0f; // 例如，允许电阻变为初始值的2倍
    parameter_gt rs_min = pmsm->Rs * 0.5f; // 例如，允许电阻变为初始值的0.5倍
    ctl_init_pid(&est->adapt_pi, adapt_kp, adapt_ki, 0.0f, rs_max, rs_min, isr_freq_hz);

    // 将PI控制器的初始输出设置为初始电阻值
    ctl_set_pid_integral(&est->adapt_pi, est->Rs_estimated);
}

/**
 * @brief 在线电阻辨识器的步进函数 (应在ISR中与FOC同步调用).
 * @param est Pointer to the online estimator structure.
 * @param ud The commanded d-axis voltage from the current controller.
 * @param id The measured d-axis current.
 * @param iq The measured q-axis current.
 * @param omega_e The measured electrical angular velocity (rad/s).
 * @return The latest estimated stator resistance (Ohm).
 */
GMP_STATIC_INLINE parameter_gt ctl_step_online_rs_est(ctl_online_rs_est_t* est, parameter_gt ud, parameter_gt id,
                                                      parameter_gt iq, parameter_gt omega_e)
{
    // --- 1. 计算电流微分项 (did/dt) ---
    // 使用后向差分近似: d_id/dt ≈ (id[k] - id[k-1]) / Ts
    parameter_gt did_dt = (id - est->id_prev) / est->Ts;

    // --- 2. 获取所需的电机参数 ---
    parameter_gt Ld = est->pmsm_params->Ld;
    parameter_gt Lq = est->pmsm_params->Lq;

    // --- 3. 构建参考模型和可调模型 ---
    // 参考模型输出 (Reference Model Output): ud_ref = ud - Ld*did/dt + ωe*Lq*iq
    parameter_gt ud_ref = ud - Ld * did_dt + omega_e * Lq * iq;

    // 可调模型输出 (Adjustable Model Output): ud_adj = Rs_hat * id
    parameter_gt ud_adj = est->Rs_estimated * id;

    // --- 4. 计算误差 ---
    parameter_gt error = ud_ref - ud_adj;

    // --- 5. 更新自适应机制 (PI控制器) ---
    // 为了保证稳定性，通常使用 id 作为误差的权重
    // 但为简化，在低速或id变化不大的情况下，可直接使用error
    // 一个折中的方法是使用id的符号或幅值进行限幅
    if (fabsf(id) > 1e-3) // 仅在d轴电流不为零时更新，避免不稳定
    {
        est->Rs_estimated = ctl_step_pid_ser(&est->adapt_pi, error);
    }

    // --- 6. 更新状态变量 ---
    est->id_prev = id;

    return est->Rs_estimated;
}

/** @} */ // end of ONLINE_RS_ESTIMATOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ONLINE_RS_ESTIMATOR_H_
