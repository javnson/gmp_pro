/**
 * @file internal_model_ctrl.h
 * @brief Implements an Internal Model Controller (IMC) for SISO systems.
 * @version 1.0
 * @date 2025-08-07
 *
 */

#ifndef _FILE_INTERNAL_MODEL_CTRL_H_
#define _FILE_INTERNAL_MODEL_CTRL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Internal Model Controller (IMC)                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup IMC_CONTROLLER Internal Model Controller (IMC)
 * @brief A robust model-based controller for SISO systems.
 * @details This module provides a robust controller based on the IMC principle.
 * It is designed for systems that can be approximated by a First-Order Plus
 * Dead-Time (FOPDT) model. The controller uses an internal model of the plant
 * to predict its response. The difference between the actual plant output and
 * the model's output (the disturbance) is used to generate a corrective
 * control action. This structure provides excellent setpoint tracking and
 * disturbance rejection.
 * 
 * Plant Model (FOPDT): 
 * @f[ G_p(s) = \frac{K_p e^{-\theta_p s}}{\tau_p s + 1} @f]
 * IMC Controller: 
 * @f[ Q(s) = \frac{1}{K_p} \frac{\tau_p s + 1}{\lambda s + 1} @f]
 * The overall control law is: 
 * @f[ u(s) = Q(s) (r(s) - d(s)) @f]
 * where d(s) is the estimated disturbance: @f[ d(s) = y_p(s) - y_m(s) @f]
 *
 * @{
 */

//================================================================================
// Type Defines
//================================================================================

#define IMC_MAX_DEAD_TIME_SAMPLES (50) // Maximum allowable dead time in samples

/**
 * @brief Initialization parameters for the IMC module.
 */
typedef struct
{
    // --- Plant Model Parameters (FOPDT) ---
    parameter_gt K_p;     ///< Plant gain.
    parameter_gt tau_p;   ///< Plant time constant (s).
    parameter_gt theta_p; ///< Plant dead time (s).

    // --- Controller Tuning ---
    parameter_gt lambda; ///< Desired closed-loop time constant (filter tuning parameter).

    // --- System Parameters ---
    parameter_gt f_ctrl; ///< Controller execution frequency (Hz).

} ctl_imc_init_t;

/**
 * @brief Main structure for the IMC controller.
 */
typedef struct
{
    // --- Output ---
    ctrl_gt u_out; ///< The calculated control output.

    // --- Internal Model State ---
    ctrl_gt y_m;                                       ///< The output of the internal plant model.
    ctrl_gt u_delay_buffer[IMC_MAX_DEAD_TIME_SAMPLES]; ///< Buffer for simulating dead time.
    uint16_t dead_time_samples;                        ///< Number of samples for dead time.
    uint16_t delay_buffer_idx;                         ///< Current index in the delay buffer.

    // --- Controller (Q) State ---
    ctrl_gt q_in_1;  ///< Previous input to the Q controller.
    ctrl_gt q_out_1; ///< Previous output from the Q controller.

    // --- Pre-calculated Parameters ---
    ctrl_gt a_p_d;  ///< Discrete-time pole for the plant model.
    ctrl_gt b_p_d;  ///< Discrete-time gain for the plant model.
    ctrl_gt a_q_d;  ///< Discrete-time pole for the Q controller.
    ctrl_gt b0_q_d; ///< Discrete-time numerator coefficient 0 for Q.
    ctrl_gt b1_q_d; ///< Discrete-time numerator coefficient 1 for Q.

} ctl_imc_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the IMC module.
 * @details Calculates the discrete-time equivalents of the plant model and the
 * IMC controller Q based on the continuous-time FOPDT parameters.
 * @param[out] imc  Pointer to the IMC structure.
 * @param[in]  init Pointer to the initialization parameters.
 * @return 0 on success, -1 on error (e.g., dead time too long).
 */
int ctl_init_imc(ctl_imc_controller_t* imc, const ctl_imc_init_t* init);

/**
 * @brief Resets the internal states of the IMC controller.
 * @param[out] imc Pointer to the IMC structure.
 */
GMP_STATIC_INLINE void ctl_clear_imc(ctl_imc_controller_t* imc)
{
    // 强制使用强类型隔离进行清零
    imc->y_m = float2ctrl(0.0f);
    imc->u_out = float2ctrl(0.0f);
    imc->delay_buffer_idx = 0;

    int i;
    for (i = 0; i < IMC_MAX_DEAD_TIME_SAMPLES; i++)
    {
        imc->u_delay_buffer[i] = float2ctrl(0.0f);
    }
}

/**
 * @brief Executes one step of the Internal Model Control algorithm.
 * @param[out] imc    Pointer to the IMC structure.
 * @param[in]  r      The reference command for the system.
 * @param[in]  y_p    The measured output from the plant.
 * @return The calculated control signal `u` to be sent to the plant.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_imc(ctl_imc_controller_t* imc, ctrl_gt r, ctrl_gt y_p)
{
//    // 1. 极速环形缓冲区寻址：消灭 % 取模操作
//    int32_t read_idx_temp = (int32_t)imc->delay_buffer_idx - (int32_t)imc->dead_time_samples;
//    if (read_idx_temp < 0)
//    {
//        read_idx_temp += IMC_MAX_DEAD_TIME_SAMPLES;
//    }
//    uint16_t read_idx = (uint16_t)read_idx_temp;
//    ctrl_gt u_delayed = imc->u_delay_buffer[read_idx];
//
//    // 2. 更新内部被控对象模型 (使用 ctl_mul 修复裸乘法溢出)
//    // y_m(k) = a_p_d * y_m(k-1) + b_p_d * u(k-d-1)
//    imc->y_m = ctl_mul(imc->a_p_d, imc->y_m) + ctl_mul(imc->b_p_d, u_delayed);
//
//    // 3. 计算模型预测误差
//    ctrl_gt model_error = r - (y_p - imc->y_m);
//
//    // 4. 计算 Q 控制器输出 (使用 ctl_mul 修复裸乘法溢出)
//    // u_q(k) = a_q_d*u_q(k-1) + b0_q_d*e(k) + b1_q_d*e(k-1)
//    ctrl_gt u_q = ctl_mul(imc->a_q_d, imc->u_q) + ctl_mul(imc->b0_q_d, model_error) + ctl_mul(imc->b1_q_d, imc->e_q_1);
//
//    // 5. 饱和限制
//    imc->u_out = ctl_sat(u_q, imc->out_max, imc->out_min);
//
//    // 6. 更新状态
//    imc->e_q_1 = model_error;
//    imc->u_q = imc->u_out;
//
//    // 7. 写入环形缓冲区并极速递增 (消灭 %)
//    imc->u_delay_buffer[imc->delay_buffer_idx] = imc->u_out;
//    imc->delay_buffer_idx++;
//    if (imc->delay_buffer_idx >= IMC_MAX_DEAD_TIME_SAMPLES)
//    {
//        imc->delay_buffer_idx = 0;
//    }
//
//    return imc->u_out;
}

/** @} */ // end of IMC_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_INTERNAL_MODEL_CTRL_H_
