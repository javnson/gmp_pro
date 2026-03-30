/**
 * @file pmsm_rs_mras.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Model Reference Adaptive System (MRAS) for Online Stator Resistance (Rs) Estimation.
 * @version 1.0.0
 * @date 2026-03-30
 *
 * @copyright Copyright GMP(c) 2026
 *
 * =========================================================================================
 * @details MATHEMATICAL FORMULATION & POPOV HYPERSTABILITY
 * =========================================================================================
 * This module dynamically estimates the stator resistance (Rs) to compensate for thermal
 * variations during motor operation. The estimated Rs is then converted directly into the 
 * Integral Gain (Ki) for the FOC current controllers.
 *
 * 1. Reference Model (Actual Motor D-Axis):
 * $u_d = R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q$
 *
 * 2. Adjustable Model (Software Observer):
 * Using the estimated resistance $\hat{R}_s$ and measured variables:
 * $L_d \frac{d\hat{i}_d}{dt} = u_d - \hat{R}_s \hat{i}_d + \omega_e L_q i_q$
 *
 * 3. Discrete Implementation (Forward Euler down-sampled by N):
 * Let $T_{mras} = N \cdot T_s$ be the execution period of this module.
 * $\hat{i}_d(k) = \hat{i}_d(k-1) + \frac{T_{mras}}{L_d} [ u_{d,avg} - \hat{R}_s(k-1)\hat{i}_d(k-1) + \omega_{e,avg} L_q i_{q,avg} ]$
 *
 * 4. Adaptive Law via Popov Hyperstability:
 * Defining the state error $\epsilon = \hat{i}_d - i_d$. To guarantee the hyperstability
 * of the equivalent non-linear feedback system, the adaptive law must satisfy the Popov 
 * integral inequality. This results in a PI regulator acting on the directionally corrected error:
 * $\hat{R}_s(k) = \left( K_{p,mras} + \frac{K_{i,mras}}{s} \right) [ (\hat{i}_d - i_d) \cdot i_d ]$
 *
 * 5. Parameter Translation & Smoothing:
 * $K_{i\_target} = \hat{R}_s \cdot Gain$
 * The target Ki is then passed through a Discrete Slope Limiter to ensure bumpless 
 * transfer to the fast-running PI controllers in the main ISR.
 * =========================================================================================
 */

#ifndef _PMSM_RS_MRAS_H_
#define _PMSM_RS_MRAS_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/continuous/continuous_pi.h>

/**
 * @defgroup pmsm_rs_mras PMSM Rs MRAS Observer
 * @brief Online resistance estimator and Ki parameter tuner.
 * @{
 */

/**
 * @brief Data structure for the MRAS Rs Observer.
 */
typedef struct _tag_pmsm_rs_mras_t
{
    // --- Internal Modules ---
    ctl_divider_t divider;          //!< Decimator for running observer at a lower frequency.
    ctl_pi_t pi_rs;                 //!< PI controller for the adaptive law to generate Rs_est.
    ctl_slope_limiter_t ki_limiter; //!< Slew rate limiter to ensure bumpless Ki updates.

    // --- Motor Parameters (PU System) ---
    parameter_gt Ld_pu;         //!< D-axis Inductance (PU).
    parameter_gt Lq_pu;         //!< Q-axis Inductance (PU).
    parameter_gt Tmras_over_Ld; //!< Pre-calculated multiplier: (T_fast * N) / Ld_pu.

    // --- Configurations & Limits ---
    ctrl_gt ki_gain;        //!< Multiplier to convert Rs_pu directly to target Ki.
    ctrl_gt min_id_exc;     //!< Minimum absolute Id required to activate the adaptive law.
    ctrl_gt min_id_exc_sum; //!< [ÐÂÔö] Pre-calculated threshold for fast condition check (min_id_exc * N).
    ctrl_gt rs_nominal_pu;  //!< The baseline nominal resistance from offline ID.

    // --- State Variables ---
    ctrl_gt id_est;    //!< Adjustable model output: Estimated D-axis current (\hat{i}_d).
    ctrl_gt rs_est;    //!< Active Estimated Resistance (\hat{R}_s).
    ctrl_gt ki_target; //!< Target Ki computed by MRAS.
    ctrl_gt ki_out;    //!< The rate-limited smooth Ki output for the FOC core.

    // --- Down-sampling Accumulators ---
    ctrl_gt sum_ud;
    ctrl_gt sum_id;
    ctrl_gt sum_iq;
    ctrl_gt sum_we;
    ctrl_gt inv_N; //!< Pre-calculated 1.0f / divider.target for fast averaging.

    // --- flag of controller ---
    fast_gt flag_enable_mras; //!< Enable MRAS controller

} ctl_pmsm_rs_mras_t;

GMP_STATIC_INLINE void ctl_clear_pmsm_rs_mras(ctl_pmsm_rs_mras_t* mras)
{

    obj->id_est = float2ctrl(0.0f);
    obj->sum_ud = float2ctrl(0.0f);
    obj->sum_id = float2ctrl(0.0f);
    obj->sum_iq = float2ctrl(0.0f);
    obj->sum_we = float2ctrl(0.0f);
}

/**
 * @brief User-friendly initialization for MRAS Rs Observer using physical parameters.
 * @param[out] obj         Pointer to the MRAS instance.
 * @param[in]  fs_fast     The fast execution frequency (Hz) of the main ISR (e.g., 20000Hz).
 * @param[in]  fs_mras     The target execution frequency (Hz) of the MRAS logic (e.g., 100Hz).
 * @param[in]  Ld_phy      Nominal D-axis Inductance in Henry.
 * @param[in]  Lq_phy      Nominal Q-axis Inductance in Henry.
 * @param[in]  Rs_phy      Nominal Stator Resistance in Ohms.
 * @param[in]  ki_default  The default system-designed Ki value.
 * @param[in]  V_base      System Base Voltage (V).
 * @param[in]  I_base      System Base Current (A).
 * @param[in]  W_base      System Base Electrical Angular Velocity (rad/s).
 */
GMP_STATIC_INLINE void ctl_init_pmsm_rs_mras_physical(ctl_pmsm_rs_mras_t* obj, parameter_gt fs_fast,
                                                      parameter_gt fs_mras, parameter_gt Ld_phy, parameter_gt Lq_phy,
                                                      parameter_gt Rs_phy, parameter_gt ki_default, parameter_gt V_base,
                                                      parameter_gt I_base, parameter_gt W_base)
{
    // 1. Calculate PU System Bases
    parameter_gt Z_base = V_base / I_base;
    parameter_gt L_base = Z_base / W_base;

    // 2. Convert Physical to PU
    parameter_gt Ld_pu = Ld_phy / L_base;
    parameter_gt Lq_pu = Lq_phy / L_base;
    parameter_gt Rs_pu = Rs_phy / Z_base;

    // 3. Calculate the MRAS to Ki multiplier (ki_target = Rs_pu * ki_gain)
    parameter_gt ki_gain = ki_default / Rs_pu;

    // 4. Calculate decimation ratio
    uint32_t divider_N = (uint32_t)(fs_fast / fs_mras);
    if (divider_N < 1)
        divider_N = 1;

    // 5. Default Minimum Id Excitation (Hardcoded to 5% of Base Current as typical default)
    parameter_gt min_id_exc = 0.05f;

    // 6. Call the base PU initialization function
    // (Ensure you update the base init function to set pi limits to 0.5f and 2.0f instead of 0.5f and 1.5f)
    ctl_init_pmsm_rs_mras(obj, fs_fast, divider_N, Ld_pu, Lq_pu, Rs_pu, ki_gain, min_id_exc);
}

/**
 * @brief Initializes the MRAS Rs Observer.
 * @param[out] obj         Pointer to the MRAS instance.
 * @param[in]  fs_fast     The fast execution frequency (Hz) of the main ISR (e.g., 20000Hz).
 * @param[in]  divider_N   The decimation ratio (e.g., 200 for 100Hz MRAS execution).
 * @param[in]  Ld_pu       Nominal D-axis Inductance (PU).
 * @param[in]  Lq_pu       Nominal Q-axis Inductance (PU).
 * @param[in]  Rs_nom_pu   Offline identified Nominal Resistance (PU).
 * @param[in]  ki_gain     The conversion factor: target_Ki = Rs_pu * ki_gain.
 * @param[in]  min_id_exc  Minimum Id excitation required to update Rs (e.g., 0.05pu).
 */
void ctl_init_pmsm_rs_mras(ctl_pmsm_rs_mras_t* obj, parameter_gt fs_fast, uint32_t divider_N, parameter_gt Ld_pu,
                           parameter_gt Lq_pu, parameter_gt Rs_nom_pu, parameter_gt ki_gain, parameter_gt min_id_exc)
{
    // 1. Divider Setup
    ctl_init_divider(&obj->divider, divider_N);
    obj->inv_N = float2ctrl(1.0f / (float)divider_N);

    // 2. Physical Parameters
    obj->Ld_pu = float2ctrl(Ld_pu);
    obj->Lq_pu = float2ctrl(Lq_pu);
    obj->Tmras_over_Ld = float2ctrl(((float)divider_N / fs_fast) / Ld_pu);
    obj->rs_nominal_pu = float2ctrl(Rs_nom_pu);
    obj->min_id_exc = float2ctrl(min_id_exc);
    obj->ki_gain = float2ctrl(ki_gain);

    // 3. PI Controller for Adaptive Law (Kp is tiny, Ki is dominant)
    // The bandwidth of this PI should be extremely low (e.g., 0.1Hz ~ 1Hz)
    parameter_gt fs_mras = fs_fast / (float)divider_N;
    ctl_init_pi(&obj->pi_rs, 0.001f, 0.05f, fs_mras);

    // Safety boundaries: Clamp Rs estimation between 50% and 150% of nominal
    ctl_set_pi_limit(&obj->pi_rs, float2ctrl(Rs_nom_pu * 2.0f), float2ctrl(Rs_nom_pu * 2.0f));
    ctl_set_pi_int_limit(&obj->pi_rs, float2ctrl(Rs_nom_pu * 2.0f), float2ctrl(Rs_nom_pu * 2.0f));

    // Seed the integrator with the known nominal offline Rs
    ctl_set_pi_integrator(&obj->pi_rs, obj->rs_nominal_pu);

    // 4. Slope Limiter Setup (For smooth Ki transitions)
    // Allows e.g., 0.00001 change per FAST ISR cycle
    ctl_init_slope_limiter(&obj->ki_limiter, 0.00001f, -0.00001f, fs_fast);

    obj->rs_est = obj->rs_nominal_pu;
    obj->ki_target = ctl_mul(obj->rs_est, obj->ki_gain);
    ctl_set_slope_limiter_current(&obj->ki_limiter, obj->ki_target);
    obj->ki_out = obj->ki_target;

    // 5. Clear States
    ctl_clear_pmsm_rs_mras(obj);
}

/**
 * @brief High-frequency execution step for the MRAS Rs Observer.
 * @details Accumulates fast data, executes the MRAS model upon divider trigger, 
 * and smooths the output Ki target every single fast cycle.
 * @param[in,out] obj Pointer to the MRAS instance.
 * @param[in]     ud  Current D-axis voltage reference (PU).
 * @param[in]     id  Current D-axis measured current (PU).
 * @param[in]     iq  Current Q-axis measured current (PU).
 * @param[in]     we  Current electrical angular velocity (PU).
 * @return ctrl_gt    The safe, slope-limited target Ki value to inject into FOC.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pmsm_rs_mras(ctl_pmsm_rs_mras_t* obj, ctrl_gt ud, ctrl_gt id, ctrl_gt iq, ctrl_gt we)
{
    // 1. Accumulate fast signals to mitigate PWM and switching noise (Must run ALWAYS)
    obj->sum_ud += ud;
    obj->sum_id += id;
    obj->sum_iq += iq;
    obj->sum_we += we;

    // 2. Down-sampled MRAS Execution
    if (ctl_step_divider(&obj->divider))
    {
        // Check observability directly using the sum to save computation
        if (obj->flag_enable_mras && (obj->sum_id > obj->min_id_exc_sum || obj->sum_id < -obj->min_id_exc_sum))
        {
            // Fully observable: Compute averages only when needed
            ctrl_gt ud_avg = ctl_mul(obj->sum_ud, obj->inv_N);
            ctrl_gt id_avg = ctl_mul(obj->sum_id, obj->inv_N);
            ctrl_gt iq_avg = ctl_mul(obj->sum_iq, obj->inv_N);
            ctrl_gt we_avg = ctl_mul(obj->sum_we, obj->inv_N);

            // A. Adjustable Observer Equations
            ctrl_gt emf_term = ctl_mul(we_avg, ctl_mul(obj->Lq_pu, iq_avg));
            ctrl_gt rs_drop = ctl_mul(obj->rs_est, obj->id_est);
            ctrl_gt delta_id_est = ctl_mul(obj->Tmras_over_Ld, ud_avg - rs_drop + emf_term);
            obj->id_est += delta_id_est;

            // B. Popov Adaptive Law
            ctrl_gt err = obj->id_est - id_avg;
            ctrl_gt pi_input = ctl_mul(err, id_avg); // Directional driving signal

            // C. PI Update
            obj->rs_est = ctl_step_pi_par(&obj->pi_rs, pi_input);

            // D. Translation to PI Parameter Target
            obj->ki_target = ctl_mul(obj->rs_est, obj->ki_gain);
        }
        else
        {
            // Non-observable region or disabled:
            // Only compute id_avg to reset the observer state and prevent windup
            ctrl_gt id_avg = ctl_mul(obj->sum_id, obj->inv_N);
            obj->id_est = id_avg;
        }

        // Reset accumulators regardless of execution path
        obj->sum_ud = float2ctrl(0.0f);
        obj->sum_id = float2ctrl(0.0f);
        obj->sum_iq = float2ctrl(0.0f);
        obj->sum_we = float2ctrl(0.0f);
    }

    // 3. Fast Loop Slew-Rate Limiter (Executed at fs_fast)
    // Even if MRAS is disabled or non-observable, this smoothly maintains the current Ki.
    obj->ki_out = ctl_step_slope_limiter(&obj->ki_limiter, obj->ki_target);

    return obj->ki_out;
}

GMP_STATIC_INLINE ctrl_gt ctl_get_pmsm_rs_mras_ki_out(ctl_pmsm_rs_mras_t* obj)
{
    return obj->ki_out;
}

/**
 * @}
 */ // end of pmsm_rs_mras group

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _PMSM_RS_MRAS_H_
