
/**
 * @file pmsm_esmo.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation of the Extended-EMF Sliding Mode Observer (ESMO).
 *
 * @version 3.1
 * @date 2024-10-27
 *
 * @copyright Copyright GMP(c) 2024
 */


#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm_esmo.h>




void ctl_init_pmsm_esmo_consultant(ctl_pmsm_esmo_t* esmo, const ctl_consultant_pmsm_t* motor,
                                   const ctl_consultant_pu_pmsm_t* pu, parameter_gt fs, parameter_gt fc_emf,
                                   parameter_gt ato_bw_hz, parameter_gt fault_time_ms)
{
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * pu->V_base) / (motor->Ld * pu->I_base);
    parameter_gt k2 = (motor->Rs * Ts) / motor->Ld;
    parameter_gt k3 = (motor->Ld - motor->Lq) / motor->Ld;

    esmo->k1 = float2ctrl(k1);
    esmo->k2 = float2ctrl(k2);
    esmo->k3 = float2ctrl(k3);

    esmo->sf_w_to_rad_tick = float2ctrl(pu->W_base * Ts);

    // 2. Sliding Gain & Margin Calculation
    // Ensuring the sliding gain exceeds the maximum back-EMF magnitude.
    parameter_gt e_max_pu = (pu->W_base * motor->flux_linkage) / pu->V_base;
    esmo->k_slide = float2ctrl(e_max_pu * 1.2f);

    // Configurable boundary layer margin (default 5%). Can be dynamically scheduled if needed.
    parameter_gt default_margin = 0.05f;
    esmo->z_margin = float2ctrl(default_margin);
    esmo->sf_z_margin_inv = float2ctrl(1.0f / default_margin);

    // 3. Sub-module Initialization
    ctl_init_filter_iir1_lpf(&esmo->filter_e[0], fs_safe, fc_emf);
    ctl_init_filter_iir1_lpf(&esmo->filter_e[1], fs_safe, fc_emf);

    // Initialize ATO with default wide saturation margins (+/- 1.5 PU) to handle deep field weakening.
    ctl_init_ato_pll(&esmo->ato_pll, ato_bw_hz, 1.0f, pu->W_base, fs_safe, 1.5f, -1.5f);

    // 4. Phase Compensation Constants
    parameter_gt wc = CTL_PARAM_CONST_2PI * fc_emf;
    esmo->sf_wc_inv = float2ctrl(pu->W_base / wc);

    // 5. Protection Mechanisms Setup
    esmo->current_err_limit = float2ctrl(0.3f);
    esmo->diverge_limit = (uint32_t)(fault_time_ms * fs_safe / 1000.0f);
    if (esmo->diverge_limit < 1)
        esmo->diverge_limit = 1;

    // 6. Finalize Initialization
    ctl_clear_pmsm_esmo(esmo);
    ctl_disable_pmsm_esmo(esmo);
}

/**
 * @brief Core initialization function using the bare physical parameters.
 */
void ctl_init_pmsm_esmo(ctl_pmsm_esmo_t* esmo, const ctl_pmsm_esmo_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    // 1. Plant Constants Definition
    parameter_gt k1 = (Ts * init->V_base) / (init->Ld * init->I_base);
    parameter_gt k2 = (init->Rs * Ts) / init->Ld;
    parameter_gt k3 = (init->Ld - init->Lq) / init->Ld;

    esmo->k1 = float2ctrl(k1);
    esmo->k2 = float2ctrl(k2);
    esmo->k3 = float2ctrl(k3);

    esmo->sf_w_to_rad_tick = float2ctrl(init->W_base * Ts);

    // 2. Sliding Gain & Margin Calculation
    parameter_gt e_max_pu = (init->W_base * init->flux_linkage) / init->V_base;
    esmo->k_slide = float2ctrl(e_max_pu * 1.2f);

    parameter_gt margin = (init->z_margin_pu > 1e-4f) ? init->z_margin_pu : 0.05f;
    esmo->z_margin = float2ctrl(margin);
    esmo->sf_z_margin_inv = float2ctrl(1.0f / margin);

    // 3. Sub-module Initialization
    ctl_init_filter_iir1_lpf(&esmo->filter_e[0], fs_safe, init->fc_emf);
    ctl_init_filter_iir1_lpf(&esmo->filter_e[1], fs_safe, init->fc_emf);

    ctl_init_ato_pll(&esmo->ato_pll, init->ato_bw_hz, 1.0f, init->W_base, fs_safe, 1.5f, -1.5f);

    // 4. Phase Compensation Constants
    parameter_gt wc = CTL_PARAM_CONST_2PI * init->fc_emf;
    esmo->sf_wc_inv = float2ctrl(init->W_base / wc);

    // 5. Protection Mechanisms Setup
    parameter_gt err_lim = (init->current_err_limit_pu > 1e-3f) ? init->current_err_limit_pu : 0.3f;
    esmo->current_err_limit = float2ctrl(err_lim);

    esmo->diverge_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (esmo->diverge_limit < 1)
        esmo->diverge_limit = 1;

    // 6. Finalize Initialization
    ctl_clear_pmsm_esmo(esmo);
    ctl_disable_pmsm_esmo(esmo);
}

/**
 * @brief Auto-tunes and populates the ESMO init structure using motor base parameters.
 * @details Translates physical motor parameters into the ESMO initialization format
 * and automatically calculates the optimal bandwidths and cutoff frequencies
 * for the back-EMF filter and the Angle Tracking Observer (ATO).
 * * @param[out] esmo_init Pointer to the ESMO init structure to be populated.
 * @param[in]  cur_init  Pointer to the generic motor and current loop base configuration.
 * @param[in]  flux_linkage Permanent magnet flux linkage in Webers (Wb).
 */
void ctl_autotune_esmo_init_from_mtr(ctl_pmsm_esmo_init_t* esmo_init,
                                     const mtr_current_init_t* cur_init,
                                     parameter_gt flux_linkage)
{
    // 防呆保护：确保传入的指针有效且基础频率合法
    if (esmo_init == 0 || cur_init == 0 || cur_init->fs < 1e-6f || cur_init->freq_base < 1e-6f)
    {
        return;
    }

    // -------------------------------------------------------------------------
    // 1. Direct Physical Parameter Mapping (物理参数直接映射)
    // -------------------------------------------------------------------------
    esmo_init->Rs = cur_init->mtr_Rs;
    esmo_init->Ld = cur_init->mtr_Ld;
    esmo_init->Lq = cur_init->mtr_Lq;
    esmo_init->flux_linkage = flux_linkage;
    esmo_init->fs = cur_init->fs;

    // -------------------------------------------------------------------------
    // 2. Per-Unit Base Values Conversion (标幺化基准值换算)
    // -------------------------------------------------------------------------
    esmo_init->V_base = cur_init->v_base;
    esmo_init->I_base = cur_init->i_base;

    // W_base = 2 * pi * f_base (将标称电频率 Hz 转换为 角速度 rad/s)
    esmo_init->W_base = CTL_PARAM_CONST_2PI * cur_init->freq_base;

    // -------------------------------------------------------------------------
    // 3. Observer Execution & Tuning Heuristics (观测器带宽与滤波经验公式整定)
    // -------------------------------------------------------------------------

    // Back-EMF Low-Pass Filter Cutoff Frequency (fc_emf)
    // 规则：滑模的开关抖振发生在 fs 级别。为了有效滤除高频抖振，同时又不会对
    // 基波反电动势造成太大的衰减和相位滞后，通常将截止频率设为 fs 的 1/10 到 1/20。
    esmo_init->fc_emf = cur_init->fs / 10.0f;

    // ATO/PLL Tracking Loop Bandwidth (ato_bw_hz)
    // 规则：ATO 锁相环的带宽决定了追踪转子动态响应的速度。
    // 它必须远低于 fc_emf 以抑制噪声，但又要足够快以跟踪电机加速。
    // 经典设定为电机标称电频率的 0.5 到 1.0 倍 (例如 50Hz 电机对应 25Hz 带宽)。
    esmo_init->ato_bw_hz = cur_init->freq_base * 0.5f;

    // -------------------------------------------------------------------------
    // 4. Protection Margins & Limits (保护与边界裕度)
    // -------------------------------------------------------------------------

    // Divergence confirmation time (故障确证防抖时间)
    // 允许 ESMO 在大动态突变时有短暂的失锁，一般设定为 50ms ~ 100ms
    esmo_init->fault_time_ms = 50.0f;

    // 最大允许电流追踪误差 (PU)，超过此值即认为观测器发散
    esmo_init->current_err_limit_pu = 0.3f; // 30% of base current

    // 滑模控制器的边界层厚度 (Boundary layer margin)
    // 用于 Quasi-SMC 防抖振，通常设置在 5% 左右
    esmo_init->z_margin_pu = 0.05f;
}

