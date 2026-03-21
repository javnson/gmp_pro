/**
 * @file ctl_dp_three_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for three-phase digital power controller modules.
 * @version 1.05
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for three-phase digital power components,
 * including the Three-Phase PLL and bridge modulation modules.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Three Phase PLL

#include <ctl/component/digital_power/inv/pll_srf.h>

void ctl_init_sfr_pll_T(srf_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_Ti,
                        parameter_gt pid_Td, parameter_gt f_ctrl)
{
    // Clear all internal states before initialization.
    ctl_clear_pll_3ph(pll);

    // Calculate the frequency scaling factor. This converts the per-unit frequency
    // into a per-unit angle increment for the given sampling time.
    pll->freq_sf = float2ctrl(f_base / f_ctrl);

    // Initialize the parallel-form PI controller for the loop.
    ctl_init_pid_Tmode(&pll->pid_pll, pid_kp, pid_Ti, pid_Td, f_ctrl);
}

/**
 * @brief Initializes the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 *
 * @param[out] pll Pointer to the @ref srf_pll_t structure to be initialized.
 * @param[in] f_base The nominal grid frequency (e.g., 50 or 60 Hz), used as the per-unit base.
 * @param[in] pid_kp Proportional gain for the phase-locking PI controller.
 * @param[in] pid_ki Integral gain for the phase-locking PI controller (in seconds).
 * @param[in] pid_kd Derivative time constant (typically 0 for a PI controller).
 * @param[in] f_ctrl The controller's execution frequency (sampling frequency) in Hz.
 */
void ctl_init_sfr_pll(srf_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_ki,
                      parameter_gt pid_kd, parameter_gt f_ctrl)
{
    // Clear all internal states before initialization.
    ctl_clear_pll_3ph(pll);

    // Calculate the frequency scaling factor. This converts the per-unit frequency
    // into a per-unit angle increment for the given sampling time.
    pll->freq_sf = float2ctrl(f_base / f_ctrl);

    // Initialize the parallel-form PI controller for the loop.
    ctl_init_pid(&pll->pid_pll, pid_kp, pid_ki, pid_kd, f_ctrl);
}

/**
 * @brief 根据带宽和阻尼比自动计算 PI 参数并初始化 SRF-PLL
 * @param[out] pll            PLL 对象指针
 * @param[in]  f_base         电网基准频率 (e.g., 50.0)
 * @param[in]  f_ctrl         控制循环频率/采样频率 (e.g., 10000.0)
 * @param[in]  voltage_mag    输入的电压矢量模长 (sqrt(alpha^2 + beta^2))。如果输入已经标幺化，则填 1.0。
 * @param[in]  bandwidth_hz   期望的 PLL 带宽 (Hz)。推荐值: 10.0 ~ 30.0 Hz
 * @param[in]  damping_factor 阻尼比。推荐值: 0.707
 */
void ctl_init_srf_pll_auto_tune(srf_pll_t* pll, parameter_gt f_base, parameter_gt f_ctrl, parameter_gt voltage_mag,
                                parameter_gt bandwidth_hz, parameter_gt damping_factor)
{
    // 1. 将带宽 Hz 转换为自然角频率 rad/s
    parameter_gt omega_n = CTL_PARAM_CONST_2PI * bandwidth_hz;

    // 2. 计算环路中的固定增益部分 K_loop = 2 * pi * Vm * f_base
    //    推导来源：
    //    - 鉴相器增益 (rad -> Vq): K_pd = 2 * pi * Vm (因为 theta 是 0~1.0)
    //    - VCO 增益 (freq_pu -> d_theta/dt): K_vco = f_base
    parameter_gt loop_gain_constant = CTL_PARAM_CONST_2PI * voltage_mag * f_base;

    // 防除零保护
    if (loop_gain_constant < 0.0001f)
    {
        loop_gain_constant = 0.0001f;
    }

    // 3. 根据二阶系统公式反解 Kp 和 Ki
    //    Kp = (2 * zeta * omega_n) / K_loop
    //    Ki = (omega_n^2) / K_loop

    parameter_gt calculated_kp = (2.0f * damping_factor * omega_n) / loop_gain_constant;
    parameter_gt calculated_ki = (omega_n * omega_n) / loop_gain_constant;

    // 对于 PI 控制器，Kd 通常为 0
    parameter_gt calculated_kd = 0.0f;

    // 4. 调用原有的初始化函数
    ctl_init_sfr_pll(pll, f_base, calculated_kp, calculated_ki, calculated_kd, f_ctrl);
}
