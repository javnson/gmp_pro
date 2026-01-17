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
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/three_phase/pll.h>

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

//////////////////////////////////////////////////////////////////////////
// Three Phase Modulation
//////////////////////////////////////////////////////////////////////////
//#include <ctl/component/digital_power/three_phase/tp_modulation.h>
//
//void ctl_init_three_phase_bridge_modulation(three_phase_bridge_modulation_t* bridge, pwm_gt pwm_full_scale,
//                                            pwm_gt pwm_deadband, ctrl_gt current_deadband)
//{
//    bridge->pwm_full_scale = pwm_full_scale;
//    // Pre-calculate and store half of the deadband for runtime efficiency.
//    bridge->pwm_deadband_half = pwm_deadband / 2;
//    bridge->current_deadband = current_deadband;
//
//    ctl_clear_three_phase_bridge_modulation(bridge);
//}

//////////////////////////////////////////////////////////////////////////
// Three Phase GFL Converter
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/three_phase/three_phase_GFL.h>

/**
 * @brief Auto-tuning GFL inverter parameters.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[in,out] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_auto_tuning_gfl_inv(gfl_inv_ctrl_init_t* init)
{
    parameter_gt T_res;
    parameter_gt LC_res_Hz;

    parameter_gt freq1;
    parameter_gt freq2;

    parameter_gt control_delay;
    parameter_gt filter_delay;

    assert(init->grid_filter_L > 1e-9f);

    // Select proper ADC digital filter cut frequency
    init->current_adc_fc = init->fs / 3;
    init->voltage_adc_fc = init->fs / 3;

    // Only L filter grid connector
    if (init->grid_filter_C < 1e-9f)
    {
        init->current_loop_bw = init->fs / 10;
        init->active_damping_resister = 0;
    }
    // LC filter grid connector
    else
    {
        // Calculate LC filter resonant frequency
        T_res = CTL_PARAM_CONST_2PI * sqrtf(init->grid_filter_L * init->grid_filter_C);
        LC_res_Hz = 1.0f / T_res;

        // select current loop BW
        freq1 = LC_res_Hz / 3;
        freq2 = init->fs / 10;
        init->current_loop_bw = fminf(freq1, freq2);

        // Calculate LC filter characteristic impedance, damping ratio is 0.5
        init->active_damping_resister = sqrtf(init->grid_filter_L / init->grid_filter_C);

        // calculate active_damping_center_freq and active_damping_Q
        init->active_damping_center_freq = LC_res_Hz;
        init->active_damping_filter_q = 1.0f;
    }

    // select current loop zero
    init->current_loop_zero = init->current_loop_bw / 10;

    // controller delay
    control_delay = CTL_PARAM_CONST_2PI * init->current_loop_bw * 1.5f / init->fs;

    // Create a LPF object and calculate phase lag
    ctl_filter_IIR1_t temp_filter;
    ctl_init_filter_iir1_lpf(&temp_filter, init->fs, init->current_adc_fc);
    filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_zero);

    init->current_phase_lag = control_delay + filter_delay;

    // select PLL bandwidth is freq_base / 3
    init->kp_pll = 2.0f * 0.707f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI;
    init->ki_pll = init->freq_base / 3.0f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI * CTL_PARAM_CONST_2PI;
}

/**
 * @brief update GFL inverter controller parameters by init structure.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_update_gfl_inv_coeff(gfl_inv_ctrl_t* inv, gfl_inv_ctrl_init_t* init)
{
    int i = 0;

    for (i = 0; i < 3; ++i)
    {
        ctl_init_filter_iir1_lpf(&inv->filter_iabc[i], init->fs, init->current_adc_fc);
        ctl_init_filter_iir1_lpf(&inv->filter_uabc[i], init->fs, init->voltage_adc_fc);
    }

    parameter_gt kp_dq =
        init->grid_filter_L * CTL_PARAM_CONST_2PI * init->current_loop_bw * init->i_base / init->v_base;
    parameter_gt ki_dq = kp_dq * CTL_PARAM_CONST_2PI * init->current_loop_zero;

    ctl_init_pid(&inv->pid_idq[phase_d], kp_dq, ki_dq, 0, init->fs);
    ctl_init_pid(&inv->pid_idq[phase_q], kp_dq, ki_dq, 0, init->fs);

    ctl_init_lead_form3(&inv->lead_compensator, init->current_phase_lag, init->current_loop_bw, init->fs);

    ctl_init_sfr_pll(&inv->pll, init->freq_base, init->kp_pll, init->ki_pll, 0, init->fs);

    ctl_init_ramp_generator_via_freq(&inv->rg, init->fs, init->freq_base, 1, 0);

    // Only L filter grid connector
    if (init->grid_filter_C < 1e-9f)
    {
        // close active damping
        inv->flag_enable_active_damping = 0;
        inv->coef_ff_damping = 0;
    }
    // LC filter grid connector
    else
    {
#if GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 1 || GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 2
        // active damping voltage = damping_gain * iCdq
        inv->coef_ff_damping = float2ctrl(init->active_damping_resister * init->i_base / init->v_base);
#elif GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 3
        // active damping voltage = damping_gain * (udq - udq_last)
        inv->coef_ff_damping = float2ctrl(init->active_damping_resister * init->grid_filter_C * init->fs);
#elif GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 4
        // TODO FIX HERE
#endif // GFL_CAPACITOR_CURRENT_CALCULATE_MODE

        ctl_init_biquad_bpf(&inv->filter_damping, init->fs, init->active_damping_center_freq,
                            init->active_damping_filter_q);
    }

    // decoupling feed-forward
    inv->coef_ff_decouple = CTL_PARAM_CONST_2PI * init->freq_base * init->i_base / init->v_base;
}

/**
 * @brief Init GFL inverter parameters.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_init_gfl_inv(gfl_inv_ctrl_t* inv, gfl_inv_ctrl_init_t* init)
{
    ctl_update_gfl_inv_coeff(inv, init);
    ctl_clear_gfl_inv_with_PLL(inv);
}

/**
 * @brief Attach GFL inverter with input/output interface.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[in,out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] pwm_out Pointer to the tri-channel PWM interface structure.
 * @param[in] adc_udc Pointer to the udc ADC interface structure.
 * @param[in] adc_idc Pointer to the idc ADC interface structure.
 * @param[in] adc_iabc Pointer to the tri-channel ADC current interface structure.
 * @param[in] adc_vabc Pointer to the tri-channel ADC voltage interface structure.
 */
void ctl_attach_gfl_inv(gfl_inv_ctrl_t* inv, adc_ift* adc_idc, adc_ift* adc_udc,
                        tri_adc_ift* adc_iabc, tri_adc_ift* adc_vabc)
{
    inv->adc_idc = adc_idc;
    inv->adc_udc = adc_udc;
    inv->adc_iabc = adc_iabc;
    inv->adc_vabc = adc_vabc;
}

//////////////////////////////////////////////////////////////////////////
// Three Phase GFL Converter
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/three_phase/three_phase_GFL.h>

void ctl_init_gfl_pq(gfl_pq_ctrl_t* pq, parameter_gt p_kp, parameter_gt p_ki, parameter_gt q_kp, parameter_gt q_ki,
                     parameter_gt i_out_max, parameter_gt fs)
{
    pq->flag_enable = 0;
    ctl_init_pid(&pq->pid_p, p_kp, p_ki, 0, fs);
    ctl_init_pid(&pq->pid_q, q_kp, q_ki, 0, fs);
    pq->max_i2_mag = float2ctrl(i_out_max * i_out_max);
}

/**
 * @brief Reset the PQ controller (clear integrators).
 * @param[in,out] pq Pointer to the PQ controller instance.
 */
void ctl_clear_gfl_pq(gfl_pq_ctrl_t* pq)
{
    ctl_clear_pid(&pq->pid_p);
    ctl_clear_pid(&pq->pid_q);

    pq->idq_set_out.dat[phase_d] = 0;
    pq->idq_set_out.dat[phase_q] = 0;
}

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq(gfl_pq_ctrl_t* pq, ctl_vector2_t* vdq, ctl_vector2_t* idq)
{
    pq->vdq_meas = vdq;
    pq->idq_meas = idq;
}

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq_to_core(gfl_pq_ctrl_t* pq, gfl_inv_ctrl_t* core)
{
    gmp_base_assert(core);
    gmp_base_assert(pq);

    pq->vdq_meas = &core->vdq;
    pq->idq_meas = &core->idq;
}
