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
// Three Phase GFL Converter
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/inv/gfl_core.h>

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

        // Auto tuning conservative design
        init->current_loop_bw = fminf(freq1, freq2) / 2;

        //        init->current_loop_bw = fminf(freq1, freq2);

        // Calculate LC filter characteristic impedance, damping ratio is 0.5
        parameter_gt k_damping_filter = 0.2f;
        init->active_damping_resister = k_damping_filter * sqrtf(init->grid_filter_L / init->grid_filter_C);

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
    filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_bw);

    init->current_phase_lag = control_delay + filter_delay;

    // select PLL bandwidth is freq_base / 3
    init->pll_bw = init->freq_base / 3;
    init->k_pll_sogi = 1.414f;

    //// 1. 定义物理常数
    //// 系统固有的环路增益：2 * PI * f_base (约 314.159)
    //parameter_gt system_gain = CTL_PARAM_CONST_2PI * init->freq_base;

    //// 2. 定义设计目标
    //// 设定带宽为基频的 1/3 (约 16.7Hz)
    //parameter_gt bandwidth_hz = init->freq_base / 3.0f;
    //parameter_gt omega_n = bandwidth_hz * CTL_PARAM_CONST_2PI;
    //parameter_gt damping = 0.707f;

    //// 3. 计算修正后的 PI 参数 (除以 system_gain)
    //// Kp = (2 * zeta * wn) / system_gain
    //init->kp_pll = (2.0f * damping * omega_n) / system_gain;

    //// Ki = (wn^2) / system_gain
    //init->ki_pll = (omega_n * omega_n) / system_gain;

    //    init->kp_pll = 2.0f * 0.707f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI;
    //    init->ki_pll = init->freq_base / 3.0f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI * CTL_PARAM_CONST_2PI;
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

    ctl_init_lead_form3(&inv->lead_compensator[phase_d], init->current_phase_lag, init->current_loop_bw, init->fs);
    ctl_init_lead_form3(&inv->lead_compensator[phase_q], init->current_phase_lag, init->current_loop_bw, init->fs);

#ifdef USING_DSOGI_PLL
    ctl_init_dsogi_pll(&inv->pll, init->freq_base, init->fs, init->v_grid, init->pll_bw, init->k_pll_sogi);
#else
    ctl_init_srf_pll_auto_tune(&inv->pll, init->freq_base, init->fs, init->v_grid, init->pll_bw, 0.707f);
#endif // USING_DSOGI_PLL

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

        ctl_init_biquad_bpf(&inv->filter_damping[phase_d], init->fs, init->active_damping_center_freq,
                            init->active_damping_filter_q);
        ctl_init_biquad_bpf(&inv->filter_damping[phase_q], init->fs, init->active_damping_center_freq,
                            init->active_damping_filter_q);
    }

    // decoupling feed-forward
    inv->coef_ff_decouple = CTL_PARAM_CONST_2PI * init->grid_filter_L * init->freq_base * init->i_base / init->v_base;
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
void ctl_attach_gfl_inv(gfl_inv_ctrl_t* inv, adc_ift* adc_idc, adc_ift* adc_udc, tri_adc_ift* adc_iabc,
                        tri_adc_ift* adc_vabc)
{
    inv->adc_idc = adc_idc;
    inv->adc_udc = adc_udc;
    inv->adc_iabc = adc_iabc;
    inv->adc_vabc = adc_vabc;
}
