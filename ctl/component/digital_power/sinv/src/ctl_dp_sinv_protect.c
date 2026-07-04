
#include <gmp_core.h>

#include <ctl/component/digital_power/sinv/sinv_protect.h>

/**
 * @brief Initializes the SINV Protection Manager with smart defaults.
 * 
 * @param[out] prot Pointer to the protection manager.
 * @param[in]  init Pointer to the configuration structure.
 */
void ctl_init_sinv_protect(ctl_sinv_protect_t* prot, const ctl_sinv_prot_init_t* init)
{
    // 1. 策略配置
    prot->error_mask = init->error_mask;
    prot->warning_mask = init->warning_mask;

    // 2. 智能默认值填充 (Smart Defaults)
    // 如果用户没填，给予工业上常见的保守值
    parameter_gt v_bus_max = (init->v_bus_max > 0.1f) ? init->v_bus_max : 420.0f;
    parameter_gt i_ac_max = (init->i_ac_max > 0.1f) ? init->i_ac_max : 20.0f;
    parameter_gt v_ctrl_max = (init->v_ctrl_max > 0.1f) ? init->v_ctrl_max : 1.5f;

    parameter_gt v_ac_rms_max = (init->v_ac_rms_max > 0.1f) ? init->v_ac_rms_max : 260.0f;
    parameter_gt v_ac_rms_min = (init->v_ac_rms_min > 0.1f) ? init->v_ac_rms_min : 180.0f;

    parameter_gt freq_nom = (init->freq_grid_nom > 0.1f) ? init->freq_grid_nom : 50.0f;
    parameter_gt freq_dev = (init->freq_dev_max > 0.01f) ? init->freq_dev_max : 0.5f;

    parameter_gt temp_max = (init->igbt_temp_max > 0.1f) ? init->igbt_temp_max : 85.0f;
    parameter_gt pt_gain = (init->pt_adc_gain > 0.001f) ? init->pt_adc_gain : 1.0f;
    parameter_gt pt_r0 = (init->pt_r0 > 10.0f) ? init->pt_r0 : 1000.0f; // 默认PT1000

    parameter_gt i_rated = (init->i_ac_rated_rms > 0.1f) ? init->i_ac_rated_rms : 10.0f;
    parameter_gt i2t_limit = (init->i2t_limit > 0.1f) ? init->i2t_limit : 1000.0f;

    // 3. 实例化并配置底层节点
    // (1) 快保护：极短消抖 (例如 2个 $20\text{kHz}$ 节拍，即 $100\mu\text{s}$)
    ctl_init_prot_single(&prot->node_dc_ovp_fast, SINV_PROT_BIT_DC_OVP_FAST, v_bus_max, 2);
    ctl_init_prot_single(&prot->node_ac_ocp_fast, SINV_PROT_BIT_AC_OCP_FAST, i_ac_max, 2);
    ctl_init_prot_single(&prot->node_ctrl_diverge, SINV_PROT_BIT_CTRL_DIVERGE, v_ctrl_max, 5);

    // (2) 慢保护：较长消抖 (例如 10 个 $1\text{ms}$ 节拍，即 $10\text{ms}$)
    // 注意：欠压 (UVP) 往往需要极长的消抖以支持 LVRT (低电压穿越)
    ctl_init_prot_single(&prot->node_ac_ovp_rms, SINV_PROT_BIT_AC_OVP_RMS, v_ac_rms_max, 20);
    ctl_init_prot_single(&prot->node_ac_uvp_rms, SINV_PROT_BIT_AC_UVP_RMS, v_ac_rms_min, 500); // 500ms 穿越

    ctl_init_prot_window(&prot->node_pll_freq, SINV_PROT_BIT_PLL_FREQ_ERR, freq_nom + freq_dev, freq_nom - freq_dev,
                         20);

    // (3) 物理传感器保护
    ctl_init_prot_pt_sensor(&prot->node_igbt_temp, SINV_PROT_BIT_IGBT_TEMP_OVP, temp_max, pt_gain, pt_r0, 0.00385f,
                            100); // 100ms 消抖

    ctl_init_prot_thermal(&prot->node_igbt_i2t, SINV_PROT_BIT_IGBT_THERMAL_I2T, i_rated, i2t_limit);

    // 4. 全部使能 (Enable All by default)
    prot->node_dc_ovp_fast.is_enabled = 1;
    prot->node_ac_ocp_fast.is_enabled = 1;
    prot->node_ctrl_diverge.is_enabled = 1;
    prot->node_ac_ovp_rms.is_enabled = 1;
    prot->node_ac_uvp_rms.is_enabled = 1;
    prot->node_pll_freq.is_enabled = 1;
    prot->node_igbt_temp.is_enabled = 1;
    prot->node_igbt_i2t.is_enabled = 1;

    // 5. 执行一次深度复位，确保状态清零
    ctl_reset_sinv_protect(prot);
}
