/** @file xplt.ctl_interface.h @brief SIL packet-to-controller mapping. */

#include <ctl/component/motor_control/interface/std_sil_motor_interface.h>
#include <xplt.peripheral.h>

#ifndef _FILE_MCS_ACIM_NT_CTL_INTERFACE_H_
#define _FILE_MCS_ACIM_NT_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum _tag_acim_adc_index
{
    ACIM_ADC_UDC = 0,
    ACIM_ADC_UA = 1,
    ACIM_ADC_UB = 2,
    ACIM_ADC_UC = 3,
    ACIM_ADC_IA = 4,
    ACIM_ADC_IB = 5,
    ACIM_ADC_IC = 6
} acim_adc_index_t;

GMP_STATIC_INLINE void ctl_input_callback(void)
{
    uuvw_src[phase_U] = simulink_rx_buffer.adc_result[ACIM_ADC_UA];
    uuvw_src[phase_V] = simulink_rx_buffer.adc_result[ACIM_ADC_UB];
    uuvw_src[phase_W] = simulink_rx_buffer.adc_result[ACIM_ADC_UC];
    iuvw_src[phase_U] = simulink_rx_buffer.adc_result[ACIM_ADC_IA];
    iuvw_src[phase_V] = simulink_rx_buffer.adc_result[ACIM_ADC_IB];
    iuvw_src[phase_W] = simulink_rx_buffer.adc_result[ACIM_ADC_IC];
    udc_src = simulink_rx_buffer.adc_result[ACIM_ADC_UDC];

    ctl_step_autoturn_pos_encoder(&pos_enc, (uint32_t)simulink_rx_buffer.digital[0]);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

GMP_STATIC_INLINE void ctl_output_callback(void)
{
    ctl_vector3_t uab0_monitor;
    ctl_ct_clarke(&uuvw.control_port.value, &uab0_monitor);

    simulink_tx_buffer.pwm_cmp[0] = spwm.pwm_out[phase_U];
    simulink_tx_buffer.pwm_cmp[1] = spwm.pwm_out[phase_V];
    simulink_tx_buffer.pwm_cmp[2] = spwm.pwm_out[phase_W];

    simulink_tx_buffer.monitor[0] = ctrl2param(mtr_ctrl.idq_ref.dat[phase_d]);
    simulink_tx_buffer.monitor[1] = ctrl2param(mtr_ctrl.idq0.dat[phase_d]);
    simulink_tx_buffer.monitor[2] = ctrl2param(mtr_ctrl.idq_ref.dat[phase_q]);
    simulink_tx_buffer.monitor[3] = ctrl2param(mtr_ctrl.idq0.dat[phase_q]);
    /* Field angle (1 pu = one electrical revolution), not shaft angle. */
    simulink_tx_buffer.monitor[4] = ctrl2param(mtr_ctrl.field_pos_if->elec_position);
    simulink_tx_buffer.monitor[5] = ctrl2param(spd_enc.encif.speed);
    simulink_tx_buffer.monitor[6] = ctrl2param(acim_sync_speed_pu);
#if MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORED
    simulink_tx_buffer.monitor[7] = ctrl2param(acim_pos_calc.w_slip_pu);
#else
    simulink_tx_buffer.monitor[7] = ctrl2param(acim_fo.slip_speed_pu);
#endif
#if (BUILD_LEVEL >= 3) && (MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS)
    simulink_tx_buffer.monitor[8] = ctrl2param(acim_fo.psi_s_est.dat[phase_alpha]);
    simulink_tx_buffer.monitor[9] = ctrl2param(acim_fo.psi_s_ref.dat[phase_alpha]);
    simulink_tx_buffer.monitor[10] = ctrl2param(acim_fo.psi_s_est.dat[phase_beta]);
    simulink_tx_buffer.monitor[11] = ctrl2param(acim_fo.psi_s_ref.dat[phase_beta]);
#else
    simulink_tx_buffer.monitor[8] = ctrl2param(mtr_ctrl.vdq_out.dat[phase_d]);
    simulink_tx_buffer.monitor[9] = ctrl2param(mtr_ctrl.vdq_out.dat[phase_q]);
    simulink_tx_buffer.monitor[10] = ctrl2param(acim_fo.psi_r_mag);
    simulink_tx_buffer.monitor[11] = (double)acim_sensorless_handover;
#endif
    /* BUILD_LEVEL 1/2 commissioning channels: stationary-frame current and
     * directly sampled phase voltage. They remain useful after loop closure. */
    simulink_tx_buffer.monitor[12] = ctrl2param(mtr_ctrl.iab0.dat[phase_alpha]);
    simulink_tx_buffer.monitor[13] = ctrl2param(mtr_ctrl.iab0.dat[phase_beta]);
#if (BUILD_LEVEL >= 3) && (MCS_ACIM_FEEDBACK_MODE == MCS_ACIM_FEEDBACK_SENSORLESS)
    /* Handover diagnostics: compare CH5/CH7 (angle/speed used by the current
     * core) with CH15/CH16 (raw observer angle/speed). */
    simulink_tx_buffer.monitor[12] = ctrl2param(acim_fo.psi_r_est.dat[phase_alpha]);
    simulink_tx_buffer.monitor[13] = ctrl2param(acim_fo.psi_r_est.dat[phase_beta]);
    simulink_tx_buffer.monitor[14] = ctrl2param(acim_fo.pos_out.elec_position);
    simulink_tx_buffer.monitor[15] = ctrl2param(acim_fo.sync_spd_out.speed);
#else
    simulink_tx_buffer.monitor[14] = ctrl2param(uab0_monitor.dat[phase_alpha]);
    simulink_tx_buffer.monitor[15] = ctrl2param(uab0_monitor.dat[phase_beta]);
#endif
}

GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    csp_sl_enable_output();
}

GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    csp_sl_disable_output();
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_MCS_ACIM_NT_CTL_INTERFACE_H_
