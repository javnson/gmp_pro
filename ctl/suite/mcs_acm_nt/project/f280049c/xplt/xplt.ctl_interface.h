//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include <xplt.peripheral.h>

#if defined ENABLE_GMP_DL_PIL_SIM
#include <core/dev/pil_core.h>
#endif

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#if defined ENABLE_GMP_DL_PIL_SIM
/** @brief Virtual plant-enable state exported through the PIL digital word. */
extern volatile fast_gt pil_output_enabled;
#endif


//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // copy source ADC data
    uuvw_src[phase_U] = ADC_readResult(INV_UU_RESULT_BASE, INV_UU);
    uuvw_src[phase_V] = ADC_readResult(INV_UV_RESULT_BASE, INV_UV);
    uuvw_src[phase_W] = ADC_readResult(INV_UW_RESULT_BASE, INV_UW);

    iuvw_src[phase_U] = ADC_readResult(INV_IU_RESULT_BASE, INV_IU);
    iuvw_src[phase_V] = ADC_readResult(INV_IV_RESULT_BASE, INV_IV);
    iuvw_src[phase_W] = ADC_readResult(INV_IW_RESULT_BASE, INV_IW);

    udc_src = ADC_readResult(INV_VBUS_RESULT_BASE, INV_VBUS);
//    idc_src = ADC_readResult(INV_IBUS_RESULT_BASE, INV_IBUS);

    // Step auto turn pos encoder
    ctl_step_autoturn_pos_encoder(&pos_enc, EQEP_getPosition(EQEP_Encoder_BASE));

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    // Write ePWM peripheral CMP
    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_U]);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_V]);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_W]);


    // Monitor Port
#if BUILD_LEVEL == 1

    DAC_setShadowValue(LAUNCHXL_DACA_BASE, spwm.vabc_out.dat[phase_U] * 2048 + 2048);
    DAC_setShadowValue(LAUNCHXL_DACB_BASE, mtr_ctrl.iab0.dat[phase_alpha] * 2048 + 2048);

#elif BUILD_LEVEL == 2

//    DAC_setShadowValue(LAUNCHXL_DACA_BASE, ctl_mul(ctl_mul(CTL_CTRL_CONST_1_OVER_SQRT3, mtr_ctrl.udc), mtr_ctrl.vab0.dat[phase_alpha]) * 2048 + 2048);
//    DAC_setShadowValue(LAUNCHXL_DACB_BASE, mtr_ctrl.iab0.dat[phase_alpha] * 2048 + 2048);

    DAC_setShadowValue(LAUNCHXL_DACA_BASE, rg.enc.elec_position * 2048 + 2048);
    DAC_setShadowValue(LAUNCHXL_DACB_BASE, pos_enc.encif.elec_position * 2048 + 2048);

#elif BUILD_LEVEL == 3
    /* DACA is the electrical field angle used by the ACIM current core.
     * DACB is the directly measured rotor electrical angle. */
    DAC_setShadowValue(LAUNCHXL_DACA_BASE, mtr_ctrl.field_pos_if->elec_position * 2048 + 2048);
    DAC_setShadowValue(LAUNCHXL_DACB_BASE, pos_enc.encif.elec_position * 2048 + 2048);

#elif BUILD_LEVEL == 4
    DAC_setShadowValue(LAUNCHXL_DACA_BASE, acim_sync_speed_pu * 2048 + 2048);
    DAC_setShadowValue(LAUNCHXL_DACB_BASE, pos_enc.encif.elec_position * 2048 + 2048);
#endif // BUILD_LEVEL
}

// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

/**
 * @brief Force every physical inverter output into its safe state.
 * @details This helper is also used when PIL is enabled so that simulated
 * control state transitions can never energize the disconnected power stage.
 */
GMP_STATIC_INLINE void ctl_force_physical_output_safe(void)
{
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);
    GPIO_WritePin(PWM_ENABLE_PORT, 0);
    GPIO_WritePin(CONTROLLER_LED, 1);
}

/** @brief Enable physical PWM output unless this is an isolated PIL build. */
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
#if defined ENABLE_GMP_DL_PIL_SIM
    pil_output_enabled = 1;
    ctl_force_physical_output_safe();
    clear_all_controllers();
#else
    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    clear_all_controllers();

    // PWM enable
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    GPIO_WritePin(CONTROLLER_LED, 0);
#endif
}

/** @brief Disable physical PWM output in every operating mode. */
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
#if defined ENABLE_GMP_DL_PIL_SIM
    pil_output_enabled = 0;
#endif
    ctl_force_physical_output_safe();
}

//=================================================================================================
// Controller interface for PIL simulation

#if defined ENABLE_GMP_DL_PIL_SIM

/** @brief Apply one SDPE-mapped PIL input sample to the controller ports. */
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    // copy source ADC data
    uuvw_src[phase_U] = rx->adc_result[GMP_PIL_RX_ADC_UU_INDEX];
    uuvw_src[phase_V] = rx->adc_result[GMP_PIL_RX_ADC_UV_INDEX];
    uuvw_src[phase_W] = rx->adc_result[GMP_PIL_RX_ADC_UW_INDEX];

    iuvw_src[phase_U] = rx->adc_result[GMP_PIL_RX_ADC_IU_INDEX];
    iuvw_src[phase_V] = rx->adc_result[GMP_PIL_RX_ADC_IV_INDEX];
    iuvw_src[phase_W] = rx->adc_result[GMP_PIL_RX_ADC_IW_INDEX];

    udc_src = rx->adc_result[GMP_PIL_RX_ADC_UDC_INDEX];

    // Step auto turn pos encoder
    ctl_step_autoturn_pos_encoder(&pos_enc, rx->digital_input);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

/** @brief Export one controller result through the SDPE-mapped PIL channels. */
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    tx->digital_out = pil_output_enabled ? 1U : 0U;

    //
    // PWM channel
    //
    tx->pwm_cmp[GMP_PIL_TX_PWM_U_INDEX] = spwm.pwm_out[phase_U];
    tx->pwm_cmp[GMP_PIL_TX_PWM_V_INDEX] = spwm.pwm_out[phase_V];
    tx->pwm_cmp[GMP_PIL_TX_PWM_W_INDEX] = spwm.pwm_out[phase_W];

    //
    // monitor
    //

    tx->monitor[GMP_PIL_TX_MONITOR_IU_INDEX] = mtr_ctrl.iuvw.dat[phase_U];
    tx->monitor[GMP_PIL_TX_MONITOR_IV_INDEX] = mtr_ctrl.iuvw.dat[phase_V];
    tx->monitor[GMP_PIL_TX_MONITOR_ID_INDEX] = mtr_ctrl.idq0.dat[phase_d];
    tx->monitor[GMP_PIL_TX_MONITOR_IQ_INDEX] = mtr_ctrl.idq0.dat[phase_q];
    /* Position is the rotor-flux field angle. Speed is deliberately the
     * mechanical rotor feedback used by the speed loop, not field speed. */
    tx->monitor[GMP_PIL_TX_MONITOR_POSITION_INDEX] = mtr_ctrl.field_pos_if->elec_position;
    tx->monitor[GMP_PIL_TX_MONITOR_SPEED_INDEX] = spd_enc.encif.speed;
}

#endif // defined ENABLE_GMP_DL_PIL_SIM

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
