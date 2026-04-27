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

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus



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

    DAC_setShadowValue(IRIS_DACA_BASE, spwm.vabc_out.dat[phase_U] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, mtr_ctrl.iab0.dat[phase_alpha] * 2048 + 2048);

#elif BUILD_LEVEL == 2

//    DAC_setShadowValue(IRIS_DACA_BASE, ctl_mul(ctl_mul(CTL_CTRL_CONST_1_OVER_SQRT3, mtr_ctrl.udc), mtr_ctrl.vab0.dat[phase_alpha]) * 2048 + 2048);
//    DAC_setShadowValue(IRIS_DACB_BASE, mtr_ctrl.iab0.dat[phase_alpha] * 2048 + 2048);

    // SMO Observer, back EMF
//    DAC_setShadowValue(IRIS_DACA_BASE, smo.e_est.dat[phase_A] * 2048 + 2048);
//    DAC_setShadowValue(IRIS_DACB_BASE, smo.e_est.dat[phase_B] * 2048 + 2048);

    // SMO angle
    DAC_setShadowValue(IRIS_DACA_BASE, smo.pos_out.elec_position * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, pos_enc.encif.elec_position * 2048 + 2048);

#elif BUILD_LEVEL == 3
    // SMO angle
    DAC_setShadowValue(IRIS_DACA_BASE, smo.pos_out.elec_position * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, pos_enc.encif.elec_position * 2048 + 2048);

#elif BUILD_LEVEL == 4
    // SMO angle
    DAC_setShadowValue(IRIS_DACA_BASE, smo.pos_out.elec_position * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, pos_enc.encif.elec_position * 2048 + 2048);
#endif // BUILD_LEVEL
}

// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_fast_enable_output()
{
    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    clear_all_controllers();

    // PWM enable
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    GPIO_WritePin(CONTROLLER_LED, 0);
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

//    clear_all_controllers();

    // PWM disable
    GPIO_WritePin(PWM_ENABLE_PORT, 0);

    GPIO_WritePin(CONTROLLER_LED, 1);
}

//=================================================================================================
// Controller interface for PIL simulation

typedef enum _tag_adc_index_items
{
    INV_ADC_ID_UDC = 0,

    INV_ADC_ID_UA = 1,
    INV_ADC_ID_UB = 2,
    INV_ADC_ID_UC = 3,

    INV_ADC_ID_IA = 4,
    INV_ADC_ID_IB = 5,
    INV_ADC_ID_IC = 6,
    INV_ADC_SENSOR_NUMBER = 7

} inv_adc_index_items;

typedef enum _tag_digital_index_items
{
    MTR1_ENCODER_OUTPUT = 0,
    MTR1_ENCODER_TURNS = 1,

    DIGITAL_INDEX_NUMBER = 2
} digital_index_items;

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    // copy source ADC data
    uuvw_src[phase_U] = rx->adc_result[INV_ADC_ID_UA];
    uuvw_src[phase_V] = rx->adc_result[INV_ADC_ID_UB];
    uuvw_src[phase_W] = rx->adc_result[INV_ADC_ID_UC];

    iuvw_src[phase_U] = rx->adc_result[INV_ADC_ID_IA];
    iuvw_src[phase_V] = rx->adc_result[INV_ADC_ID_IB];
    iuvw_src[phase_W] = rx->adc_result[INV_ADC_ID_IC];

    udc_src = rx->adc_result[INV_ADC_ID_UDC];

    // Step auto turn pos encoder
    ctl_step_autoturn_pos_encoder(&pos_enc, rx->digital_input);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    //
    // PWM channel
    //
    tx->pwm_cmp[0] = spwm.pwm_out[phase_U];
    tx->pwm_cmp[1] = spwm.pwm_out[phase_V];
    tx->pwm_cmp[2] = spwm.pwm_out[phase_W];

    //
    // monitor
    //

    // Scope 1
    tx->monitor[0] = mtr_ctrl.iuvw.dat[phase_A];
    tx->monitor[1] = mtr_ctrl.iuvw.dat[phase_B];
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
