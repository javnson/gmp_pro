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

#include <core/dev/pil_core.h>

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
    vabc_src[phase_A] = ADC_readResult(INV_UA_RESULT_BASE, INV_UA);
    vabc_src[phase_B] = ADC_readResult(INV_UB_RESULT_BASE, INV_UB);
    vabc_src[phase_C] = ADC_readResult(INV_UC_RESULT_BASE, INV_UC);

    iabc_src[phase_A] = ADC_readResult(INV_IA_RESULT_BASE, INV_IA);
    iabc_src[phase_B] = ADC_readResult(INV_IB_RESULT_BASE, INV_IB);
    iabc_src[phase_C] = ADC_readResult(INV_IC_RESULT_BASE, INV_IC);

    uuvw_src[phase_U] = ADC_readResult(INV_UU_RESULT_BASE, INV_UU);
    uuvw_src[phase_V] = ADC_readResult(INV_UV_RESULT_BASE, INV_UV);
    uuvw_src[phase_W] = ADC_readResult(INV_UW_RESULT_BASE, INV_UW);

    iuvw_src[phase_U] = ADC_readResult(INV_IU_RESULT_BASE, INV_IU);
    iuvw_src[phase_V] = ADC_readResult(INV_IV_RESULT_BASE, INV_IV);
    iuvw_src[phase_W] = ADC_readResult(INV_IW_RESULT_BASE, INV_IW);

    udc_src = ADC_readResult(INV_VBUS_RESULT_BASE, INV_VBUS);
    idc_src = ADC_readResult(INV_IBUS_RESULT_BASE, INV_IBUS);

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    // Write ePWM peripheral CMP
#if defined USING_NPC_MODULATOR

    EPWM_setCounterCompareValue(EPWM_J4_PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_A_OUTER]);
    EPWM_setCounterCompareValue(EPWM_J4_PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_B_OUTER]);
    EPWM_setCounterCompareValue(EPWM_J4_PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_C_OUTER]);
    EPWM_setCounterCompareValue(EPWM_J8_PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_A_INNER]);
    EPWM_setCounterCompareValue(EPWM_J8_PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_B_INNER]);
    EPWM_setCounterCompareValue(EPWM_J8_PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[NPC_IDX_PHASE_C_INNER]);

#else

    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_U]);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_V]);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, spwm.pwm_out[phase_W]);

#endif // USING_NPC_MODULATOR

    // Monitor Port
#if BUILD_LEVEL == 1

    //    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.angle * 2048 + 2048);
    //    DAC_setShadowValue(IRIS_DACA_BASE, inv_ctrl.abc_out.dat[phase_B]  * 2048 + 2048);

    // grid current and inverter current
    DAC_setShadowValue(IRIS_DACA_BASE, iabc.control_port.value.dat[phase_C] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, iuvw.control_port.value.dat[phase_C] * 2048 + 2048);

    // grid voltage and inverter voltage
    //    DAC_setShadowValue(IRIS_DACB_BASE, uuvw.control_port.value.dat[phase_C] * 2048 + 2048);
    //    DAC_setShadowValue(IRIS_DACA_BASE, vabc.control_port.value.dat[phase_C] * 2048 + 2048);

    //    DAC_setShadowValue(IRIS_DACB_BASE, EPWM_getCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A)/2);
    //    DAC_setShadowValue(IRIS_DACA_BASE, EPWM_getCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A)/2);

    //    DAC_setShadowValue(IRIS_DACB_BASE, pwm_out.value[2]/2);
    //    DAC_setShadowValue(IRIS_DACA_BASE, pwm_out.value[1]/2);
    //    inv_ctrl.

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

    ctl_enable_gfl_inv(&inv_ctrl);

    // PWM enable
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    GPIO_WritePin(PWM_RESET_PORT, 0);

    GPIO_WritePin(CONTROLLER_LED, 0);
}

// Disable Output
GMP_STATIC_INLINE void ctl_fast_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    ctl_disable_gfl_inv(&inv_ctrl);

    // PWM disable
    GPIO_WritePin(PWM_ENABLE_PORT, 0);

    GPIO_WritePin(CONTROLLER_LED, 1);
}

//=================================================================================================
// Controller interface

typedef enum _tag_sinv_adc_index_items
{
    INV_ADC_ID_IDC = 0,
    INV_ADC_ID_VDC = 1,
    INV_ADC_ID_UAB = 2,
    INV_ADC_ID_UBC = 3,
    INV_ADC_ID_IA = 4,
    INV_ADC_ID_IB = 5,
    INV_ADC_ID_IC = 6,
    INV_ADC_SENSOR_NUMBER = 7

} inv_adc_index_items;

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    // copy source ADC data
    vabc_src[phase_A] = rx->adc_result[INV_ADC_ID_UAB];
    vabc_src[phase_B] = rx->adc_result[INV_ADC_ID_UBC];
    vabc_src[phase_C] = 0;

    iabc_src[phase_A] = rx->adc_result[INV_ADC_ID_IA];
    iabc_src[phase_B] = rx->adc_result[INV_ADC_ID_IB];
    iabc_src[phase_C] = rx->adc_result[INV_ADC_ID_IC];

    uuvw_src[phase_U] = 0;
    uuvw_src[phase_V] = 0;
    uuvw_src[phase_W] = 0;

    iuvw_src[phase_U] = 0;
    iuvw_src[phase_V] = 0;
    iuvw_src[phase_W] = 0;

    udc_src = rx->adc_result[INV_ADC_ID_IDC];
    idc_src = rx->adc_result[INV_ADC_ID_VDC];

    // invoke ADC p.u. routine
    ctl_step_tri_ptr_adc_channel(&iabc);
    ctl_step_tri_ptr_adc_channel(&vabc);
    ctl_step_tri_ptr_adc_channel(&iuvw);
    ctl_step_tri_ptr_adc_channel(&uuvw);
    ctl_step_ptr_adc_channel(&idc);
    ctl_step_ptr_adc_channel(&udc);
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    // Write ePWM peripheral CMP
    tx->pwm_cmp[0] = spwm.pwm_out[phase_U];
    tx->pwm_cmp[1] = spwm.pwm_out[phase_V];
    tx->pwm_cmp[2] = spwm.pwm_out[phase_W];

    // Monitor Port
#if BUILD_LEVEL == 1

    //
    // monitor
    //

    // Scope 1
    tx->monitor[0] = inv_ctrl.iabc.dat[phase_A];
    tx->monitor[1] = inv_ctrl.iabc.dat[phase_B];

    // Scope 2
    tx->monitor[2] = inv_ctrl.vab_pos.dat[phase_alpha];
    tx->monitor[3] = inv_ctrl.vab_pos.dat[phase_beta];

    // Scope 3
    tx->monitor[4] = inv_ctrl.vab0.dat[phase_alpha];
    tx->monitor[5] = inv_ctrl.vab0.dat[phase_beta];

    // Scope 4
    tx->monitor[6] = ctl_get_gfl_pll_error(&inv_ctrl);
    tx->monitor[7] = inv_ctrl.angle;

#endif // BUILD_LEVEL
}


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
