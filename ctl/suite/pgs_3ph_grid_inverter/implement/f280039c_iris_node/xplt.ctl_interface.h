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

#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

#include <xplt.peripheral.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// IRIS Board Pin mapping
//

#ifndef BOARD_PIN_MAPPING
#define BOARD_PIN_MAPPING

// PWM Channels
#define PHASE_U_BASE IRIS_EPWM1_BASE
#define PHASE_V_BASE IRIS_EPWM2_BASE
#define PHASE_W_BASE IRIS_EPWM3_BASE

// PWM Enable
#define PWM_ENABLE_PORT IRIS_GPIO1
#define PWM_RESET_PORT IRIS_GPIO3

// Vbus Voltage Channels
//#define MOTOR_VBUS_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_VBUS

// ADC Voltage Channels
//#define MOTOR_VU_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_VV_RESULT_BASE IRIS_ADCB_RESULT_BASE
//#define MOTOR_VW_RESULT_BASE IRIS_ADCC_RESULT_BASE

//#define MOTOR_VU
//#define MOTOR_VV
//#define MOTOR_VW

// ADC Current Channels
//#define MOTOR_IU_RESULT_BASE IRIS_ADCA_RESULT_BASE
//#define MOTOR_IV_RESULT_BASE IRIS_ADCB_RESULT_BASE
//#define MOTOR_IW_RESULT_BASE IRIS_ADCC_RESULT_BASE

//#define MOTOR_IU
//#define MOTOR_IV
//#define MOTOR_IW

#endif //BOARD_PIN_MAPPING

//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//

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
    idc_src = ADC_readResult(INV_VBUS_RESULT_BASE, INV_VBUS);

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
    // copy source pwm data
    pwm_out.raw.value.dat[phase_U] = inv_ctrl.pwm_out_pu.dat[phase_U];
    pwm_out.raw.value.dat[phase_V] = inv_ctrl.pwm_out_pu.dat[phase_V];
    pwm_out.raw.value.dat[phase_W] = inv_ctrl.pwm_out_pu.dat[phase_W];

    // invoke PWM p.u. routine
    ctl_calc_pwm_tri_channel(&pwm_out);

    // PWM output,EPWM1_BASE
    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[0]);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[1]);

    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[0]);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[1]);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[2]);

    // Monitor Port, 8 channels
#if BUILD_LEVEL == 1

//    DAC_setShadowValue(IRIS_DACB_BASE, inv_ctrl.angle * 2048 + 2048);
//    DAC_setShadowValue(IRIS_DACA_BASE, inv_ctrl.abc_out.dat[phase_B]  * 2048 + 2048);

    // grid current and inverter current
//    DAC_setShadowValue(IRIS_DACB_BASE, iuvw.control_port.value.dat[phase_A] * 2048 + 2048);
//    DAC_setShadowValue(IRIS_DACA_BASE, iabc.control_port.value.dat[phase_A] * 2048 + 2048);

    // grid voltage and inverter voltage
    DAC_setShadowValue(IRIS_DACB_BASE, uuvw.control_port.value.dat[phase_A] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACA_BASE, vabc.control_port.value.dat[phase_A] * 2048 + 2048);

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
GMP_STATIC_INLINE void ctl_enable_output()
{
    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    ctl_enable_three_phase_inverter(&inv_ctrl);

    // PWM enable
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    GPIO_WritePin(PWM_RESET_PORT, 0);

    GPIO_WritePin(IRIS_LED2, 0);
}

// Disable Output
GMP_STATIC_INLINE void ctl_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(PHASE_U_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_V_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_W_BASE, EPWM_TZ_FORCE_EVENT_OST);

    ctl_disable_three_phase_inverter(&inv_ctrl);

    // PWM disable
    GPIO_WritePin(PWM_ENABLE_PORT, 0);

    GPIO_WritePin(IRIS_LED2, 1);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
