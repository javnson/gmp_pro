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


//////////////////////////////////////////////////////////////////////////
// IRIS Board Pin mapping
//

#ifndef BOARD_PIN_MAPPING
#define BOARD_PIN_MAPPING

// PWM Channels
#define PHASE_U_BASE IRIS_EPWM1_BASE
#define PHASE_V_BASE IRIS_EPWM2_BASE
#define PHASE_W_BASE IRIS_EPWM3_BASE

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

// QEP Encoder Channel
#define EQEP_Encoder_BASE IRIS_EQEP1_BASE


#endif //BOARD_PIN_MAPPING


//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//

// raw data
extern adc_gt uabc_raw[3];
extern adc_gt iabc_raw[3];
extern adc_gt udc_raw;
extern adc_gt idc_raw;
extern pmsm_controller_t pmsm_ctrl;


// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // update system tick
        gmp_step_system_tick();

        // copy ADC data to raw buffer
        // NOTICE use Result base not adc base.
        udc_raw = ADC_readResult(MOTOR_VBUS_RESULT_BASE, MOTOR_VBUS);

        uabc_raw[phase_U] = ADC_readResult(MOTOR_VU_RESULT_BASE, MOTOR_VU);
        uabc_raw[phase_V] = ADC_readResult(MOTOR_VV_RESULT_BASE, MOTOR_VV);
        uabc_raw[phase_W] = ADC_readResult(MOTOR_VW_RESULT_BASE, MOTOR_VW);

        iabc_raw[phase_U] = ADC_readResult(MOTOR_IU_RESULT_BASE, MOTOR_IU);
        iabc_raw[phase_V] = ADC_readResult(MOTOR_IV_RESULT_BASE, MOTOR_IV);
        iabc_raw[phase_W] = ADC_readResult(MOTOR_IW_RESULT_BASE, MOTOR_IW);

        // invoke ADC p.u. routine
        ctl_step_tri_ptr_adc_channel(&iabc);
        ctl_step_tri_ptr_adc_channel(&uabc);
        ctl_step_ptr_adc_channel(&idc);
        ctl_step_ptr_adc_channel(&udc);


#ifdef PMSM_CTRL_USING_QEP_ENCODER
    // Step auto turn pos encoder
    ctl_step_autoturn_pos_encoder(&pos_enc, EQEP_getPosition(EQEP_Encoder_BASE));
#else // PMSM_CTRL_USING_QEP_ENCODER
    // invoke position encoder routine.
    ctl_step_as5048a_pos_encoder(&pos_enc);
#endif // PMSM_CTRL_USING_QEP_ENCODER

}

extern uint32_t output_voltage_compare;

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    ctl_calc_pwm_tri_channel(&pwm_out);

    // PWM output
    EPWM_setCounterCompareValue(PHASE_U_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_U]);
    EPWM_setCounterCompareValue(PHASE_V_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_V]);
    EPWM_setCounterCompareValue(PHASE_W_BASE, EPWM_COUNTER_COMPARE_A, pwm_out.value[phase_W]);

    // Monitor Port
#if BUILD_LEVEL == 1

    // output DAC
//    DAC_setShadowValue(IRIS_DACA_BASE, pmsm_ctrl.iab0.dat[phase_A] * 2048 + 2048);
    DAC_setShadowValue(IRIS_DACB_BASE, rg.enc.position * 2048 + 2048);

    DAC_setShadowValue(IRIS_DACA_BASE, pmsm_ctrl.vab0_set.dat[phase_A] * 2048 + 2048);

#endif // BUILD_LEVEL
}


// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);


// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE void ctl_enable_output()
{
    //ctl_enable_pmsm_ctrl_output(&pmsm_ctrl);

    // Clear any Trip Zone flag
    EPWM_clearTripZoneFlag(IRIS_EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(IRIS_EPWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(IRIS_EPWM3_BASE, EPWM_TZ_FORCE_EVENT_OST);

    GPIO_WritePin(IRIS_LED2, 0);
}

// Disable Output
GMP_STATIC_INLINE void ctl_disable_output()
{
    // Disables the PWM device
    EPWM_forceTripZoneEvent(IRIS_EPWM1_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(IRIS_EPWM2_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(IRIS_EPWM3_BASE, EPWM_TZ_FORCE_EVENT_OST);

    GPIO_WritePin(IRIS_LED2, 1);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
