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

#include <ext/encoder/as5048/as5048a.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// device related functions
// Controller interface
//

extern ptr_adc_channel_t uc;
extern ptr_adc_channel_t uin;
extern ptr_adc_channel_t il;


extern pwm_channel_t pwm_out;

// Functions without controller nano framework.
#ifndef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{

    //
    // deal with all the ADC data
    //
    ctl_step_ptr_adc_channel(&il);
    ctl_step_ptr_adc_channel(&uin);
    ctl_step_ptr_adc_channel(&uc);
}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    //ctl_calc_pwm_tri_channel(&pwm_out);

    //
    // Step I: Run PWM calculate
    //

    //

    //
    // Step II: Write PWM compare register
    //

    // write to compare
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ctl_calc_pwm_channel(&pwm_out, ctl_get_boost_ctrl_modulation(&boost_ctrl)));

    

}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE
void ctl_enable_output()
{
    //        csp_sl_enable_output();
}

// Disable Output
GMP_STATIC_INLINE
void ctl_disable_output()
{
    //        csp_sl_disable_output();
}

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
