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
// device related functions
// Controller interface
//

extern ptr_adc_channel_t boost_adc_channel[ADC_BOOST_CHANNEL_NUM];

extern pwm_channel_t pwm_out;

// Input Callback
GMP_STATIC_INLINE
void ctl_input_callback(void)
{
    // invoke ADC p.u. routine
    for (size_gt index = 0; index < ADC_BOOST_CHANNEL_NUM; ++index)
        ctl_step_ptr_adc_channel(&boost_adc_channel[index]);
}

// Output Callback
GMP_STATIC_INLINE
void ctl_output_callback(void)
{
    //
    // PWM Channel
    //
    simulink_tx_buffer.pwm_cmp[0] = ctl_calc_pwm_channel(&pwm_out, ctl_get_boost_ctrl_modulation(&boost_ctrl));

    //
    // Monitor
    //
    simulink_tx_buffer.monitor[0] = boost_adc_channel[ADC_RESULT_IL].control_port.value;
    simulink_tx_buffer.monitor[1] = boost_adc_channel[ADC_RESULT_UIN].control_port.value;
    simulink_tx_buffer.monitor[2] = boost_adc_channel[ADC_RESULT_UOUT].control_port.value;

    simulink_tx_buffer.monitor[3] = boost_ctrl.voltage_pid.out;
    simulink_tx_buffer.monitor[4] = boost_ctrl.current_pid.out;

    simulink_tx_buffer.dac[0] = 20;
}

// Enable Motor Controller
// Enable Output
GMP_STATIC_INLINE
void ctl_enable_output()
{
    ctl_enable_boost_ctrl(&boost_ctrl);

    csp_sl_enable_output();

    flag_system_running = 1;
}

// Disable Output
GMP_STATIC_INLINE
void ctl_disable_output()
{
    flag_system_running = 0;
    csp_sl_disable_output();
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
