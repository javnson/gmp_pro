
//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should define your own controller objects,
// and initilize them.
//
// User should implement a ctl loop function, this
// function would be called every main loop.
//
// User should implement a state machine if you are using
// Controller Nanon framework.
//

#include <gmp_core.h>

#include <ctrl_settings.h>

#include "ctl_main.h"

#include <xplt.peripheral.h>

#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#include <ctl/component/digital_power/basic/boost.h>

pid_regular_t current_pid, voltage_pid;
ctrl_gt pwm_out_pu;

// Boost Controller Suite
boost_ctrl_t boost_ctrl;

//
adc_bias_calibrator_t adc_calibrator;
fast_gt flag_enable_adc_calibrator = 0;
fast_gt index_adc_calibrator = 0;

// enable motor running
volatile fast_gt flag_enable_system = 0;

// CTL initialize routine
void ctl_init()
{
    ctl_init_boost_ctrl(
        // Boost controller
        &boost_ctrl,
        // Voltage PID controller
        1.5, 0.02, 0,
        // Current PID controller
        3, 0.01, 0,
        // Controller frequency, Hz
        CONTROLLER_FREQUENCY);

#if (BUILD_LEVEL == 1)
    // Open loop
    ctl_boost_ctrl_openloop_mode(&boost_ctrl);
    ctl_set_boost_ctrl_voltage_openloop(&boost_ctrl, float2ctrl(0.5));
#elif (BUILD_LEVEL == 2)
    // Current Loop
    ctl_boost_ctrl_current_mode(&boost_ctrl);
    ctl_set_boost_ctrl_current(&boost_ctrl, float2ctrl(0.5));
#else
    // Voltage loop
    ctl_boost_ctrl_voltage_mode(&boost_ctrl);
    ctl_set_boost_ctrl_voltage(&boost_ctrl, float2ctrl(0.8));
#endif // BUILD_LEVEL

    ctl_disable_boost_ctrl_output(&boost_ctrl);

    // if in simulation mode, enable system
#if !defined SPECIFY_PC_ENVIRONMENT
    // stop here and wait for user start the motor controller
    while (flag_enable_system == 0)
    {
    }

#endif // SPECIFY_PC_ENVIRONMENT

    ctl_enable_boost_ctrl_output(&boost_ctrl);
    ctl_enable_output();

//#if defined SPECIFY_ENABLE_ADC_CALIBRATE    
//    // Enable ADC calibrate
//    flag_enable_adc_calibrator = 1;
//    index_adc_calibrator = 0;
//
//    // Select ADC calibrate
//    // ctl_disable_pmsm_ctrl_output(&pmsm_ctrl);
//    ctl_enable_adc_calibrator(&adc_calibrator);
//#endif // SPECIFY_ENABLE_ADC_CALIBRATE
}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

uint16_t sgen_out = 0;

void ctl_mainloop(void)
{
    // int spd_target = gmp_base_get_system_tick() / 100 - 4;

    // ctl_set_pmsm_ctrl_speed(&pmsm_ctrl, float2ctrl(0.1) * spd_target - float2ctrl(1.0));

    //
    // Judge if PWM is enabled
    //
    // if (pmsm_ctrl.flag_enable_output)
    //{
    //    ctl_enable_output();
    //}
    // else
    //{
    //    ctl_disable_output();
    //}

    return;
}
