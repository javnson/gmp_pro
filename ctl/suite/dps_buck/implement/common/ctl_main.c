
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

#include <ctl/component/digital_power/basic/buck.h>

// enable controller
#if !defined SPECIFY_PC_ENVIRONMENT
volatile fast_gt flag_system_enable = 0;
#else
volatile fast_gt flag_system_enable = 1;
#endif // SPECIFY_PC_ENVIRONMENT

volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// Boost Controller Suite
buck_ctrl_t buck_ctrl;

//
adc_bias_calibrator_t adc_calibrator;
fast_gt flag_enable_adc_calibrator = 0;
fast_gt index_adc_calibrator = 0;

// enable motor running
volatile fast_gt flag_enable_system = 0;

// CTL initialize routine
void ctl_init()
{
    // Stage I: Disable output
    ctl_disable_output();

    // Stage II: Init controller
    ctl_init_buck_ctrl(
        // Boost controller
        &buck_ctrl,
        // Voltage PID controller
        1.5f, 0.02f, 0,
        // Current PID controller
        1.5f, 0.01f, 0,
        // valid voltage output range
        0.1f, 1.0f,
        // input filter cut frequency
        1000.0f,
        // Controller frequency, Hz
        CONTROLLER_FREQUENCY);

    // Stage III: Config operating mode
#if (BUILD_LEVEL == 1)
    // Open loop
    ctl_buck_ctrl_openloop_mode(&buck_ctrl);
    ctl_set_buck_ctrl_voltage_openloop(&buck_ctrl, float2ctrl(0.18));

#elif (BUILD_LEVEL == 2)
    // Current Loop
    ctl_buck_ctrl_current_mode(&buck_ctrl);
    ctl_set_buck_ctrl_current(&buck_ctrl, float2ctrl(0.05));

#elif (BUILD_LEVEL == 3)
    // Voltage loop
    ctl_buck_ctrl_voltage_mode(&boost_ctrl);
    ctl_set_buck_ctrl_voltage(&boost_ctrl, float2ctrl(0.1));

#elif (BUILD_LEVEL == 4)
    // Current loop for DC bus

#elif (BUILD_LEVEL == 5)
    // Current loop MPPT

#endif // BUILD_LEVEL

}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

uint16_t sgen_out = 0;

fast_gt firsttime_flag = 0;
fast_gt startup_flag = 0;
fast_gt started_flag = 0;
time_gt tick_bias = 0;

void ctl_mainloop(void)
{
    // When the program is reach here, the following things will happen:
    // 1. software non-block delay 500ms
    // 2. judge if spll theta error convergence has occurred
    // 3. then enable system

    if (flag_system_enable)
    {

        // first time flag
        // log the first time enable the system
        if (firsttime_flag == 0)
        {
            tick_bias = gmp_base_get_system_tick();
            firsttime_flag = 1;
        }

        // a delay of 100ms
        if ((started_flag == 0) && ((gmp_base_get_system_tick() - tick_bias) > 100) && (startup_flag == 0))
        {
            startup_flag = 1;
        }

        // judge if PLL is close to target
        if ((started_flag == 0) && (startup_flag == 1) && ctl_ready_mainloop())
        {
            ctl_enable_output();
            started_flag = 1;
        }
    }
    // if system is disabled
    else // (flag_system_enable == 0)
    {
        ctl_disable_output();

        // clear all flags
        firsttime_flag = 0;
        startup_flag = 0;
        started_flag = 0;
        tick_bias = 0;
    }

    return;
}

// This mainloop will run again and again to judge if system meets online condition,
// when flag_system_enable is set.
// if return 1 the system is ready to enable.
// if return 0 the system is not ready to enable
fast_gt ctl_ready_mainloop(void)
{
    ctl_clear_buck_ctrl(&buck_ctrl);

    return 1;
}
