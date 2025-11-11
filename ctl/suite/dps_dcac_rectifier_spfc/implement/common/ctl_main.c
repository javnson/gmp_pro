
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


// enable controller
#if !defined SPECIFY_PC_ENVIRONMENT
volatile fast_gt flag_system_enable = 0;
#else
volatile fast_gt flag_system_enable = 1;
#endif // SPECIFY_PC_ENVIRONMENT

volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// Boost Controller Suite
boost_ctrl_t boost_ctrl;

#if BUILD_LEVEL >= 4
mppt_PnO_algo_t mppt;
ctrl_gt mppt_set;
#endif // BUILD_LEVEL

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
        1.5f, 0.02f, 0,
        // Current PID controller
        3.0f, 0.01f, 0,
        // valid voltage output range
        0.1f, 1,
        // input filter cut frequency
        1000.0,
        // Controller frequency, Hz
        CONTROLLER_FREQUENCY);

#if BUILD_LEVEL >= 4

    ctl_init_mppt_PnO_algo(
        // MPPT object handle
        &mppt,
        // initial voltage reference
        0.8f,
        // scope of search
        0.95f, 0.4f,
        // Range of search speed
        0.05f, 0.001f,
        // Convergence time constant of search speed when reaching steady state
        1.0f,
        // MPPT algorithm divider
        20.0f,
        // ISR frequency
        CONTROLLER_FREQUENCY);

#endif

#if (BUILD_LEVEL == 1)
    // Open loop
    ctl_boost_ctrl_openloop_mode(&boost_ctrl);
    ctl_set_boost_ctrl_voltage_openloop(&boost_ctrl, float2ctrl(0.5));

#elif (BUILD_LEVEL == 2)
    // Current Loop
    ctl_boost_ctrl_current_mode(&boost_ctrl);
    ctl_set_boost_ctrl_current(&boost_ctrl, float2ctrl(0.3));

#elif (BUILD_LEVEL == 3)
    // Voltage loop
    ctl_boost_ctrl_voltage_mode(&boost_ctrl);
    ctl_set_boost_ctrl_voltage(&boost_ctrl, float2ctrl(0.5));

#elif (BUILD_LEVEL == 4)
    // MPPT open loop
    ctl_boost_ctrl_openloop_mode(&boost_ctrl);
    ctl_set_boost_ctrl_voltage_openloop(&boost_ctrl, float2ctrl(0.5));

    ctl_disable_mppt_PnO_algo(&mppt);

#elif (BUILD_LEVEL == 5)
    // Current loop MPPT

#endif // BUILD_LEVEL

    ctl_disable_output();

    // if in simulation mode, enable system automatically
#if !defined SPECIFY_PC_ENVIRONMENT

    while (flag_system_enable == 0)
    {
    }

#endif // SPECIFY_PC_ENVIRONMENT
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

#if BUILD_LEVEL >= 4

        if (gmp_base_get_system_tick() - tick_bias > 300)
        {
            ctl_enable_mppt_PnO_algo(&mppt);
        }

#endif // BUILD_LEVEL
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
    ctl_clear_boost_ctrl(&boost_ctrl);

    return 1;
}
