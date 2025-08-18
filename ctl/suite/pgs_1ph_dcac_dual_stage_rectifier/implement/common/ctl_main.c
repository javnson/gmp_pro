
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

#include <ctl/component/digital_power/single_phase/single_phase_dc_ac.h>

#include<ctl/component/digital_power/basic/buck.h>


sinv_ctrl_t sinv_ctrl;
ctrl_gt V_set = float2ctrl(0.36f);
ctl_pid_t current_outer;
ctrl_gt ig_rms_ref = float2ctrl(0.1);


//
// buck controller
buck_ctrl_t buck_ctrl;
//

// enable motor running
#if !defined SPECIFY_PC_ENVIRONMENT
volatile fast_gt flag_system_enable = 0;
#else
volatile fast_gt flag_system_enable = 1;
#endif // SPECIFY_PC_ENVIRONMENT

volatile fast_gt flag_system_running = 0;

volatile fast_gt flag_error = 0;

// CTL initialize routine
void ctl_init()
{
    // stop here and wait for user start the motor controller
    ctl_disable_output();

    sinv_init_t init;

    init.base_freq = 50.0f;

#if BUILD_LEVEL >= 8
    // AC Voltage PID
    init.v_ctrl_kp = 0.7f;
    init.v_ctrl_Ti = 0.01f;
    init.v_ctrl_Td = 0;

#else
    // voltage controller parameters
    init.v_ctrl_kp = 0.8f;
    init.v_ctrl_Ti = 0.02f;
    init.v_ctrl_Td = 0;
#endif // BUILD_LEVEL

    // PLL parameters
    init.pll_ctrl_kp = 0.75f / 20;
    init.pll_ctrl_Ti = 1000.0f;
    init.pll_ctrl_cut_freq = 9.0f;

#if BUILD_LEVEL <= 3
    // current controller parameters
    init.i_ctrl_kp = 0.02f;
    init.i_ctrl_kr = 340.0f;
    init.i_ctrl_cut_freq = 8.0f;
#elif BUILD_LEVEL <= 5
    // current controller parameters
    init.i_ctrl_kp = 1.0e-5f;
    init.i_ctrl_kr = 80.0f;
    init.i_ctrl_cut_freq = 8.0f;
#elif BUILD_LEVEL <= 7
    // current controller parameters
    init.i_ctrl_kp = 1.0e-5f;
    init.i_ctrl_kr = 80.0f;
    init.i_ctrl_cut_freq = 8.0f;
#elif BUILD_LEVEL <= 9
    // current controller parameters
    init.i_ctrl_kp = 1.0e-5f;
    init.i_ctrl_kr = 80.0f;
    init.i_ctrl_cut_freq = 8.0f;
#elif BUILD_LEVEL == 10
    // current controller parameters
    init.i_ctrl_kp = 1.0e-5f;
    init.i_ctrl_kr = 80.0f;
    init.i_ctrl_cut_freq = 8.0f;

#endif // BUILD_LEVEL

    // harmonic controller parameters
    init.harm_ctrl_kr_3 = 1;
    init.harm_ctrl_cut_freq_3 = 5.0f;
    init.harm_ctrl_kr_5 = 3;
    init.harm_ctrl_cut_freq_5 = 5.0f;
    init.harm_ctrl_kr_7 = 3;
    init.harm_ctrl_cut_freq_7 = 5.0f;
    init.harm_ctrl_kr_9 = 6;
    init.harm_ctrl_cut_freq_9 = 5.0f;

    // adc input filter cut frequency
    init.adc_filter_fc = 1000.0;

    // controller frequency
    init.f_ctrl = CONTROLLER_FREQUENCY;

    // init sinv Controller
    ctl_init_sinv_ctrl(&sinv_ctrl, &init);

#if BUILD_LEVEL == 10
    ctl_init_pid_ser(&current_outer, 0.7, 0.1, 0, CONTROLLER_FREQUENCY);
#endif // BUILD_LEVEL




    // buck_initialization
    // current tend to have a lower kp.
    // if output voltage has high frequency noise select less current kp.
    ctl_init_buck_ctrl(
        // Buck controller
        &buck_ctrl,
        // Voltage PID controller parameters
        1.0f, 0.00005f, 0,
        // Current PID controller parameters
        0.7f, 10.0f, 0,
        // valid uin range
        0.1f, 1,
        // Controller frequency, Hz
        1000,
        CONTROLLER_FREQUENCY);


#if BUILD_LEVEL == 1
    // Voltage open loop, inverter
    ctl_set_sinv_openloop_inverter(&sinv_ctrl);
    ctl_set_sinv_current_ref(&sinv_ctrl, float2ctrl(0.6));
    ctl_set_sinv_freerun(&sinv_ctrl);
    ctl_disable_sinv_harm_ctrl(&sinv_ctrl);

#elif BUILD_LEVEL == 2
    // current close loop, inverter
    ctl_set_sinv_current_closeloop_inverter(&sinv_ctrl);
    ctl_set_sinv_current_ref(&sinv_ctrl, float2ctrl(0.1));
    ctl_set_sinv_freerun(&sinv_ctrl);
    ctl_disable_sinv_harm_ctrl(&sinv_ctrl);

#elif BUILD_LEVEL == 3

    // current close loop, with harm control
    ctl_set_sinv_current_closeloop_inverter(&sinv_ctrl);
    ctl_set_sinv_current_ref(&sinv_ctrl, float2ctrl(0.1));
    ctl_set_sinv_freerun(&sinv_ctrl);
    ctl_enable_sinv_harm_ctrl(&sinv_ctrl);

#elif BUILD_LEVEL == 4

    // rectifier, current loop
    ctl_set_sinv_current_closeloop_inverter(&sinv_ctrl);
    ctl_set_sinv_current_ref(&sinv_ctrl, float2ctrl(-0.1));
    ctl_set_sinv_pll(&sinv_ctrl);
    ctl_disable_sinv_harm_ctrl(&sinv_ctrl);

#elif BUILD_LEVEL == 5

    // rectifier, current loop, with harm control
    ctl_set_sinv_current_closeloop_inverter(&sinv_ctrl);
    ctl_set_sinv_current_ref(&sinv_ctrl, float2ctrl(-0.1));
    ctl_set_sinv_pll(&sinv_ctrl);
    ctl_enable_sinv_harm_ctrl(&sinv_ctrl);

#elif BUILD_LEVEL == 6

    // rectifier voltage loop, without harm control
    ctl_set_sinv_voltage_closeloop_rectifier(&sinv_ctrl);
    ctl_set_sinv_voltage_ref(&sinv_ctrl, float2ctrl(0.4));
    ctl_disable_sinv_harm_ctrl(&sinv_ctrl);
    ctl_set_sinv_pll(&sinv_ctrl);

#elif BUILD_LEVEL == 7
    // rectifier voltage loop, with harm control
    ctl_set_sinv_voltage_closeloop_rectifier(&sinv_ctrl);
    ctl_set_sinv_voltage_ref(&sinv_ctrl, float2ctrl(0.4));
    ctl_enable_sinv_harm_ctrl(&sinv_ctrl);
    ctl_set_sinv_pll(&sinv_ctrl);

#elif BUILD_LEVEL == 8
    // inverter voltage loop, without harm control
    ctl_set_sinv_voltage_closeloop_inverter(&sinv_ctrl);
    ctl_set_sinv_voltage_ref(&sinv_ctrl, float2ctrl(0.1));
    ctl_disable_sinv_harm_ctrl(&sinv_ctrl);
    ctl_set_sinv_freerun(&sinv_ctrl);

#elif BUILD_LEVEL == 9
    // inverter voltage loop, with harm control
    ctl_set_sinv_voltage_closeloop_inverter(&sinv_ctrl);
    ctl_set_sinv_voltage_ref(&sinv_ctrl, float2ctrl(0.4));
    ctl_enable_sinv_harm_ctrl(&sinv_ctrl);
    ctl_set_sinv_freerun(&sinv_ctrl);

#elif BUILD_LEVEL == 10

#endif // BUILD_LEVEL

    
    // buck controller settings
    
    ctl_buck_ctrl_voltage_mode(&buck_ctrl);
   // ctl_set_buck_uo(&buck_ctrl, float2ctrl(0.2f));
    
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

        // a delay of 500ms
        if ((started_flag == 0) && ((gmp_base_get_system_tick() - tick_bias) > 500) && (startup_flag == 0))
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

    ctl_clear_sinv(&sinv_ctrl);

    if (ctl_is_sinv_inverter_mode(&sinv_ctrl))
        return 1;
    else

        return (ctl_abs(ctl_get_spll_error_fbk(&sinv_ctrl.spll)) < CTRL_SPLL_EPSILON);
}
