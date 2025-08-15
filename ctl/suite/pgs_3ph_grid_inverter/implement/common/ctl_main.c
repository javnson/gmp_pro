
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

#include <ctl/component/digital_power/three_phase/three_phase_dc_ac.h>

inv_ctrl_t inv_ctrl;

// enable controller
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

    three_phase_inv_init_t init;

    init.v_base = CTRL_VOLTAGE_BASE;
    init.i_base = CTRL_CURRENT_BASE;
    init.freq_base = 50.0f;
    init.Lf = 680e-6f;
    init.fs = CONTROLLER_FREQUENCY;
    init.adc_fc = 1e3f;

    init.kp_id_ctrl = 1.8f;
    init.Ti_id_ctrl = 0.002f;
    init.kp_iq_ctrl = 1.8f;
    init.Ti_iq_ctrl = 0.002f;

    init.kp_vd_ctrl = 0.8f;
    init.Ti_vd_ctrl = 0.005f;
    init.kp_vq_ctrl = 0.8f;
    init.Ti_vq_ctrl = 0.005f;

    init.kp_vdn_ctrl = 0.2f;
    init.Ti_vdn_ctrl = 0.01f;
    init.kp_vqn_ctrl = 0.2f;
    init.Ti_vqn_ctrl = 0.01f;

    init.kp_idn_ctrl = 0.7f;
    init.Ti_idn_ctrl = 0.01f;
    init.kp_iqn_ctrl = 0.7f;
    init.Ti_iqn_ctrl = 0.01f;

    init.kp_pll_ctrl = 0.1f;
    init.Ti_pll_ctrl = 0.001f;

    init.harm_ctrl_kr_5 = 5;
    init.harm_ctrl_cut_freq_5 = 1;
    init.harm_ctrl_kr_7 = 5;
    init.harm_ctrl_cut_freq_7 = 1;

    init.zero_ctrl_kp = 0.001;
    init.zero_ctrl_Ti = 0.01;
    init.zero_ctrl_kr_3 = 1;
    init.zero_ctrl_cut_freq_3 = 1;
    init.zero_ctrl_kr_9 = 1;
    init.zero_ctrl_cut_freq_9 = 1;

    init.kp_droop = 0.001f;
    init.kq_droop = 0.001f;

    init.id_lim_droop = 0.1f;
    init.iq_lim_droop = 0.1f;

    // init sinv Controller
    ctl_init_three_phase_inv(&inv_ctrl, &init);

#if BUILD_LEVEL == 1

    // Voltage open loop, inverter
    ctl_set_three_phase_inv_openloop_mode(&inv_ctrl);
    ctl_set_three_phase_inv_voltage_openloop(&inv_ctrl, float2ctrl(0.6), float2ctrl(0));
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_disable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_disable_three_phase_negative_ctrl(&inv_ctrl);
    ctl_disable_three_phase_feedforware(&inv_ctrl);

#elif BUILD_LEVEL == 2

    // current close loop, inverter
    ctl_set_three_phase_inv_current_mode(&inv_ctrl);
    ctl_set_three_phase_inv_current(&inv_ctrl, 0.025, 0.005);
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_disable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_disable_three_phase_negative_ctrl(&inv_ctrl);
    ctl_disable_three_phase_feedforware(&inv_ctrl);

#elif BUILD_LEVEL == 3

    // current close loop, with feed forward
    ctl_set_three_phase_inv_current_mode(&inv_ctrl);
    ctl_set_three_phase_inv_current(&inv_ctrl, 0.025, 0.005);
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_disable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_disable_three_phase_negative_ctrl(&inv_ctrl);
    ctl_enable_three_phase_feedforware(&inv_ctrl);

#elif BUILD_LEVEL == 4

    // current close loop, with feed forward, negative control
    ctl_set_three_phase_inv_current_mode(&inv_ctrl);
    ctl_set_three_phase_inv_current(&inv_ctrl, 0.025, 0.005);
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_disable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_enable_three_phase_negative_current_ctrl(&inv_ctrl);
    ctl_enable_three_phase_feedforware(&inv_ctrl);

#elif BUILD_LEVEL == 5

    // current close loop, with feed forward, negative control, harm control
    ctl_set_three_phase_inv_current_mode(&inv_ctrl);
    ctl_set_three_phase_inv_current(&inv_ctrl, 0.025, 0.005);
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_enable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_enable_three_phase_negative_current_ctrl(&inv_ctrl);
    ctl_enable_three_phase_feedforware(&inv_ctrl);

#elif BUILD_LEVEL == 6

    // voltage loop, inverter, voltage loop, current loop, ff
    ctl_set_three_phase_inv_voltage_mode(&inv_ctrl);
    ctl_set_three_phase_inv_voltage(&inv_ctrl, 0.12);
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_disable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_disable_three_phase_negative_ctrl(&inv_ctrl);
    ctl_enable_three_phase_feedforware(&inv_ctrl);

#elif BUILD_LEVEL == 7
    // inverter, voltage loop, current loop, ff, negative voltage control
    ctl_set_three_phase_inv_voltage_mode(&inv_ctrl);
    ctl_set_three_phase_inv_voltage(&inv_ctrl, 0.12);
    ctl_set_three_phase_inv_freerun(&inv_ctrl);
    ctl_disable_three_phase_harm_ctrl(&inv_ctrl);
    ctl_enable_three_phase_negative_voltage_ctrl(&inv_ctrl);
    //ctl_enable_three_phase_feedforware(&inv_ctrl);

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

        // a delay of 500ms
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
    ctl_clear_three_phase_inv(&inv_ctrl);

    if (ctl_is_three_phase_inverter_mode(&inv_ctrl))
        return 1;
    else
        return (ctl_abs(ctl_get_three_phase_pll_error(&inv_ctrl)) < CTRL_SPLL_EPSILON);
}
