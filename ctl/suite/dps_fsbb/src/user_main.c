//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User main implementation for Single-Phase Inverter (SINV).
//

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "ctl_main.h"
#include "user_main.h"
#include <stdlib.h>

#include <core/dev/mem_presp.h>
#include <core/dev/pil_core.h>
#include <core/dev/tunable.h>

//=================================================================================================
// Datalink protocol online Debug module

gmp_datalink_t dl;

//
// PIL (processor in loop module)
//
gmp_pil_sim_t pil;

//
// Tunable Dictionary (Mapped for SINV)
//
const gmp_param_item_t dict_m1[] = {
    // CiA 402 State Machine & Protection
    {&cia402_sm.current_cmd, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW},
    {&cia402_sm.current_state, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO},
    {&protection.active_errors, GMP_PARAM_TYPE_U32, GMP_PARAM_PERM_RO},

    // User Setpoints
    {&g_p_ref_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
    {&g_q_ref_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},

    // PLL Observations
    {&pll.v_mag, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&pll.frequency, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},

    // Power Meter Observations
    {&pq_meter.p_avg, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&pq_meter.q_avg, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&pq_meter.v_rms, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&pq_meter.i_rms, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},

    // DC Bus Observation
    {&adc_v_bus.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);
gmp_param_tunable_t tunable;

//
// Memory perspective Dictionary (Mapped for SINV)
//
const gmp_mem_region_t mem_regions[] = {
    {.base_addr = &rc_core, .byte_length = sizeof(rc_core) * GMP_PORT_DATA_SIZE_PER_BYTES, .perm = GMP_MEM_PERM_RW},
    {.base_addr = &pll, .byte_length = sizeof(pll) * GMP_PORT_DATA_SIZE_PER_BYTES, .perm = GMP_MEM_PERM_RW},
};
const uint16_t mem_regions_count = sizeof(mem_regions) / sizeof(mem_regions[0]);
gmp_mem_persp_t mem_persp_server;

//
// Datalink protocol stack task
//
gmp_task_status_t tsk_dl_debug_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    flush_dl_rx_buffer();

    // In PC simulation environment the DL protocol module is disabled.
#ifndef SPECIFY_PC_ENVIRONMENT

    gmp_dl_event_t e = gmp_dev_dl_loop_cb(&dl);

    switch (e)
    {
    // if TX data is ready, do transmit
    case GMP_DL_EVENT_TX_RDY:
        // send tx buffer message
        flush_dl_tx_buffer();
        // ack TX state machine.
        gmp_dev_dl_tx_state_done(&dl);
        break;

    case GMP_DL_EVENT_RX_OK:
        // Ack PIL simulation message
        if (gmp_pil_sim_rx_cb(&pil))
            break;

        // Ack parameter tunable message
        if (gmp_param_tunable_rx_cb(&tunable))
            break;

        // Ack memory perspective message
        if (gmp_mem_persp_rx_cb(&mem_persp_server))
            break;

        // Echo Command
        if (dl.rx_head.cmd == 0x99)
        {
            // echo payload_buf
            gmp_dev_dl_tx_request(&dl, dl.rx_head.seq_id, GMP_DL_CMD_ECHO, dl.expected_payload_len, dl.payload_buf);
            // ack this message
            gmp_dev_dl_msg_handled(&dl);
            break;
        }

        // default handler
        gmp_dev_dl_default_rx_handler(&dl);
        break;
    }

#endif // SPECIFY_PC_ENVIRONMENT

    return GMP_TASK_DONE;
}

gmp_scheduler_t sched;

//=================================================================================================
// task manager

// GPIO
extern gpio_halt user_led;

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    gmp_base_print(TEXT_STRING("Hello SINV World!\r\n"));

    static fast_gt led_stat = 0;
    if (led_stat == 0)
    {
        led_stat = 1;
        gmp_hal_gpio_write(user_led, 0);
    }
    else
    {
        led_stat = 0;
        gmp_hal_gpio_write(user_led, 1);
    }

    return GMP_TASK_DONE;
}

extern void send_monitor_data(void);
gmp_task_status_t tsk_monitor(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    send_monitor_data();

    return GMP_TASK_DONE;
}

// Wrapper task for ctl_main.c state machine dispatch
gmp_task_status_t tsk_ctl_main(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    ctl_mainloop();

    return GMP_TASK_DONE;
}

// External declaration for slow protection task defined in ctl_main.c
extern gmp_task_status_t tsk_protect(gmp_task_t* tsk);
gmp_task_status_t tsk_startup(gmp_task_t* tsk);

//
// Non-blocking task scheduler
//
// All tasks must be non blocking tasks
gmp_task_t tasks[] = {
    // name,          task,                period(ms),  init_phase, is_enabled, pParam
    {"blink_led", tsk_blink, 1000, 0, 1, NULL},    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL},
    {"monitor_data", tsk_monitor, 5, 0, 1, NULL},  // 5ms -> 200Hz refresh rate
    {"ctl_mainloop", tsk_ctl_main, 1, 0, 1, NULL}, // 1ms state machine tick
    {"slow_protect", tsk_protect, 10, 0, 1, NULL}, // 10ms thermal/RMS protection
    {"startup", tsk_startup, 500, 0, 1, NULL},
};

//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    // init scheduler
    gmp_scheduler_init(&sched);

    for (i = 0; i < sizeof(tasks) / sizeof(gmp_task_t); ++i)
        gmp_scheduler_add_task(&sched, &tasks[i]);

    // init datalink protocol
    gmp_dev_dl_init(&dl);

    // enable PIL simulation environment
    gmp_pil_sim_init(&pil, &dl, 0x10);

    // Band DL module with tunable and persp module.
    gmp_param_tunable_init(&tunable, &dl, 0x30, dict_m1, var_tunable_count);
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
}

// Initialization tasks after all peripherals have been initialized
gmp_task_status_t tsk_startup(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // Add necessary init code here.

    // startup process is complete, close this task
    tsk->is_enabled = 0;

    return GMP_TASK_DONE;
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}
