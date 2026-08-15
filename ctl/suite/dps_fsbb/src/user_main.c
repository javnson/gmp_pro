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

#include <core/dev/datalink/mem_presp.h>
#include <core/dev/datalink/pil_core.h>
#include <core/dev/datalink/tunable.h>
#if !defined SPECIFY_PC_ENVIRONMENT
#include <ctl/component/dsa/dsa_dl_scope.h>
#endif

/** @brief Flush received Data Link bytes from the platform transport. */
void flush_dl_rx_buffer(void);

/** @brief Flush pending Data Link bytes to the platform transport. */
void flush_dl_tx_buffer(void);

//=================================================================================================
// Datalink protocol online Debug module

gmp_datalink_t dl;
gmp_dl_facility_t legacy_echo_facility;
volatile uint32_t dl_facility_init_errors;
#if !defined SPECIFY_PC_ENVIRONMENT
CTL_DSA_DL_SCOPE_DEFINE_USER("Control Scope")
#endif

//
// PIL (processor in loop module)
//
#if defined ENABLE_GMP_DL_PIL_SIM
gmp_pil_sim_t pil;
#endif

//
// Tunable Dictionary (Mapped for SINV)
//
const gmp_param_item_t dict_m1[] = {
    // CiA 402 state and user commands
    {&cia402_sm.current_cmd, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW},
    {&cia402_sm.current_state, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO},
    {&g_v_out_ref_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
    {&g_i_limit_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},

    // Feedback and controller state
    {&adc_v_in.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_v_out.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_i_L.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
#if defined FSBB_ENABLE_IOUT_SAMPLE
    {&adc_i_load.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
#endif
    {&dcdc_core.v_target, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&dcdc_core.i_target, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&dcdc_core.v_out_formal, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&v_req, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {(void*)&g_fsbb_faults, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO},
};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);
gmp_param_tunable_t tunable;

//
// Memory perspective Dictionary (Mapped for SINV)
//
const gmp_mem_region_t mem_regions[] = {
    // ISO C does not permit a zero-length array. Keep one inert element while
    // explicitly publishing zero active regions.
    {NULL, 0, GMP_MEM_PERM_RO},
};
const uint16_t mem_regions_count = 0;
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
        (void)gmp_dev_dl_dispatch_rx(&dl);
        break;
    }

#endif // SPECIFY_PC_ENVIRONMENT

    return GMP_TASK_DONE;
}

gmp_scheduler_t sched;

//=================================================================================================
// task manager

// GPIO
gpio_halt user_led;

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    gmp_base_print(TEXT_STRING("FSBB controller online.\r\n"));

    if (g_fsbb_faults != FSBB_FAULT_NONE)
    {
        gmp_hal_gpio_write(user_led, 0);
        return GMP_TASK_DONE;
    }

    if (g_fsbb_output_enabled)
    {
        gmp_hal_gpio_write(user_led, 0);
        return GMP_TASK_DONE;
    }

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
    {"blink_led", tsk_blink, 1000, 0, 1, NULL},
    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL},
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
    {
#if defined ENABLE_GMP_DL_PIL_SIM
        if (tasks[i].handler == tsk_dl_debug_device)
            tasks[i].is_enabled = 0;
#endif
        gmp_scheduler_add_task(&sched, &tasks[i]);
    }

    // init datalink protocol
    gmp_dev_dl_init(&dl);
    dl_facility_init_errors = 0U;
    gmp_dev_dl_init_echo_alias(&legacy_echo_facility, 0x99U);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &legacy_echo_facility) ? 0U : 1U;

    // Enable PIL only when selected by the SDPE project setting.
#if defined ENABLE_GMP_DL_PIL_SIM
    gmp_pil_sim_init(&pil, &dl, GMP_PIL_DL_BASE_COMMAND);
    gmp_pil_sim_set_masks(&pil, GMP_PIL_TX_MASK, GMP_PIL_RX_MASK);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &pil.facility) ? 0U : 1U;
#endif

    // Band DL module with tunable and persp module.
    gmp_param_tunable_init(&tunable, &dl, 0x30, dict_m1, var_tunable_count);
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &tunable.facility) ? 0U : 1U;
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &mem_persp_server.facility) ? 0U : 1U;
#if !defined SPECIFY_PC_ENVIRONMENT
    user_init_dl_scope(&dl);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, user_dl_scope_facility()) ? 0U : 1U;
#endif

#if defined SPECIFY_PC_ENVIRONMENT
    // The SIL target has no external CiA402 master. Use the normal sequenced
    // transitions and request operation immediately after initialization.
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;
#endif
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
#if defined ENABLE_GMP_DL_PIL_SIM
    (void)tsk_dl_debug_device(NULL);
#endif
    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}
