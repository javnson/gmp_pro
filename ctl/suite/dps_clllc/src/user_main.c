//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User main implementation for the bidirectional CLLLC/DAB converter.
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
#include <ctl/component/dsa/dsa_dl_scope.h>

/** @brief Flush received Data Link bytes from the platform transport. */
void flush_dl_rx_buffer(void);

/** @brief Flush pending Data Link bytes to the platform transport. */
void flush_dl_tx_buffer(void);

//=================================================================================================
// Datalink protocol online Debug module

gmp_datalink_t dl;
gmp_dl_facility_t legacy_echo_facility;
volatile uint32_t dl_facility_init_errors;

//
// Tunable Dictionary
//
const gmp_param_item_t dict_m1[] = {
    // CiA 402 State Machine & Protection
    {&cia402_sm.current_cmd, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW},
    {&cia402_sm.current_state, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO},

    {&g_v_out_ref_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
    {&g_i_limit_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
//    {&g_modulation_target_user, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW},
    {&g_modulation_command, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_v_primary.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_i_primary.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_v_secondary.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_i_secondary.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&adc_i_resonant.control_port.value, GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},

};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);
gmp_param_tunable_t tunable;

//
// Memory perspective Dictionary (Mapped for SINV)
//
const gmp_mem_region_t mem_regions[] = {{NULL, 0, GMP_MEM_PERM_RO}};
const uint16_t mem_regions_count = 0;
gmp_mem_persp_t mem_persp_server;

//
// Scope
//
#if defined GMP_DL_SCOPE_STORAGE_RAMGS2_3
#pragma DATA_SECTION(user_dl_scope_storage, "mass_data")
#endif
ctrl_gt user_dl_scope_storage[
    CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(CTL_DSA_DL_SCOPE_CHANNELS, GMP_DL_SCOPE_DEPTH)];
ctl_dsa_dl_scope_t user_dl_scope;

//
// PIL (processor in loop module)
//
#if defined ENABLE_GMP_DL_PIL_SIM
gmp_pil_sim_t pil;
#endif

/** @brief Initialize Data Link facilities in their published command order. */
void init_dl_modules(void)
{
    gmp_dev_dl_init(&dl);
    dl_facility_init_errors = 0U;

    gmp_dev_dl_init_echo_alias(&legacy_echo_facility, 0x99U);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &legacy_echo_facility) ? 0U : 1U;

    gmp_param_tunable_init(&tunable, &dl, 0x30, dict_m1, var_tunable_count);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &tunable.facility) ? 0U : 1U;

    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &mem_persp_server.facility) ? 0U : 1U;

    ctl_init_dsa_dl_scope_workspace(
        &user_dl_scope, &dl, CTL_DSA_DL_SCOPE_DEFAULT_CMD, "Control Scope",
        user_dl_scope_storage,
        (uint32_t)(sizeof(user_dl_scope_storage) / sizeof(user_dl_scope_storage[0])),
        CTL_DSA_DL_SCOPE_CHANNELS, (uint32_t)(CONTROLLER_FREQUENCY));
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, ctl_dsa_dl_scope_facility(&user_dl_scope)) ? 0U : 1U;

#if defined ENABLE_GMP_DL_PIL_SIM
    gmp_pil_sim_init(&pil, &dl, GMP_PIL_DL_BASE_COMMAND);
    gmp_pil_sim_set_masks(&pil, GMP_PIL_TX_MASK, GMP_PIL_RX_MASK);
    dl_facility_init_errors += gmp_dev_dl_append_facility(
        &dl, &pil.facility) ? 0U : 1U;
#endif
}

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

    gmp_base_print(TEXT_STRING("CLLLC controller alive.\r\n"));

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
    /* Toggle every 500 ms: one complete SYS_LED on/off period is 1 s. */
    {"blink_led", tsk_blink, 500, 0, 1, NULL},
    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL},
    {"monitor_data", tsk_monitor, 5, 0, 1, NULL},  // 5ms -> 200Hz refresh rate
    {"ctl_mainloop", tsk_ctl_main, 1, 0, 1, NULL}, // 1ms state machine tick
    {"slow_protect", tsk_protect, 10, 0, 1, NULL}, // 10ms thermal/RMS protection
    /* Run immediately after the 1 kHz scheduler starts.  The CiA402 state
       machine still enforces its configured transition delays. */
    {"startup", tsk_startup, 1, 0, 1, NULL},
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

    init_dl_modules();
}

// Initialization tasks after all peripherals have been initialized
gmp_task_status_t tsk_startup(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    /* Use the normal CiA402 transition sequence; no external command is
       required for the first commissioning build. */
    cia402_sm.flag_enable_control_word = 0;
    cia402_sm.current_cmd = CIA402_CMD_ENABLE_OPERATION;

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
