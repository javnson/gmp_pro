/**
 * @file user_main.c
 * @brief ACIM suite background scheduler, Data Link, Scope and PIL services.
 */

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "ctl_main.h"
#include "user_main.h"
#include <stdlib.h>

#include <core/dev/datalink/mem_presp.h>
#if defined ENABLE_GMP_DL_PIL_SIM
#include <core/dev/datalink/pil_core.h>
#endif
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
#if !defined SPECIFY_PC_ENVIRONMENT
#if defined GMP_DL_SCOPE_STORAGE_RAMGS2_3
#pragma DATA_SECTION(user_dl_scope_buffer, "ramgs2_3")
#pragma DATA_SECTION(user_dl_scope_history, "ramgs2_3")
#endif
CTL_DSA_DL_SCOPE_DEFINE_USER("Control Scope")
#endif

/** @brief Processor-in-the-Loop service enabled by the target SDPE switch. */
#if defined ENABLE_GMP_DL_PIL_SIM
gmp_pil_sim_t pil;
#endif

//
// Tunable Dictionary
//
const gmp_param_item_t dict_m1[] = {
    // address,                         type,               permission,        name
    {&cia402_sm.current_cmd,            GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW, NULL},
    {&cia402_sm.current_state,          GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO, NULL},
    {&protection.error_code,            GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.udc,                     GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.idq_ref.dat[phase_d],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.idq_ref.dat[phase_q],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.idq0.dat[phase_d],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.idq0.dat[phase_q],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.iuvw.dat[phase_U],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.iuvw.dat[phase_V],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.iuvw.dat[phase_W],       GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&mtr_ctrl.vdq_ref.dat[phase_d],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mtr_ctrl.vdq_ref.dat[phase_q],    GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&mech_ctrl.target_velocity,        GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RW, NULL},
    {&spd_enc.encif.speed,              GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&acim_sync_speed_pu,                GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&acim_fo.spd_out.speed,             GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&acim_fo.sync_spd_out.speed,        GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
    {&acim_handover.id_ref_out,          GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO, NULL},
};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);
gmp_param_tunable_t tunable;

//
// Memory perspective Dictionary
//
const gmp_mem_region_t mem_regions[] = {
    // base address, byte length, permission, name
    {.base_addr = &mtr_ctrl,
     .byte_length = sizeof(mtr_ctrl) * GMP_PORT_DATA_SIZE_PER_BYTES,
     .perm = GMP_MEM_PERM_RW,
     .name = NULL},
    {.base_addr = &mech_ctrl,
     .byte_length = sizeof(mech_ctrl) * GMP_PORT_DATA_SIZE_PER_BYTES,
     .perm = GMP_MEM_PERM_RW,
     .name = NULL},
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
    //
    // if TX data is ready, do transmit
    //
    case GMP_DL_EVENT_TX_RDY:

        // send tx buffer message
        flush_dl_tx_buffer();

        // ack TX state machine.
        gmp_dev_dl_tx_state_done(&dl);
        break;

    case GMP_DL_EVENT_RX_OK:

        /** Dispatch PIL commands only in a target explicitly built for PIL. */
#if defined ENABLE_GMP_DL_PIL_SIM
        if (gmp_pil_sim_rx_cb(&pil))
            break;
#endif

        //
        // Ack parameter tunable message
        //
        if (gmp_param_tunable_rx_cb(&tunable))
            break;

        //
        // Ack memory perspective message
        //
        if (gmp_mem_persp_rx_cb(&mem_persp_server))
            break;

        /** Dispatch the independent four-channel Scope service. */
        if (user_dispatch_dl_scope())
            break;

        //
        // Echo Command
        //
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
gpio_halt user_led;

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

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

void send_monitor_data(void);
gmp_task_status_t tsk_monitor(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    send_monitor_data();

    return GMP_TASK_DONE;
}

//
// Non-blocking task scheduler
//
gmp_scheduler_t sched;

// All tasks must be non blocking tasks
gmp_task_t tasks[] = {
    // name,     task,      period(ms),  init_phase, is_enabled, pParam
    {"blink_led", tsk_blink, 1000, 0, 1, NULL},
    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL},
    {"monitor_data", tsk_monitor, 2, 0, 1, NULL},
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
        /** Disable the scheduled DL task because PIL services it in every background loop. */
        if (tasks[i].handler == tsk_dl_debug_device)
            tasks[i].is_enabled = 0;
#endif

        gmp_scheduler_add_task(&sched, &tasks[i]);
    }

    // init datalink protocol
    gmp_dev_dl_init(&dl);

    /** Bind PIL only when the independent target SDPE switch is enabled. */
#if defined ENABLE_GMP_DL_PIL_SIM
    gmp_pil_sim_init(&pil, &dl, GMP_PIL_DL_BASE_COMMAND);
    gmp_pil_sim_set_masks(&pil, GMP_PIL_TX_MASK, GMP_PIL_RX_MASK);
#endif

    // Band DL module with tunable and persp module.
    gmp_param_tunable_init(&tunable, &dl, 0x30, dict_m1, var_tunable_count);
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
#if !defined SPECIFY_PC_ENVIRONMENT
    user_init_dl_scope(&dl);
#endif
}

// Initialization tasks after all peripherals have been initialized
gmp_task_status_t tsk_startup(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    //
    // Add necessary init code here.
    //

    //
    // startup process is complete, close this task
    //
    tsk->is_enabled = 0;

    return GMP_TASK_DONE;
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
#if defined ENABLE_GMP_DL_PIL_SIM
    /** Service the synchronous PIL transport without scheduler-period latency. */
    (void)tsk_dl_debug_device(NULL);
#endif

    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}
