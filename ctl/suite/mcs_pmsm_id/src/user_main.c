// This is the example of user main.

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

//=================================================================================================
// global variables

at_device_entity_t at_dev;
time_gt uart_last_tick;
gmp_scheduler_t sched;

gpio_halt gpio_led;

/** @brief Drain platform UART data into the legacy receive ring buffer. */
void at_device_flush_rx_buffer(void);

#if defined ENABLE_GMP_DL_PIL_SIM
gmp_datalink_t dl;

/** @brief Move bytes collected by the legacy UART ISR into the Data Link parser. */
void flush_dl_rx_buffer(void)
{
    data_gt byte;
    at_device_flush_rx_buffer();
    while (ringbuf_get_one(&at_dev.buffer, &byte))
        gmp_dev_dl_push_byte(&dl, byte);
}

/** @brief Transmit the current Data Link frame through the configured debug UART. */
void flush_dl_tx_buffer(void)
{
    gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl),
                       gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                           gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
    }
}

gmp_pil_sim_t pil;

const gmp_param_item_t dict_m1[] = {
    // address, data type, permission
    {&cia402_sm.current_cmd, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RW},
    {&cia402_sm.current_state, GMP_PARAM_TYPE_U16, GMP_PARAM_PERM_RO},
    {&mtr_ctrl.idq0.dat[phase_d], GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
    {&mtr_ctrl.idq0.dat[phase_q], GMP_PARAM_TYPE_F32, GMP_PARAM_PERM_RO},
};
const uint16_t var_tunable_count = sizeof(dict_m1) / sizeof(dict_m1[0]);
gmp_param_tunable_t tunable;

const gmp_mem_region_t mem_regions[] = {
    // base address, byte length, permission
    {&mtr_ctrl, sizeof(mtr_ctrl) * GMP_PORT_DATA_SIZE_PER_BYTES, GMP_MEM_PERM_RO},
};
const uint16_t mem_regions_count = sizeof(mem_regions) / sizeof(mem_regions[0]);
gmp_mem_persp_t mem_persp_server;

#if !defined SPECIFY_PC_ENVIRONMENT
CTL_DSA_DL_SCOPE_DEFINE_USER("Control Scope")
#endif
#endif // defined ENABLE_GMP_DL_PIL_SIM

//=================================================================================================
// AT command

/* 2.1 Enable asynchronous Handler */
at_status_t enable_handler(at_device_entity_t* dev, at_cmd_type_t type, char* args, uint16_t len)
{
    GMP_UNUSED_VAR(dev);
    GMP_UNUSED_VAR(type);
    GMP_UNUSED_VAR(args);
    GMP_UNUSED_VAR(len);

    gmp_base_print(TEXT_STRING("[WOW] enable handle was called!\r\n"));

    cia402_send_cmd(&cia402_sm, CIA402_CMD_ENABLE_OPERATION);

    return AT_STATUS_OK;
}

/* 2.2 Disable asynchronous Handler */
at_status_t poweroff_handler(at_device_entity_t* dev, at_cmd_type_t type, char* args, uint16_t len)
{
    GMP_UNUSED_VAR(dev);
    GMP_UNUSED_VAR(type);
    GMP_UNUSED_VAR(args);
    GMP_UNUSED_VAR(len);

    gmp_base_print(TEXT_STRING("[WOW] Power OFF handle was called!\r\n"));

    cia402_send_cmd(&cia402_sm, CIA402_CMD_DISABLE_VOLTAGE);

    return AT_STATUS_OK;
}

/* 2.3 Reset asynchronous Handler */
at_status_t rst_handler(at_device_entity_t* dev, at_cmd_type_t type, char* args, uint16_t len)
{
    GMP_UNUSED_VAR(dev);
    GMP_UNUSED_VAR(type);
    GMP_UNUSED_VAR(args);
    GMP_UNUSED_VAR(len);

    gmp_base_print(TEXT_STRING("[WOW] rst_handler, with arg: %s!\r\n"), args);

    cia402_fault_reset(&cia402_sm);

    return AT_STATUS_OK;
}

at_status_t spdset_handler(at_device_entity_t* dev, at_cmd_type_t type, char* args, uint16_t len)
{
    GMP_UNUSED_VAR(dev);
    GMP_UNUSED_VAR(type);
    GMP_UNUSED_VAR(args);
    GMP_UNUSED_VAR(len);

    gmp_base_print(TEXT_STRING("[WOW] spdset_handler, with arg: %s!\r\n"), args);

    if (type == AT_CMD_TYPE_SETUP)
    {
        ctl_set_mech_target_velocity(&mech_ctrl, strtof(args, NULL));
    }

    return AT_STATUS_OK;
}

/* 3. AT device Error Handle */
void at_device_error_handler(at_device_entity_t* dev, at_error_code_t code)
{
    GMP_UNUSED_VAR(dev);

    if (code == AT_ERR_RX_OVERFLOW)
    {
        gmp_base_print("[WOW] System Overload!\r\n");
    }
    else
    {
        gmp_base_print("[WOW] Unknown error happened!\r\n");
    }
}

/*  Command List for AT device (non-const is necessary) */
at_device_cmd_t at_cmds[] = {
    // name,    name_len, attr, handler,      help_info
    {"PWRON", 5, 0, enable_handler, "Enable Controller Operation."},
    {"PWROFF", 6, 0, poweroff_handler, "Power off"},
    {"RST", 3, 0, rst_handler, "Reset Sys"},
    {"SPDSET", 6, 0, spdset_handler, "Set speed reference"}};

//=================================================================================================
// task manager

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    static fast_gt led_level = 0;

    led_level = !led_level;

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    gmp_hal_gpio_write(gpio_led, led_level);

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_at_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // AT device dispatch function
    at_device_flush_rx_buffer();
    at_device_dispatch(&at_dev);

    return GMP_TASK_DONE;
}

/** @brief Service the Data Link stack when the SDPE PIL switch owns the UART. */
gmp_task_status_t tsk_dl_debug_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);
#if defined ENABLE_GMP_DL_PIL_SIM && !defined SPECIFY_PC_ENVIRONMENT
    flush_dl_rx_buffer();
    switch (gmp_dev_dl_loop_cb(&dl))
    {
    case GMP_DL_EVENT_TX_RDY:
        flush_dl_tx_buffer();
        gmp_dev_dl_tx_state_done(&dl);
        break;
    case GMP_DL_EVENT_RX_OK:
        if (gmp_pil_sim_rx_cb(&pil) ||
            gmp_param_tunable_rx_cb(&tunable) ||
            gmp_mem_persp_rx_cb(&mem_persp_server) ||
            user_dispatch_dl_scope())
            break;
        gmp_dev_dl_default_rx_handler(&dl);
        break;
    default:
        break;
    }
#endif
    return GMP_TASK_DONE;
}

void send_monitor_data(void);
gmp_task_status_t tsk_monitor(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    send_monitor_data();

    return GMP_TASK_DONE;
}

// protect task this function would be implemented in ctl_main.c
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

// All tasks must be non blocking tasks
gmp_task_t tasks[] = {
    // name,     task,      period(ms),  init_phase, is_enabled, pParam
    {"protect", tsk_protect, 1000, 0, 1, NULL},
    {"blink_led", tsk_blink, 1000, 100, 1, NULL},
    {"at_device", tsk_at_device, 5, 1, 1, NULL},
    {"dl_online", tsk_dl_debug_device, 2, 0, 1, NULL},
    {"monitor_data", tsk_monitor, 2, 0, 1, NULL}};

//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    at_device_init(&at_dev, at_cmds, sizeof(at_cmds) / sizeof(at_device_cmd_t), at_device_error_handler);

#if defined ENABLE_GMP_DL_PIL_SIM
    gmp_dev_dl_init(&dl);
    gmp_pil_sim_init(&pil, &dl, GMP_PIL_DL_BASE_COMMAND);
    gmp_pil_sim_set_masks(&pil, GMP_PIL_TX_MASK, GMP_PIL_RX_MASK);
    gmp_param_tunable_init(&tunable, &dl, 0x30, dict_m1, var_tunable_count);
    gmp_mem_persp_init(&mem_persp_server, &dl, 0x50, mem_regions, mem_regions_count);
#if !defined SPECIFY_PC_ENVIRONMENT
    user_init_dl_scope(&dl);
#endif
#endif

    gmp_scheduler_init(&sched);

    for (i = 0; i < sizeof(tasks) / sizeof(gmp_task_t); ++i)
    {
#if defined ENABLE_GMP_DL_PIL_SIM
        if (tasks[i].handler == tsk_at_device || tasks[i].handler == tsk_dl_debug_device)
            tasks[i].is_enabled = 0;
#else
        if (tasks[i].handler == tsk_dl_debug_device)
            tasks[i].is_enabled = 0;
#endif
        gmp_scheduler_add_task(&sched, &tasks[i]);
    }
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
