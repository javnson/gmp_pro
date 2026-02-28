// This is the example of user main.

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"

//=================================================================================================
// global variables

at_device_entity_t at_dev;
time_gt uart_last_tick;
gmp_scheduler_t sched;

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
    {"PWMON", 4, 0, enable_handler, "Enable Controller Operation."},
    {"PWROFF", 4, 0, poweroff_handler, "Power off"},
    {"RST", 3, 0, rst_handler, "Reset Sys"}};

//=================================================================================================
// task manager

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    return GMP_TASK_DONE;
}

void at_device_flush_rx_buffer();
gmp_task_status_t tsk_at_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // AT device dispatch function
    at_device_flush_rx_buffer();
    at_device_dispatch(&at_dev);

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
    {"monitor_data", tsk_monitor, 2, 0, 1, NULL}};

//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    at_device_init(&at_dev, at_cmds, sizeof(at_cmds) / sizeof(at_device_cmd_t), at_device_error_handler);

    gmp_scheduler_init(&sched);

    for (i = 0; i < sizeof(tasks) / sizeof(gmp_task_t); ++i)
        gmp_scheduler_add_task(&sched, &tasks[i]);
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}
