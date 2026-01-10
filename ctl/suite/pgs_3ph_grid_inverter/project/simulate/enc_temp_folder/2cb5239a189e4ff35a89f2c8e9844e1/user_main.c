// This is the example of user main.

//////////////////////////////////////////////////////////////////////////
// headers here

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"


//////////////////////////////////////////////////////////////////////////
// global variables here

at_device_entity_t at_dev;
time_gt uart_last_tick;
gmp_scheduler_t sched;


//////////////////////////////////////////////////////////////////////////
// AT command list
//

/* 2. 异步 Handler 示例 */
at_status_t wifi_handler(at_device_entity_t *dev, at_cmd_type_t type, char* args, uint16_t len)
{
    gmp_base_print(TEXT_STRING("[WOW] wifi_handle!\r\n"));

    return AT_STATUS_OK;
}

/* 2. 异步 Handler 示例 */
at_status_t mqtt_handler(at_device_entity_t *dev, at_cmd_type_t type, char* args, uint16_t len)
{
    gmp_base_print(TEXT_STRING("[WOW] mqtt_handle!\r\n"));

    return AT_STATUS_OK;
}

/* 2. 异步 Handler 示例 */
at_status_t rst_handler(at_device_entity_t *dev, at_cmd_type_t type, char* args, uint16_t len)
{
    gmp_base_print(TEXT_STRING("[WOW] rst_handler, with arg: %s!\r\n"), args);

    return AT_STATUS_OK;
}


/* 3. 错误处理 */
void at_device_error_handler(at_device_entity_t *dev, at_error_code_t code) {
    if (code == AT_ERR_RX_OVERFLOW)
    {
        gmp_base_print("[WOW] System Overload!\r\n");
    }
    else
    {
        gmp_base_print("[WOW] Unknown error happened!\r\n");
    }
}


/* 1. 定义命令表 (必须非 const，或者位于 RAM) */
at_device_cmd_t at_cmds[] = {
    // name,    name_len, attr, handler,      help_info
    {"MQTT",    4,        0,    mqtt_handler, "MQTT Pub"},
    {"WIFI",    4,        0,    wifi_handler, "WiFi Conn"},
    {"RST",     3,        0,    rst_handler,  "Reset Sys"}
};

//////////////////////////////////////////////////////////////////////////
// task manager

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    return GMP_TASK_DONE;
}

void at_device_flush_rx_buffer();
gmp_task_status_t tsk_at_device(gmp_task_t* tsk)
{
    // AT device dispatch function
    at_device_flush_rx_buffer();
    at_device_dispatch(&at_dev);

    return GMP_TASK_DONE;
}

void send_monitor_data(void);

gmp_task_status_t tsk_monitor(gmp_task_t* tsk)
{
    send_monitor_data();

    return GMP_TASK_DONE;
}

gmp_task_t tasks[] = {
   // name,     task,      period(ms),  init_phase, is_enabled, pParam
   { "blink_led", tsk_blink, 1000, 0, 1, NULL},
   { "at_device", tsk_at_device, 5, 1, 1, NULL},
   { "monitor_data", tsk_monitor, 2, 0, 1, NULL}
};

//////////////////////////////////////////////////////////////////////////
// initialize routine here
GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    at_device_init(&at_dev, at_cmds, sizeof(at_cmds)/sizeof(at_device_cmd_t), at_device_error_handler);

    gmp_scheduler_init(&sched);

    for(i = 0; i<sizeof(tasks)/sizeof(gmp_task_t); ++i)
        gmp_scheduler_add_task(&sched, &tasks[i]);

}

//////////////////////////////////////////////////////////////////////////
// endless loop function here

void at_device_clear_rx_buf();

void mainloop(void)
{
    // run task scheduler
    gmp_scheduler_dispatch(&sched);
}

