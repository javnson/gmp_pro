// This is the example of user main.

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"
#include "ctl_main.h"
#include <stdlib.h>

// peripheral
#include <core/dev/display/ht16k33.h>
#include <core/dev/gpio/pca9555.h>
#include <core/dev/sensor/hdc1080.h>

//=================================================================================================
// global variables

at_device_entity_t at_dev;
time_gt uart_last_tick;
gmp_scheduler_t sched;

// devices
iic_halt iic_bus;
ht16k33_dev_t ht16k33;
pca9555_dev_t pca9555;
hdc1080_dev_t hdc1080;

// GPIO
gpio_halt user_led;

void beep_on()
{
    pca9555_set_pin_output(&pca9555, PCA9555_PORT_0, 5, 1);
}

void beep_off()
{
    pca9555_set_pin_output(&pca9555, PCA9555_PORT_0, 5, 0);
}

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

    if(type == AT_CMD_TYPE_SETUP)
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
    {"SPDSET", 6, 0, spdset_handler, "Set speed reference"}
};

//=================================================================================================
// task manager

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    static fast_gt led_stat = 0;
    if(led_stat == 0)
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

void at_device_flush_rx_buffer();
gmp_task_status_t tsk_at_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // AT device dispatch function
    at_device_flush_rx_buffer();
    at_device_dispatch(&at_dev);

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t *dev = (ht16k33_dev_t *) tsk->user_data;

    // TODO: fresh LED buffer here.

    dev->display_ram[0] = 0xFF;

    ec_gt ret = ht16k33_update_display(dev);


    // if meets error, close this task
    if(ret != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
    }

    return GMP_TASK_DONE;
}


gmp_task_status_t tsk_joystick(gmp_task_t* tsk)
{
    pca9555_dev_t *dev = (pca9555_dev_t *) tsk->user_data;



    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_key_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t *dev = (ht16k33_dev_t *) tsk->user_data;
    fast_gt key_id = 0;

    ec_gt ret = ht16k33_read_keys(dev, &key_id);

    // if meets error, close this task
    if(ret != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
    }


    if(key_id != 0)
    {
        // TODO: response key message
        gmp_base_print("Receive Key Message, %d\r\n", key_id);

        if(key_id < 8)
        {
            pca9555_set_pin_output(&pca9555,PCA9555_PORT_0,key_id,1);
            beep_on();
        }
        else if(key_id < 20)
        {
            pca9555_set_pin_output(&pca9555, PCA9555_PORT_1, key_id - 14, 1);
            beep_off();
        }


    }



    return GMP_TASK_DONE;
}


// protect task this function would be implemented in ctl_main.c
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

// All tasks must be non blocking tasks
gmp_task_t tasks[] = {
    // name,     task,      period(ms),  init_phase, is_enabled, pParam
    {"protect", tsk_protect, 1000, 0, 1, NULL},
    {"blink_led", tsk_blink, 1000, 100, 1, NULL},
    {"at_device", tsk_at_device, 5, 1, 0, NULL},
    {"joystick", tsk_joystick, 20, 10, 0, (void*)&pca9555},
    {"flush_key", tsk_key_flush, 100, 10, 0, (void*)&ht16k33},
    {"flush_led", tsk_LED_flush, 500, 200, 0, (void*)&ht16k33}
};



//=================================================================================================
// initialize routine

GMP_NO_OPT_PREFIX
void init(void) GMP_NO_OPT_SUFFIX
{
    int i;

    ht16k33_init_t ht16k33_init_struct =
    {
     .brightness = 15,
     .blink_rate = 0,
     .int_enable = 0,
     .int_act_high = 0
    };

    //ht16k33_init(&ht16k33, iic_bus, HT16K33_DEFAULT_DEV_ADDR, &ht16k33_init_struct);

    pca9555_init_t pca9555_init_struct =
    {
     .cfg_port0 = 0x1F,
     .cfg_port1 = 0x00,
     .out_port0 = 0x00,
     .out_port1 = 0x00,
     .pol_port0 = 0,
     .pol_port1 = 0
    };

    pca9555_init(&pca9555, iic_bus, PCA9555_CALC_ADDR(0,0,0), &pca9555_init_struct);

    //beep_off();

    hdc1080_config_reg_t hdc1080_cfg = {.all = 0};
    hdc1080_cfg.bits.mode = 1; // continuous acquisition data

//    hdc1080_init(&hdc1080, iic_bus, HDC1080_I2C_ADDR_DEFAULT, hdc1080_cfg);

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
