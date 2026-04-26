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
hdc1080_dev_t hdc1080;

// GPIO
gpio_halt user_led;
gpio_halt gpio_beep;

gmp_datalink_t dl;

void beep_on()
{
    gmp_hal_gpio_write(gpio_beep, 1);
}

void beep_off()
{
    gmp_hal_gpio_write(gpio_beep, 0);
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
        //ctl_set_mech_target_velocity(&mech_ctrl, strtof(args, NULL));
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
    {"PWRON",   5, 0, enable_handler, "Enable Controller Operation."},
    {"PWROFF",  6, 0, poweroff_handler, "Power off"},
    {"RST",     3, 0, rst_handler, "Reset Sys"},
    {"SPDSET",  6, 0, spdset_handler, "Set speed reference"}
};

//=================================================================================================
// task manager


// 定义一些全局变量方便在 CCS 的 Expressions 窗口中观察
uint16_t test_write_val = 0;
uint16_t test_read_val = 0;
uint32_t test_pass_count = 0;
uint32_t test_fail_count = 0;

gmp_task_status_t tsk_blink(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

//    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

    static fast_gt led_stat = 0;
    if(led_stat == 0)
    {
        led_stat = 1;
        gmp_hal_gpio_write(user_led, 0);
        SPI_writeReg(0x01, 0x0003);
    }
    else
    {
        led_stat = 0;
        gmp_hal_gpio_write(user_led, 1);
        SPI_writeReg(0x01, 0x0000);
    }

    SPI_writeReg(0x03, 0x00FF);

    // trigger ADC
    SPI_writeReg(0x05, 0x8000);
    SPI_writeReg(0x06, 0xA000);
    SPI_writeReg(0x07, 0xF000);

    uint16_t adc_result;
    adc_result = SPI_readReg(0x08);

    SPI_writeReg(0x04, adc_result);

    return GMP_TASK_DONE;
}

void at_device_flush_rx_buffer();
void flush_dl_rx_buffer();
gmp_task_status_t tsk_at_device(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // AT device dispatch function
//    at_device_flush_rx_buffer();
//    at_device_dispatch(&at_dev);


    flush_dl_rx_buffer();

    gmp_dl_event_t e = gmp_dev_dl_loop_cb(&dl);

    switch(e)
    {
    // if TX data is ready, do transmit
    case GMP_DL_EVENT_TX_RDY:
        // 物理发送逻辑（使用你平台的 UART/DMA 发送函数）
        gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl), gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10); // 发送 Header (含 SOF 和 转义)
        if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
        {
           gmp_hal_uart_write(IRIS_UART_USB_BASE, gmp_dev_dl_get_tx_hw_pld_ptr(&dl), gmp_dev_dl_get_tx_hw_pld_size(&dl), 10); // 发送 Payload 和 P_CRC
        }

        gmp_dev_dl_tx_state_done(&dl); // 释放 TX 状态机
        break;

    case GMP_DL_EVENT_RX_OK:

        if (dl.rx_head.cmd == 0x99)
                    {
                        // 使用刚刚写好的 Builder 连写 API，原封不动地将 payload_buf 中的数据打包回传
                        gmp_dev_dl_tx_request(&dl, dl.rx_head.seq_id, GMP_DL_CMD_ECHO,
                                              dl.expected_payload_len, dl.payload_buf);
                        dl.flag_reply_handled = 1; // 告诉框架我们已经处理了
                    }

        break;

    }


    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t *dev = (ht16k33_dev_t *) tsk->user_data;

    // TODO: fresh LED buffer here.
    ec_gt ret = ht16k33_update_display(dev);


    // if meets error, close this task
    if(ret != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
    }

    return GMP_TASK_DONE;
}

// 共阴极数码管段码表
// 包含: 0-9, A-F, H, L, P, U, -, ., 暗
const unsigned char led_lut[] = {
    0x3F,  // 0  (a,b,c,d,e,f)
    0x06,  // 1  (b,c)
    0x5B,  // 2  (a,b,d,e,g)
    0x4F,  // 3  (a,b,c,d,g)
    0x66,  // 4  (b,c,f,g)
    0x6D,  // 5  (a,c,d,f,g)
    0x7D,  // 6  (a,c,d,e,f,g)
    0x07,  // 7  (a,b,c)
    0x7F,  // 8  (a,b,c,d,e,f,g)
    0x6F,  // 9  (a,b,c,d,f,g)
    0x77,  // A  (a,b,c,e,f,g)
    0x7C,  // b  (c,d,e,f,g)  - 通常用小写b区分数字8
    0x39,  // C  (a,d,e,f)
    0x5E,  // d  (b,c,d,e,g)  - 通常用小写d区分数字0
    0x79,  // E  (a,d,e,f,g)
    0x71,  // F  (a,e,f,g)
    0x76,  // H  (b,c,e,f,g)
    0x38,  // L  (d,e,f)
    0x73,  // P  (a,b,e,f,g)
    0x3E,  // U  (b,c,d,e,f)
    0x40,  // -  (g) - 负号或横杠
    0x80,  // .  (dp) - 小数点
    0x00   // 无显示 (全灭)
};

void update_led_content_8byte(ht16k33_dev_t* dev, uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8)
{
    dev->display_ram[0] = ch1;
    dev->display_ram[2] = ch2;
    dev->display_ram[4] = ch3;
    dev->display_ram[6] = ch4;
    dev->display_ram[8] = ch5;
    dev->display_ram[10] = ch6;
    dev->display_ram[12] = ch7;
    dev->display_ram[14] = ch8;
    dev->is_dirty = 1;
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
        update_led_content_8byte(dev, led_lut[2], led_lut[0], led_lut[2], led_lut[6], led_lut[20], led_lut[key_id/10], led_lut[key_id%10], led_lut[20]);

        gmp_base_print("Receive Key Message, %d\r\n", key_id);
    }

    return GMP_TASK_DONE;
}


gmp_task_status_t tsk_startup(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    static uint16_t beep_counter = 0;

    if(beep_counter == 0)
        beep_on();
    else if(beep_counter == 1)
        beep_off();
    else if (beep_counter == 2)
        beep_on();
    else if (beep_counter == 3)
        beep_off();

    beep_counter += 1;

    // if program is complete,
    // init all the peripherals,
    // and close this routine.
    if(beep_counter >= 4)
    {
        ht16k33_init_t ht16k33_init_struct =
        {
         .brightness = 15,
         .blink_rate = 0,
         .int_enable = 0,
         .int_act_high = 0
        };

        ec_gt ec = ht16k33_init(&ht16k33, iic_bus, HT16K33_DEFAULT_DEV_ADDR, &ht16k33_init_struct);

        if(ec == GMP_EC_OK)
        {
            sched.task_list[2]->is_enabled = 1;
            sched.task_list[3]->is_enabled = 1;
        }

        hdc1080_config_reg_t hdc1080_cfg = {.all = 0};
        hdc1080_cfg.bits.mode = 1; // continuous acquisition data

        //hdc1080_init(&hdc1080, iic_bus, HDC1080_I2C_ADDR_DEFAULT, hdc1080_cfg);

        // startup process is complete.
        tsk->is_enabled = 0;
    }

    return GMP_TASK_DONE;
}


// protect task this function would be implemented in ctl_main.c
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

// All tasks must be non blocking tasks
gmp_task_t tasks[] = {
    // name,     task,      period(ms),  init_phase, is_enabled, pParam
    {"blink_led", tsk_blink, 1000, 100, 1, NULL},
    {"at_device", tsk_at_device, 2, 1, 1, NULL},
    {"flush_key", tsk_key_flush, 100, 10, 0, (void*)&ht16k33},
    {"flush_led", tsk_LED_flush, 500, 200, 0, (void*)&ht16k33},
    {"startup", tsk_startup, 500, 0, 1, NULL}
};



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

    // 1. 初始化数据链路层
    gmp_dev_dl_init(&dl);
}

//=================================================================================================
// endless loop routine

GMP_NO_OPT_PREFIX
void mainloop(void) GMP_NO_OPT_SUFFIX
{
    // run task scheduler
    gmp_scheduler_dispatch(&sched);



}
