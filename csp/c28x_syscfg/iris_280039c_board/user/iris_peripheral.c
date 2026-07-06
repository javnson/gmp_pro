
// GMP basic core header
#include <gmp_core.h>

// user main header
#include "ctl_main.h"
#include "user_main.h"
#include <stdlib.h>

// peripheral
#include <core/dev/display/ht16k33.h>
#include <core/dev/gpio/pca9555.h>
#include <core/dev/sensor/hdc1080.h>

#include <oled_driver.h>

//=================================================================================================
// BEEP control function



gpio_halt gpio_beep;

void beep_on()
{
    gmp_hal_gpio_write(gpio_beep, 1);
}

void beep_off()
{
    gmp_hal_gpio_write(gpio_beep, 0);
}

//=================================================================================================
// LED control function

// devices
iic_halt iic_bus;
ht16k33_dev_t ht16k33;
hdc1080_dev_t hdc1080;

//
// Common cathode digital tube segment code table
//
const unsigned char led_lut[] = {
    0x3F, // 0  (a,b,c,d,e,f)
    0x06, // 1  (b,c)
    0x5B, // 2  (a,b,d,e,g)
    0x4F, // 3  (a,b,c,d,g)
    0x66, // 4  (b,c,f,g)
    0x6D, // 5  (a,c,d,f,g)
    0x7D, // 6  (a,c,d,e,f,g)
    0x07, // 7  (a,b,c)
    0x7F, // 8  (a,b,c,d,e,f,g)
    0x6F, // 9  (a,b,c,d,f,g)
    0x77, // A  (a,b,c,e,f,g)
    0x7C, // b  (c,d,e,f,g)
    0x39, // C  (a,d,e,f)
    0x5E, // d  (b,c,d,e,g)
    0x79, // E  (a,d,e,f,g)
    0x71, // F  (a,e,f,g)
    0x76, // H  (b,c,e,f,g)
    0x38, // L  (d,e,f)
    0x73, // P  (a,b,e,f,g)
    0x3E, // U  (b,c,d,e,f)
    0x40, // -  (g) - dash
    0x80, // .  (dp) - dot
    0x00  // close all
};

void update_led_content_8byte(ht16k33_dev_t* dev, uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5,
                              uint16_t ch6, uint16_t ch7, uint16_t ch8)
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

gmp_task_status_t tsk_LED_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;

    if(flag_init_cmpt)
    {
    // fresh LED buffer here.
    ec_gt ret = ht16k33_update_display(dev);

    // if meets error, close this task
    if (ret != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
    }
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t tsk_key_flush(gmp_task_t* tsk)
{
    ht16k33_dev_t* dev = (ht16k33_dev_t*)tsk->user_data;
    fast_gt key_id = 0;

    if(flag_init_cmpt)
    {

    ec_gt ret = ht16k33_read_keys(dev, &key_id);

    // if meets error, close this task
    if (ret != GMP_EC_OK)
    {
        tsk->is_enabled = 0;
    }

    if (key_id != 0)
    {
        // response key message
        update_led_content_8byte(dev, led_lut[2], led_lut[0], led_lut[2], led_lut[6], led_lut[20], led_lut[key_id / 10],
                                 led_lut[key_id % 10], led_lut[20]);

        gmp_base_print("Receive Key Message, %d\r\n", key_id);
    }
    }

    return GMP_TASK_DONE;
}

gmp_task_status_t oled_show_task(gmp_task_t* tsk)
{
    static uint16_t index;

    char output_msg[32];

    if (flag_init_cmpt == 1)
    {
        sprintf(output_msg, "index: %d C", index++);
        oled_show_str(0, 2, output_msg);
    }

    return GMP_TASK_DONE;
}

//=================================================================================================
// FPGA control function

gmp_task_status_t fpga_test_task(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    static fast_gt led_stat = 0;
    if (led_stat == 0)
    {
        led_stat = 1;
        SPI_writeReg(0x01, 0x0003);
    }
    else
    {
        led_stat = 0;
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
