/**
 * @file    ht16k33.c
 * @brief   Hardware-agnostic driver implementation for HT16K33 LED matrix / Key scan controller.
 * @note    This file strictly depends on the GMP HAL I2C interface (gmp_hal_iic.h) 
 * and is fully decoupled from any specific MCU platform.
 * @note    Optimized with partial memory refresh and strict bounds checking.
 */

#include <gmp_core.h>
#include <core/dev/display/ht16k33.h>

/**
 * @brief   Initialize the HT16K33 device.
 * Probes the device, starts the oscillator, and applies init settings.
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     bus       I2C hardware handle to attach.
 * @param[in]     dev_addr  7-bit device address (e.g., 0x70).
 * @param[in]     init_cfg  Pointer to the initialization parameters.
 * @param[in]     timeout   Timeout for I2C operations in milliseconds.
 * * @return  ec_gt           GMP_EC_OK on success, GMP_EC_NACK if device is missing, 
 * or other general error codes.
 */
ec_gt ht16k33_init(ht16k33_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const ht16k33_init_t* init_cfg)
{
    ec_gt ret;
    uint32_t i;

    if ((dev == NULL) || (init_cfg == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    dev->bus = bus;
    dev->dev_addr = dev_addr;
    dev->last_key = 0;
    dev->last_trigger = gmp_base_get_system_tick();

    /* Clear only the configured amount of local display RAM */
    for (i = 0; i < HT16K33_CFG_DISP_RAM_SIZE; i++)
    {
        dev->display_ram[i] = 0x00;
    }
    dev->is_dirty = 1;

    /* 1. System Setup (Oscillator ON & Device Probe) */
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, HT16K33_CMD_OSC_ON, 1, HT16K33_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. ROW/INT Set */
    fast_gt rowint_cmd = HT16K33_REG_ROWINT_SET;
    if (init_cfg->int_enable)
    {
        rowint_cmd |= 0x01;
        if (init_cfg->int_act_high)
        {
            rowint_cmd |= 0x02;
        }
    }
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, rowint_cmd, 1, HT16K33_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 3. Brightness Set */
    fast_gt brightness_cmd = HT16K33_REG_BRIGHTNESS | (init_cfg->brightness & 0x0F);
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, brightness_cmd, 1, HT16K33_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 4. Display Setup */
    fast_gt display_cmd = HT16K33_REG_DISPLAY_SETUP | 0x01;
    display_cmd |= ((init_cfg->blink_rate & 0x03) << 1);
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, display_cmd, 1, HT16K33_CFG_TIMEOUT);

    return ret;
}
/**
 * @brief   Flush the local display RAM to the HT16K33 if is_dirty flag is set.
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     timeout   Timeout for I2C operations in milliseconds.
 * * @return  ec_gt           GMP_EC_OK on success or if no update is needed.
 */
ec_gt ht16k33_update_display(ht16k33_dev_t* dev)
{
    ec_gt ret;

    if (dev == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    if (!dev->is_dirty)
    {
        return GMP_EC_OK;
    }

    /* Write ONLY the configured amount of RAM to save I2C bandwidth */
    ret = gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, 0x00, 1, dev->display_ram, HT16K33_CFG_DISP_RAM_SIZE,
                                HT16K33_CFG_TIMEOUT);

    if (ret == GMP_EC_OK)
    {
        dev->is_dirty = 0;
    }

    return ret;
}

/**
 * @brief   Scan and return the current pressed key ID.
 * * @param[in]  dev          Pointer to the device object.
 * @param[out] key_id_ret   Pointer to store the pressed key ID (1~39). Returns 0 if no key is pressed.
 * @param[in]  timeout      Timeout for I2C operations in milliseconds.
 * * @return  ec_gt           GMP_EC_OK on success.
 */
ec_gt ht16k33_read_keys(ht16k33_dev_t* dev, fast_gt* key_id_ret)
{
    if ((dev == NULL) || (key_id_ret == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    /* Dynamically allocate array size based on user configuration */
    data_gt key_data[HT16K33_CFG_KEY_RAM_SIZE] = {0};
    uint32_t byteIdx, bitIdx;
    fast_gt current_key;

    /* Read ONLY the necessary key RAM bytes to save I2C bandwidth */
    ec_gt ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, HT16K33_REG_KEY_DATA_ADDR, 1, key_data,
                                     HT16K33_CFG_KEY_RAM_SIZE, HT16K33_CFG_TIMEOUT);

    if (ret != GMP_EC_OK)
    {
        *key_id_ret = 0;
        return ret;
    }

    *key_id_ret = 0;
    for (byteIdx = 0; byteIdx < HT16K33_CFG_KEY_RAM_SIZE; byteIdx++)
    {
        if (key_data[byteIdx] != 0)
        {
            for (bitIdx = 0; bitIdx < 8; bitIdx++)
            {
                if (key_data[byteIdx] & (1 << bitIdx))
                {
                    uint32_t ksRow = byteIdx / 2;
                    uint32_t kCol = (byteIdx % 2) * 8 + bitIdx;

                    current_key = (fast_gt)((ksRow * 13) + kCol + 1);


                    if(current_key != dev->last_key || gmp_base_get_diff_system_tick(dev->last_trigger) > 120)
                    {
                        dev->last_key = current_key;
                        *key_id_ret = dev->last_key;
                        dev->last_trigger = gmp_base_get_system_tick();
                    }

                    return GMP_EC_OK;
                }
            }
        }
    }

    return GMP_EC_OK;
}
/**
 * @brief   Perform a full-screen display test (Turns on all LEDs).
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     timeout   Timeout for I2C operations in milliseconds.
 * * @return  ec_gt           GMP_EC_OK on success.
 */
ec_gt ht16k33_test_all_leds_on(ht16k33_dev_t* dev)
{
    uint32_t i;

    if (dev == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    /* Fill only the configured active RAM with 0xFF */
    for (i = 0; i < HT16K33_CFG_DISP_RAM_SIZE; i++)
    {
        dev->display_ram[i] = 0xFF;
    }

    dev->is_dirty = 1;

    return ht16k33_update_display(dev);
}
