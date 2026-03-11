/**
 * @file    bh1750.c
 * @brief   Hardware-agnostic driver implementation for ROHM BH1750FVI.
 */

#include <gmp_core.h>

#include <core/dev/sensor/bh1750.h>


ec_gt bh1750_init(bh1750_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const bh1750_init_t* init_cfg)
{
    ec_gt ret;

    if ((dev == NULL) || (init_cfg == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    dev->bus = bus;
    dev->dev_addr = dev_addr;

    /* Cache configurations */
    dev->mode = init_cfg->mode;
    dev->mt_reg = init_cfg->mt_reg;
    if (dev->mt_reg < 31)
        dev->mt_reg = 31;
    if (dev->mt_reg > 254)
        dev->mt_reg = 254;

    /* 1. Device Probe & Wake Up: Send Power ON command */
    /* This will return GMP_EC_NACK if device is not connected */
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, BH1750_CMD_POWER_ON, 1, BH1750_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Reset Data Register (Only valid when Power ON) */
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, BH1750_CMD_RESET, 1, BH1750_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 3. Apply Measurement Time (MT) if different from default */
    if (dev->mt_reg != BH1750_DEFAULT_MT)
    {
        ret = bh1750_set_measurement_time(dev, dev->mt_reg);
        if (ret != GMP_EC_OK)
            return ret;
    }

    /* 4. Trigger the configured Mode */
    ret = bh1750_set_mode(dev, dev->mode);

    return ret;
}

ec_gt bh1750_set_mode(bh1750_dev_t* dev, bh1750_mode_et mode)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->mode = mode;

    /* Send the mode command. 
     * Note: For One-Time mode, this command triggers the measurement.
     * The device will automatically power down after the measurement is done.
     */
    return gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, (uint32_t)mode, 1, BH1750_CFG_TIMEOUT);
}

ec_gt bh1750_set_measurement_time(bh1750_dev_t* dev, uint8_t mt_val)
{
    ec_gt ret;

    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* MT valid range according to datasheet: 31 to 254 */
    if (mt_val < 31)
        mt_val = 31;
    if (mt_val > 254)
        mt_val = 254;

    dev->mt_reg = mt_val;

    /* 1. Send High 3 bits of MT */
    uint8_t high_cmd = BH1750_CMD_MT_HIGH_BIT_MASK | (mt_val >> 5);
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, high_cmd, 1, BH1750_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Send Low 5 bits of MT */
    uint8_t low_cmd = BH1750_CMD_MT_LOW_BIT_MASK | (mt_val & 0x1F);
    ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, low_cmd, 1, BH1750_CFG_TIMEOUT);

    return ret;
}

ec_gt bh1750_power_down(bh1750_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;
    return gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, BH1750_CMD_POWER_DOWN, 1, BH1750_CFG_TIMEOUT);
}

ec_gt bh1750_read_lux(bh1750_dev_t* dev, float* lux_ret)
{
    if ((dev == NULL) || (lux_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    data_gt buf[2] = {0};

    /* * Read 2 bytes from the sensor directly.
     * Note: addr_len is set to 0. We don't send any register pointer.
     */
    ec_gt ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, 0, 0, buf, 2, BH1750_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
    {
        *lux_ret = 0.0f;
        return ret;
    }

    /* Assemble raw 16-bit data */
    uint32_t raw_data = (buf[0] << 8) | buf[1];

    /* * Standard Datasheet Calculation:
     * Lux = (Raw Data / 1.2) * (69 / MT_reg)
     */
    float lux = ((float)raw_data / 1.2f) * (69.0f / (float)dev->mt_reg);

    /* * If High-Resolution Mode 2 is used, the physical resolution is 0.5 lx,
     * so the result must be divided by 2.
     */
    if ((dev->mode == BH1750_MODE_CONT_H_RES2) || (dev->mode == BH1750_MODE_ONCE_H_RES2))
    {
        lux = lux / 2.0f;
    }

    *lux_ret = lux;

    return GMP_EC_OK;
}
