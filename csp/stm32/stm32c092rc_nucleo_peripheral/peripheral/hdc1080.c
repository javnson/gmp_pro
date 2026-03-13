/**
 * @file    hdc1080.c
 * @brief   Hardware-agnostic driver implementation for TI HDC1080.
 */

#include <gmp_core.h>
#include <core/dev/sensor/hdc1080.h>

/**
 * @brief Internal helper to perform a blocking delay using system ticks.
 */
static void hdc1080_delay_ms(uint32_t ms)
{
    time_gt start = gmp_base_get_system_tick();
    while (!gmp_base_is_delay_elapsed(start, ms))
    {
        /* Busy wait until the delay elapsed */
    }
}

ec_gt hdc1080_init(hdc1080_dev_t* dev, iic_halt bus, addr16_gt dev_addr, hdc1080_config_reg_t init_cfg)
{
    ec_gt ret;
    uint32_t manuf_id = 0;

    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->bus = bus;
    dev->dev_addr = dev_addr;

    /* Cache the mode directly from the bit-field for later use */
    dev->mode = (hdc1080_mode_et)init_cfg.bits.mode;

    /* 1. Device Probe: Read Manufacturer ID */
    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_MANUFACTURER_ID, 1, &manuf_id, 2,
                               HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;
    if (manuf_id != HDC1080_EXPECTED_MANUF_ID)
        return GMP_EC_NACK;

    /* 2. Configure initialization specific bits */
    /* Force software reset bit to 1 to trigger internal reboot */
    init_cfg.bits.soft_reset = 1;
    /* Ensure reserved bits are strictly 0 as per datasheet */
    init_cfg.bits.reserved_14 = 0;
    init_cfg.bits.reserved_7_0 = 0;

    /* 3. Write Configuration (Directly push the 'all' 16-bit value!) */
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, HDC1080_REG_CONFIGURATION, 1, init_cfg.all, 2,
                                HDC1080_CFG_TIMEOUT);

    /* Wait for soft reset to complete */
    hdc1080_delay_ms(15);

    return ret;
}

ec_gt hdc1080_read_config(hdc1080_dev_t* dev, uint16_t* config_ret)
{
    if ((dev == NULL) || (config_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t raw_val = 0;
    ec_gt ret =
        gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_CONFIGURATION, 1, &raw_val, 2, HDC1080_CFG_TIMEOUT);
    if (ret == GMP_EC_OK)
    {
        *config_ret = (uint16_t)raw_val;
    }
    return ret;
}

ec_gt hdc1080_read_device_ids(hdc1080_dev_t* dev, uint16_t* manuf_id_ret, uint16_t* dev_id_ret)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;
    ec_gt ret = GMP_EC_OK;
    uint32_t val = 0;

    if (manuf_id_ret)
    {
        ret =
            gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_MANUFACTURER_ID, 1, &val, 2, HDC1080_CFG_TIMEOUT);
        if (ret == GMP_EC_OK)
            *manuf_id_ret = (uint16_t)val;
    }

    if (dev_id_ret && (ret == GMP_EC_OK))
    {
        ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_DEVICE_ID, 1, &val, 2, HDC1080_CFG_TIMEOUT);
        if (ret == GMP_EC_OK)
            *dev_id_ret = (uint16_t)val;
    }

    return ret;
}

ec_gt hdc1080_read_serial_id(hdc1080_dev_t* dev, uint64_t* serial_ret)
{
    if ((dev == NULL) || (serial_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t s1 = 0, s2 = 0, s3 = 0;
    ec_gt ret;

    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_SERIAL_ID_1, 1, &s1, 2, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_SERIAL_ID_2, 1, &s2, 2, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, HDC1080_REG_SERIAL_ID_3, 1, &s3, 2, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Combine 3x 16-bit registers into a 48-bit serial ID */
    *serial_ret = (((uint64_t)s1) << 32) | (((uint64_t)s2) << 16) | ((uint64_t)s3);

    return GMP_EC_OK;
}

ec_gt hdc1080_read_temperature(hdc1080_dev_t* dev, float* temp_c_ret)
{
    if ((dev == NULL) || (temp_c_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* 1. Trigger Measurement: Send Pointer Register & STOP */
    ec_gt ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, HDC1080_REG_TEMPERATURE, 1, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Wait for conversion (14-bit takes ~6.35ms, 15ms is safe) */
    hdc1080_delay_ms(15);

    /* 3. Read Result: Read 2 bytes WITHOUT sending pointer address again */
    uint32_t raw_temp = 0;
    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, 0, 0, &raw_temp, 2, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 4. Convert via Datasheet Formula: Temp(C) = (RAW / 2^16) * 165 - 40 */
    *temp_c_ret = (((float)raw_temp / 65536.0f) * 165.0f) - 40.0f;

    return GMP_EC_OK;
}

ec_gt hdc1080_read_humidity(hdc1080_dev_t* dev, float* hum_rh_ret)
{
    if ((dev == NULL) || (hum_rh_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* 1. Trigger Measurement */
    ec_gt ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, HDC1080_REG_HUMIDITY, 1, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Wait for conversion */
    hdc1080_delay_ms(15);

    /* 3. Read Result */
    uint32_t raw_hum = 0;
    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, 0, 0, &raw_hum, 2, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 4. Convert via Datasheet Formula: Hum(%RH) = (RAW / 2^16) * 100 */
    *hum_rh_ret = (((float)raw_hum / 65536.0f) * 100.0f);

    return GMP_EC_OK;
}

ec_gt hdc1080_read_temp_hum_sequence(hdc1080_dev_t* dev, float* temp_c_ret, float* hum_rh_ret)
{
    if ((dev == NULL) || (temp_c_ret == NULL) || (hum_rh_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* Protection: This function only works if hardware is in SEQUENCE mode */
    if (dev->mode != HDC1080_MODE_SEQUENCE)
        return GMP_EC_GENERAL_ERROR;

    /* 1. Trigger Sequence: Writing to 0x00 triggers BOTH Temp and Hum sequentially */
    ec_gt ret = gmp_hal_iic_write_cmd(dev->bus, dev->dev_addr, HDC1080_REG_TEMPERATURE, 1, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Wait for BOTH conversions (6.35ms + 6.5ms = ~13ms, 20ms is very safe) */
    hdc1080_delay_ms(20);

    /* 3. Read 4 continuous bytes (Temp MSB, Temp LSB, Hum MSB, Hum LSB) */
    data_gt buf[4] = {0};
    ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, 0, 0, buf, 4, HDC1080_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    uint32_t raw_temp = (buf[0] << 8) | buf[1];
    uint32_t raw_hum = (buf[2] << 8) | buf[3];

    *temp_c_ret = (((float)raw_temp / 65536.0f) * 165.0f) - 40.0f;
    *hum_rh_ret = (((float)raw_hum / 65536.0f) * 100.0f);

    return GMP_EC_OK;
}
