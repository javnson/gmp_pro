/**
 * @file    ds1337.c
 * @brief   Hardware-agnostic driver implementation for DS1337 RTC.
 */


#include <gmp_core.h>

#include <core/dev/rtc/ds1337.h>

/* ========================================================================= */
/* ==================== PRIVATE HELPER FUNCTIONS =========================== */
/* ========================================================================= */

/**
 * @brief Convert Decimal to Binary-Coded Decimal (BCD)
 */
GMP_STATIC_INLINE uint8_t dec_to_bcd(uint8_t val)
{
    return (uint8_t)(((val / 10) << 4) | (val % 10));
}

/**
 * @brief Convert Binary-Coded Decimal (BCD) to Decimal
 */
GMP_STATIC_INLINE uint8_t bcd_to_dec(uint8_t val)
{
    return (uint8_t)(((val >> 4) * 10) + (val & 0x0F));
}

/* ========================================================================= */
/* ==================== PUBLIC API IMPLEMENTATION ========================== */
/* ========================================================================= */

ec_gt ds1337_init(ds1337_dev_t* dev, iic_halt bus, const ds1337_init_t* init_cfg)
{
    ec_gt ret;
    uint32_t status_val = 0;

    if ((dev == NULL) || (init_cfg == NULL))
        return GMP_EC_GENERAL_ERROR;

    dev->bus = bus;
    dev->dev_addr = DS1337_I2C_ADDR;

    /* 1. Read Status Register to check OSF (Oscillator Stop Flag) */
    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, DS1337_REG_STATUS, 1, &status_val, 1, DS1337_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* If OSF is 1, the oscillator was stopped (e.g., first power up or battery failure).
     * We must clear it to allow the clock to be deemed reliable.
     */
    if (status_val & DS1337_STATUS_OSF)
    {
        status_val &= ~DS1337_STATUS_OSF;
        gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, DS1337_REG_STATUS, 1, status_val, 1, DS1337_CFG_TIMEOUT);
    }

    /* 2. Configure Control Register (Enable/Disable Osc, Set INT mode) */
    uint32_t ctrl_val = 0;
    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, DS1337_REG_CONTROL, 1, &ctrl_val, 1, DS1337_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    if (init_cfg->enable_oscillator)
        ctrl_val &= ~0x80; /* Clear EOSC bit to START oscillator */
    else
        ctrl_val |= 0x80; /* Set EOSC bit to STOP oscillator */

    if (init_cfg->int_mode == DS1337_INTCN_ALARM)
        ctrl_val |= 0x04; /* Set INTCN bit */
    else
        ctrl_val &= ~0x04; /* Clear INTCN bit */

    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, DS1337_REG_CONTROL, 1, ctrl_val, 1, DS1337_CFG_TIMEOUT);
}

ec_gt ds1337_set_time(ds1337_dev_t* dev, const ds1337_time_t* time)
{
    if ((dev == NULL) || (time == NULL))
        return GMP_EC_GENERAL_ERROR;

    data_gt buf[7] = {0};

    buf[0] = dec_to_bcd(time->seconds);
    buf[1] = dec_to_bcd(time->minutes);
    buf[2] = dec_to_bcd(time->hours); /* Force 24-hour mode (Bit 6 is 0) */
    buf[3] = dec_to_bcd(time->day);
    buf[4] = dec_to_bcd(time->date);
    buf[5] = dec_to_bcd(time->month); /* Century bit is kept 0 */
    buf[6] = dec_to_bcd(time->year);

    /* Burst write 7 bytes starting from Seconds register */
    return gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, DS1337_REG_SECONDS, 1, buf, 7, DS1337_CFG_TIMEOUT);
}

ec_gt ds1337_get_time(ds1337_dev_t* dev, ds1337_time_t* time_ret)
{
    if ((dev == NULL) || (time_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    data_gt buf[7] = {0};

    ec_gt ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, DS1337_REG_SECONDS, 1, buf, 7, DS1337_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    time_ret->seconds = bcd_to_dec(buf[0] & 0x7F);
    time_ret->minutes = bcd_to_dec(buf[1] & 0x7F);
    time_ret->hours = bcd_to_dec(buf[2] & 0x3F); /* Assuming 24-hour mode */
    time_ret->day = bcd_to_dec(buf[3] & 0x07);
    time_ret->date = bcd_to_dec(buf[4] & 0x3F);
    time_ret->month = bcd_to_dec(buf[5] & 0x1F); /* Strip Century bit (Bit 7) */
    time_ret->year = bcd_to_dec(buf[6]);

    return GMP_EC_OK;
}

ec_gt ds1337_set_alarm1(ds1337_dev_t* dev, const ds1337_time_t* alarm_time, ds1337_alarm1_rate_et rate)
{
    if ((dev == NULL) || (alarm_time == NULL))
        return GMP_EC_GENERAL_ERROR;

    data_gt buf[4] = {0};
    uint8_t m_bits = (uint8_t)rate;

    /* Parse mask bits into M1, M2, M3, M4 (Bit 7 of each register) */
    buf[0] = dec_to_bcd(alarm_time->seconds) | ((m_bits & 0x01) ? 0x80 : 0x00);
    buf[1] = dec_to_bcd(alarm_time->minutes) | ((m_bits & 0x02) ? 0x80 : 0x00);
    buf[2] = dec_to_bcd(alarm_time->hours) | ((m_bits & 0x04) ? 0x80 : 0x00);

    /* Day or Date? If rate & 0x40, DY/DT bit is 1 (Day match) */
    uint8_t day_date_val = (m_bits & 0x40) ? dec_to_bcd(alarm_time->day) : dec_to_bcd(alarm_time->date);
    buf[3] = day_date_val | ((m_bits & 0x08) ? 0x80 : 0x00) | ((m_bits & 0x40) ? 0x40 : 0x00);

    return gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, DS1337_REG_ALARM1_SEC, 1, buf, 4, DS1337_CFG_TIMEOUT);
}

ec_gt ds1337_set_alarm2(ds1337_dev_t* dev, const ds1337_time_t* alarm_time, ds1337_alarm2_rate_et rate)
{
    if ((dev == NULL) || (alarm_time == NULL))
        return GMP_EC_GENERAL_ERROR;

    data_gt buf[3] = {0};
    uint8_t m_bits = (uint8_t)rate;

    /* Alarm 2 has NO seconds register. It triggers at 00 seconds. */
    buf[0] = dec_to_bcd(alarm_time->minutes) | ((m_bits & 0x01) ? 0x80 : 0x00);
    buf[1] = dec_to_bcd(alarm_time->hours) | ((m_bits & 0x02) ? 0x80 : 0x00);

    uint8_t day_date_val = (m_bits & 0x40) ? dec_to_bcd(alarm_time->day) : dec_to_bcd(alarm_time->date);
    buf[2] = day_date_val | ((m_bits & 0x04) ? 0x80 : 0x00) | ((m_bits & 0x40) ? 0x40 : 0x00);

    return gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, DS1337_REG_ALARM2_MIN, 1, buf, 3, DS1337_CFG_TIMEOUT);
}

ec_gt ds1337_enable_alarms(ds1337_dev_t* dev, bool enable_a1, bool enable_a2)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint32_t ctrl_val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, DS1337_REG_CONTROL, 1, &ctrl_val, 1, DS1337_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    if (enable_a1)
        ctrl_val |= 0x01;
    else
        ctrl_val &= ~0x01;
    if (enable_a2)
        ctrl_val |= 0x02;
    else
        ctrl_val &= ~0x02;

    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, DS1337_REG_CONTROL, 1, ctrl_val, 1, DS1337_CFG_TIMEOUT);
}

ec_gt ds1337_get_status(ds1337_dev_t* dev, uint8_t* status_ret)
{
    if ((dev == NULL) || (status_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t status_val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, DS1337_REG_STATUS, 1, &status_val, 1, DS1337_CFG_TIMEOUT);
    if (ret == GMP_EC_OK)
        *status_ret = (uint8_t)status_val;

    return ret;
}

ec_gt ds1337_clear_status(ds1337_dev_t* dev, uint8_t status_mask_to_clear)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint32_t status_val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, DS1337_REG_STATUS, 1, &status_val, 1, DS1337_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Status flags are cleared by writing 0 to them */
    status_val &= ~(status_mask_to_clear);

    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, DS1337_REG_STATUS, 1, status_val, 1, DS1337_CFG_TIMEOUT);
}
