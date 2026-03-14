/**
 * @file    ad5933.c
 * @brief   Hardware-agnostic driver implementation for AD5933.
 */

#include <gmp_core.h>
#include <math.h>

#include <core/dev/application/ad5933.h>

/* ========================================================================= */
/* ==================== PRIVATE HAL WRAPPERS =============================== */
/* ========================================================================= */

/**
 * @brief Write multi-byte registers (MSB first) explicitly byte-by-byte
 * as recommended by the ADI official implementation.
 */
static ec_gt ad5933_write_reg_multi(ad5933_dev_t* dev, uint8_t reg_addr, uint32_t value, uint8_t bytes)
{
    uint8_t i;

    for (i = 0; i < bytes; i++)
    {
        uint8_t shift = (bytes - 1 - i) * 8;
        uint8_t byte_val = (value >> shift) & 0xFF;

        ec_gt ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, reg_addr + i, 1, byte_val, 1, AD5933_CFG_TIMEOUT);
        if (ret != GMP_EC_OK)
            return ret;
    }
    return GMP_EC_OK;
}

/**
 * @brief Specific read function required by AD5933 using the 0xB0 Pointer Command.
 */
static ec_gt ad5933_read_reg_byte(ad5933_dev_t* dev, uint8_t reg_addr, data_gt* data)
{
    /* 1. Set the Address Pointer */
    data_gt ptr_cmd[2] = {AD5933_ADDR_POINTER, reg_addr};
    ec_gt ret = gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, 0, 0, ptr_cmd, 2, AD5933_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Read the actual byte (addr_len = 0 implies direct read) */
    return gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, 0, 0, data, 1, AD5933_CFG_TIMEOUT);
}

static ec_gt ad5933_read_reg_multi(ad5933_dev_t* dev, uint8_t reg_addr, uint32_t* value, uint8_t bytes)
{
    uint32_t val = 0;
    uint8_t i;

    for (i = 0; i < bytes; i++)
    {
        data_gt byte_val = 0;
        ec_gt ret = ad5933_read_reg_byte(dev, reg_addr + i, &byte_val);
        if (ret != GMP_EC_OK)
            return ret;
        val = (val << 8) | byte_val;
    }
    *value = val;
    return GMP_EC_OK;
}

/* ========================================================================= */
/* ==================== PUBLIC API IMPLEMENTATION ========================== */
/* ========================================================================= */

ec_gt ad5933_init(ad5933_dev_t* dev, iic_halt bus, const ad5933_init_t* init_cfg)
{
    ec_gt ret;

    if ((dev == NULL) || (init_cfg == NULL))
        return GMP_EC_GENERAL_ERROR;

    dev->bus = bus;
    dev->dev_addr = AD5933_I2C_ADDR;
    dev->current_range = init_cfg->range;
    dev->current_gain = init_cfg->gain;
    dev->gain_factor = 0;

    if (init_cfg->clk_src == AD5933_CLK_EXTERNAL)
    {
        dev->sys_clk = init_cfg->ext_clk_freq;
    }
    else
    {
        dev->sys_clk = AD5933_INTERNAL_SYS_CLK;
    }

    /* 1. Reset the device and set clock source */
    uint8_t ctrl_lb = (1 << 4) | (init_cfg->clk_src << 3);
    ret = ad5933_write_reg_multi(dev, AD5933_REG_CTRL_LB, ctrl_lb, 1);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Put into Standby and apply Gain/Range */
    return ad5933_set_function(dev, AD5933_FUNC_STANDBY);
}

ec_gt ad5933_reset(ad5933_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Read current LB, set reset bit, write back */
    uint32_t lb_val = 0;
    ec_gt ret = ad5933_read_reg_multi(dev, AD5933_REG_CTRL_LB, &lb_val, 1);
    if (ret != GMP_EC_OK)
        return ret;

    lb_val |= (1 << 4);
    return ad5933_write_reg_multi(dev, AD5933_REG_CTRL_LB, lb_val, 1);
}

ec_gt ad5933_set_function(ad5933_dev_t* dev, ad5933_func_et function)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    ad5933_ctrl_hb_t hb;
    hb.all = 0;
    hb.bits.function = function;
    hb.bits.range = dev->current_range;
    hb.bits.pga_gain = dev->current_gain;

    return ad5933_write_reg_multi(dev, AD5933_REG_CTRL_HB, hb.all, 1);
}

ec_gt ad5933_config_sweep(ad5933_dev_t* dev, uint32_t start_freq_hz, uint32_t inc_freq_hz, uint16_t inc_num)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    if (inc_num > 511)
        inc_num = 511;

        /* OPTIMIZATION: 
     * Original ADI Float: (Start_Freq * 4 / MCLK) * 2^27
     * Our 64-bit Integer: (Start_Freq * 2^29) / MCLK
     * This entirely eliminates the floating point library overhead!
     */
#ifdef SPECIFY_ENABLE_INTEGER64
    uint32_t start_reg = (uint32_t)(((uint64_t)start_freq_hz * (1ULL << 29)) / dev->sys_clk);
    uint32_t inc_reg = (uint32_t)(((uint64_t)inc_freq_hz * (1ULL << 29)) / dev->sys_clk);
#else
    /* Fallback if compiler strictly forbids 64-bit math (Rare on modern C) */
    uint32_t start_reg = (uint32_t)((double)start_freq_hz * 4.0 / dev->sys_clk * 134217728.0);
    uint32_t inc_reg = (uint32_t)((double)inc_freq_hz * 4.0 / dev->sys_clk * 134217728.0);
#endif

    ec_gt ret;
    ret = ad5933_write_reg_multi(dev, AD5933_REG_FREQ_START, start_reg, 3);
    if (ret != GMP_EC_OK)
        return ret;

    ret = ad5933_write_reg_multi(dev, AD5933_REG_FREQ_INC, inc_reg, 3);
    if (ret != GMP_EC_OK)
        return ret;

    return ad5933_write_reg_multi(dev, AD5933_REG_INC_NUM, inc_num, 2);
}

ec_gt ad5933_set_settling_time(ad5933_dev_t* dev, ad5933_settling_mult_et multiplier, uint16_t cycles)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint16_t reg_val = (cycles & 0x01FF) | (multiplier << 9);
    return ad5933_write_reg_multi(dev, AD5933_REG_SETTLING_CYCLES, reg_val, 2);
}

ec_gt ad5933_read_temperature(ad5933_dev_t* dev, float* temp_c_ret)
{
    if ((dev == NULL) || (temp_c_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    ec_gt ret = ad5933_set_function(dev, AD5933_FUNC_MEASURE_TEMP);
    if (ret != GMP_EC_OK)
        return ret;

    /* Wait for valid temperature data */
    uint32_t status = 0;
    time_gt start_time = gmp_base_get_system_tick();
    do
    {
        if (gmp_base_is_delay_elapsed(start_time, AD5933_CFG_TIMEOUT))
            return GMP_EC_TIMEOUT;
        ret = ad5933_read_reg_multi(dev, AD5933_REG_STATUS, &status, 1);
        if (ret != GMP_EC_OK)
            return ret;
    } while ((status & AD5933_STAT_TEMP_VALID) == 0);

    uint32_t raw_temp = 0;
    ret = ad5933_read_reg_multi(dev, AD5933_REG_TEMP_DATA, &raw_temp, 2);
    if (ret != GMP_EC_OK)
        return ret;

    /* Convert 14-bit signed to Celsius */
    if (raw_temp < 8192)
    {
        *temp_c_ret = (float)raw_temp / 32.0f;
    }
    else
    {
        *temp_c_ret = (float)((int32_t)raw_temp - 16384) / 32.0f;
    }

    return GMP_EC_OK;
}

ec_gt ad5933_get_complex_data(ad5933_dev_t* dev, int16_t* real_ret, int16_t* imag_ret)
{
    if ((dev == NULL) || (real_ret == NULL) || (imag_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t status = 0;
    time_gt start_time = gmp_base_get_system_tick();
    do
    {
        if (gmp_base_is_delay_elapsed(start_time, AD5933_CFG_TIMEOUT))
            return GMP_EC_TIMEOUT;
        ec_gt ret = ad5933_read_reg_multi(dev, AD5933_REG_STATUS, &status, 1);
        if (ret != GMP_EC_OK)
            return ret;
    } while ((status & AD5933_STAT_DATA_VALID) == 0);

    uint32_t real_val = 0, imag_val = 0;
    ec_gt ret;

    ret = ad5933_read_reg_multi(dev, AD5933_REG_REAL_DATA, &real_val, 2);
    if (ret != GMP_EC_OK)
        return ret;

    ret = ad5933_read_reg_multi(dev, AD5933_REG_IMAG_DATA, &imag_val, 2);
    if (ret != GMP_EC_OK)
        return ret;

    *real_ret = (int16_t)real_val;
    *imag_ret = (int16_t)imag_val;

    return GMP_EC_OK;
}

ec_gt ad5933_calculate_gain_factor(ad5933_dev_t* dev, float cal_impedance_ohms, ad5933_func_et function)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    ec_gt ret = ad5933_set_function(dev, function);
    if (ret != GMP_EC_OK)
        return ret;

    int16_t real_data = 0, imag_data = 0;
    ret = ad5933_get_complex_data(dev, &real_data, &imag_data);
    if (ret != GMP_EC_OK)
        return ret;

    float magnitude = sqrtf(((float)real_data * real_data) + ((float)imag_data * imag_data));

    if (magnitude == 0.0)
        return GMP_EC_GENERAL_ERROR; /* Prevent divide by zero */

    dev->gain_factor = 1.0 / (magnitude * cal_impedance_ohms);

    return GMP_EC_OK;
}

ec_gt ad5933_calculate_impedance(ad5933_dev_t* dev, ad5933_func_et function, float* impedance_ohms_ret)
{
    if ((dev == NULL) || (impedance_ohms_ret == NULL))
        return GMP_EC_GENERAL_ERROR;
    if (dev->gain_factor <= 0.0)
        return GMP_EC_NOT_READY; /* Ensure calibrated */

    ec_gt ret = ad5933_set_function(dev, function);
    if (ret != GMP_EC_OK)
        return ret;

    int16_t real_data = 0, imag_data = 0;
    ret = ad5933_get_complex_data(dev, &real_data, &imag_data);
    if (ret != GMP_EC_OK)
        return ret;

    float magnitude = sqrtf(((double)real_data * real_data) + ((double)imag_data * imag_data));

    if (magnitude == 0.0f)
        return GMP_EC_GENERAL_ERROR; /* Prevent divide by zero */

    *impedance_ohms_ret = 1.0f / (magnitude * dev->gain_factor);

    return GMP_EC_OK;
}
