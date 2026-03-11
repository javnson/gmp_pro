/**
 * @file    bmp280.c
 * @brief   Hardware-agnostic driver implementation for Bosch BMP280.
 * @example
 * bmp280_dev_t my_bmp;
 * bmp280_init_t bmp_cfg = {0};
 *  1. config sampling param
 * bmp_cfg.ctrl.bits.mode = BMP280_MODE_NORMAL;   // 开启内部持续采样模式
 * bmp_cfg.ctrl.bits.osrs_t = BMP280_OSRS_2X;     // 温度 2 倍过采样
 * bmp_cfg.ctrl.bits.osrs_p = BMP280_OSRS_16X;    // 压力 16 倍高精度过采样
 * bmp_cfg.config.bits.t_sb = BMP280_TSB_125_MS;  // 每次采样间隔 125ms
 * bmp_cfg.config.bits.filter = BMP280_FILTER_16; // 开启最高抗风干扰 IIR 滤波器
 *  2. 传入配置并挂载设备 (底层自动完成 24 字节出厂系数下载) 
 * bmp280_init(&my_bmp, I2CA_BASE, BMP280_CALC_ADDR(0), bmp_cfg);
 *  3. 在主循环中随时以 O(1) 的总线开销读取双数据
 * float temp, press;
 * bmp280_read_temp_and_pressure(&my_bmp, &temp, &press);
 */

#include <gmp_core.h>

#include <core/dev/sensor/bmp280.h>


/**
 * @brief Internal delay utilizing the system tick
 */
static void bmp280_delay_ms(uint32_t ms)
{
    time_gt start = gmp_base_get_system_tick();
    while (!gmp_base_is_delay_elapsed(start, ms))
    {
    }
}

/**
 * @brief Reads the 24-byte calibration block and constructs the signed/unsigned 16-bit variables.
 */
static ec_gt bmp280_read_calibration(bmp280_dev_t* dev)
{
    data_gt buf[24] = {0};

    ec_gt ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, BMP280_REG_CALIB_START, 1, buf, 24, BMP280_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* BMP280 stores calibration data in Little-Endian format (LSB first) */
    dev->calib.dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
    dev->calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    dev->calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
    dev->calib.dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
    dev->calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
    dev->calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    dev->calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    dev->calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    dev->calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    dev->calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    dev->calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    dev->calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

    return GMP_EC_OK;
}

ec_gt bmp280_init(bmp280_dev_t* dev, iic_halt bus, addr16_gt dev_addr, bmp280_init_t init_cfg)
{
    ec_gt ret;
    uint8_t chip_id = 0;

    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->bus = bus;
    dev->dev_addr = dev_addr;
    dev->t_fine = 0;

    /* 1. Device Probe: Read ID */
    ret = bmp280_get_id(dev, &chip_id);
    if (ret != GMP_EC_OK)
        return ret;
    if (chip_id != BMP280_EXPECTED_ID)
        return GMP_EC_NACK;

    /* 2. Soft Reset to wake up from unknown states */
    ret = bmp280_reset(dev);
    if (ret != GMP_EC_OK)
        return ret;
    bmp280_delay_ms(10); /* Wait for reset to complete */

    /* 3. Extract Factory Calibration Constants (CRITICAL) */
    ret = bmp280_read_calibration(dev);
    if (ret != GMP_EC_OK)
        return ret;

    /* 4. Write Configuration Reg (0xF5) */
    init_cfg.config.bits.reserved = 0; /* Ensure reserved is 0 */
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, BMP280_REG_CONFIG, 1, init_cfg.config.all, 1,
                                BMP280_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 5. Write Ctrl_Meas Reg (0xF4) - This starts the measurement if mode != SLEEP */
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, BMP280_REG_CTRL_MEAS, 1, init_cfg.ctrl.all, 1,
                                BMP280_CFG_TIMEOUT);

    return ret;
}

ec_gt bmp280_reset(bmp280_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;
    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, BMP280_REG_RESET, 1, BMP280_CMD_SOFT_RESET, 1,
                                 BMP280_CFG_TIMEOUT);
}

ec_gt bmp280_get_id(bmp280_dev_t* dev, uint8_t* id_ret)
{
    if ((dev == NULL) || (id_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, BMP280_REG_ID, 1, &val, 1, BMP280_CFG_TIMEOUT);
    if (ret == GMP_EC_OK)
        *id_ret = (uint8_t)val;
    return ret;
}

/**
 * @brief Bosch Official Temperature Compensation Formula
 */
static float bmp280_compensate_T(bmp280_dev_t* dev, int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) * ((int32_t)dev->calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) *
            ((int32_t)dev->calib.dig_T3)) >>
           14;

    dev->t_fine = var1 + var2; /* Cache t_fine for Pressure calculation */
    T = (dev->t_fine * 5 + 128) >> 8;

    return (float)T / 100.0f;
}

/**
 * @brief Bosch Official Pressure Compensation Formula (Returns Pascals)
 * @note Requires 64-bit integer support for highest accuracy.
 */
static float bmp280_compensate_P(bmp280_dev_t* dev, int32_t adc_P)
{
#ifdef SPECIFY_ENABLE_INTEGER64
    int64_t var1, var2, p;

    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) + ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;

    if (var1 == 0)
        return 0.0f; /* Avoid division by zero */

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

    return (float)p / 256.0f;
#else
    /* Fallback 32-bit math approach if 64-bit is strictly unavailable */
    int32_t var1, var2;
    uint32_t p;

    var1 = (((int32_t)dev->t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dev->calib.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)dev->calib.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)dev->calib.dig_P4) << 16);
    var1 = (((dev->calib.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int32_t)dev->calib.dig_P2) * var1) >> 1)) >>
           18;
    var1 = ((((32768 + var1)) * ((int32_t)dev->calib.dig_P1)) >> 15);

    if (var1 == 0)
        return 0.0f;

    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / (uint32_t)var1) * 2;

    var1 = (((int32_t)dev->calib.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)dev->calib.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dev->calib.dig_P7) >> 4));

    return (float)p;
#endif
}

ec_gt bmp280_read_temp_and_pressure(bmp280_dev_t* dev, float* temp_c_ret, float* pressure_pa_ret)
{
    if ((dev == NULL) || (temp_c_ret == NULL) || (pressure_pa_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    data_gt buf[6] = {0};

    /* Burst read 6 bytes starting from PRESS_MSB (0xF7) */
    ec_gt ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, BMP280_REG_PRESS_MSB, 1, buf, 6, BMP280_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Assemble raw 20-bit ADC values */
    int32_t adc_P = (int32_t)((((uint32_t)buf[0]) << 12) | (((uint32_t)buf[1]) << 4) | (((uint32_t)buf[2]) >> 4));
    int32_t adc_T = (int32_t)((((uint32_t)buf[3]) << 12) | (((uint32_t)buf[4]) << 4) | (((uint32_t)buf[5]) >> 4));

    /* Compensate (Temperature MUST be calculated first to update t_fine) */
    *temp_c_ret = bmp280_compensate_T(dev, adc_T);
    *pressure_pa_ret = bmp280_compensate_P(dev, adc_P);

    return GMP_EC_OK;
}

ec_gt bmp280_read_temperature(bmp280_dev_t* dev, float* temp_c_ret)
{
    float dummy_press;
    /* Reading both is required to ensure t_fine is updated synchronously */
    return bmp280_read_temp_and_pressure(dev, temp_c_ret, &dummy_press);
}

ec_gt bmp280_read_pressure(bmp280_dev_t* dev, float* pressure_pa_ret)
{
    float dummy_temp;
    /* Pressure compensation depends on current temperature's t_fine */
    return bmp280_read_temp_and_pressure(dev, &dummy_temp, pressure_pa_ret);
}
