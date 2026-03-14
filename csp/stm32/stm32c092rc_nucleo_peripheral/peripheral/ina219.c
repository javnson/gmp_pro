/**
 * @file    ina219.c
 * @brief   Hardware-agnostic driver implementation for TI INA219.
 */


#include <gmp_core.h>

#include <core/dev/meter/ina219.h>

ec_gt ina219_init(ina219_dev_t* dev, iic_halt bus, addr16_gt dev_addr, ina219_config_reg_t init_cfg)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->bus = bus;
    dev->dev_addr = dev_addr;

    /* Safely default LSBs to 0. Must be set by calibrate() before usage. */
    dev->current_lsb_mA = 0.0f;
    dev->power_lsb_mW = 0.0f;

    /* Ensure reserved bit is 0 */
    init_cfg.bits.reserved = 0;

    /* Push configuration to hardware (16-bit payload) */
    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, INA219_REG_CONFIG, 1, init_cfg.all, 2, INA219_CFG_TIMEOUT);
}

ec_gt ina219_calibrate(ina219_dev_t* dev, uint16_t cal_val, float current_lsb_mA)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Cache the LSB scaling factors for future reads */
    dev->current_lsb_mA = current_lsb_mA;

    /* Datasheet: Power LSB is fixed at 20 times the Current LSB */
    dev->power_lsb_mW = current_lsb_mA * 20.0f;

    /* Write to the Calibration Register (0x05) to activate the internal math engine */
    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, INA219_REG_CALIBRATION, 1, cal_val, 2, INA219_CFG_TIMEOUT);
}

ec_gt ina219_read_bus_voltage(ina219_dev_t* dev, float* voltage_V_ret)
{
    if ((dev == NULL) || (voltage_V_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, INA219_REG_BUS_VOLTAGE, 1, &val, 2, INA219_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Datasheet: Shift right 3 bits to remove CNVR and OVF flags.
     * The LSB is 4mV.
     */
    uint16_t raw_bus = (uint16_t)(val >> 3);

    *voltage_V_ret = (float)raw_bus * 0.004f; /* Convert to Volts */

    return GMP_EC_OK;
}

ec_gt ina219_read_shunt_voltage(ina219_dev_t* dev, float* voltage_mV_ret)
{
    if ((dev == NULL) || (voltage_mV_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, INA219_REG_SHUNT_VOLTAGE, 1, &val, 2, INA219_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Datasheet: Shunt Voltage is a 16-bit SIGNED integer.
     * The LSB is 10uV (or 0.01mV).
     */
    int16_t raw_shunt = (int16_t)val;

    *voltage_mV_ret = (float)raw_shunt * 0.01f; /* Convert to millivolts */

    return GMP_EC_OK;
}

ec_gt ina219_read_current(ina219_dev_t* dev, float* current_mA_ret)
{
    if ((dev == NULL) || (current_mA_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* Safety Check: Has calibration been performed? */
    if (dev->current_lsb_mA <= 0.0f)
        return GMP_EC_NOT_READY;

    uint32_t val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, INA219_REG_CURRENT, 1, &val, 2, INA219_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Current is a 16-bit SIGNED integer */
    int16_t raw_current = (int16_t)val;

    *current_mA_ret = (float)raw_current * dev->current_lsb_mA;

    return GMP_EC_OK;
}

ec_gt ina219_read_power(ina219_dev_t* dev, float* power_mW_ret)
{
    if ((dev == NULL) || (power_mW_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* Safety Check: Has calibration been performed? */
    if (dev->power_lsb_mW <= 0.0f)
        return GMP_EC_NOT_READY;

    uint32_t val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, INA219_REG_POWER, 1, &val, 2, INA219_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Power is an UNSIGNED 16-bit integer (can only be positive) */
    uint16_t raw_power = (uint16_t)val;

    *power_mW_ret = (float)raw_power * dev->power_lsb_mW;

    return GMP_EC_OK;
}
