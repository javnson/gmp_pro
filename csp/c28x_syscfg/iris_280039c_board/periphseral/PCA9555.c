/**
 * @file    pca9555.c
 * @brief   Hardware-agnostic driver implementation for PCA9555.
 */


#include <gmp_core.h>
#include <core/dev/gpio/pca9555.h>

ec_gt pca9555_init(pca9555_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const pca9555_init_t* init_cfg)
{
    ec_gt ret;

    if ((dev == NULL) || (init_cfg == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    dev->bus = bus;
    dev->dev_addr = dev_addr;

    /* Initialize shadow registers from user configuration */
    dev->shadow_cfg[0] = init_cfg->cfg_port0;
    dev->shadow_cfg[1] = init_cfg->cfg_port1;
    dev->shadow_out[0] = init_cfg->out_port0;
    dev->shadow_out[1] = init_cfg->out_port1;
    dev->shadow_pol[0] = init_cfg->pol_port0;
    dev->shadow_pol[1] = init_cfg->pol_port1;

    /* 1. Write Polarity Registers (Device probe naturally happens here) */
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, PCA9555_REG_POL_PORT0, 1, dev->shadow_pol[0], 1,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, PCA9555_REG_POL_PORT1, 1, dev->shadow_pol[1], 1,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Write Initial Output States (Important to do this before enabling output direction) */
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, PCA9555_REG_OUT_PORT0, 1, dev->shadow_out[0], 1,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, PCA9555_REG_OUT_PORT1, 1, dev->shadow_out[1], 1,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* 3. Write Direction Config Registers */
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, PCA9555_REG_CFG_PORT0, 1, dev->shadow_cfg[0], 1,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;
    ret = gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, PCA9555_REG_CFG_PORT1, 1, dev->shadow_cfg[1], 1,
                                PCA9555_CFG_TIMEOUT);

    return ret;
}

ec_gt pca9555_set_pin_direction(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, pca9555_dir_et dir)
{
    if ((dev == NULL) || (pin_num > 7))
        return GMP_EC_GENERAL_ERROR;

    uint8_t reg_addr = (port == PCA9555_PORT_0) ? PCA9555_REG_CFG_PORT0 : PCA9555_REG_CFG_PORT1;

    /* Modify shadow register */
    if (dir == PCA9555_DIR_INPUT)
    {
        dev->shadow_cfg[port] |= (1 << pin_num);
    }
    else
    {
        dev->shadow_cfg[port] &= ~(1 << pin_num);
    }

    /* Write shadow register to hardware */
    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, reg_addr, 1, dev->shadow_cfg[port], 1, PCA9555_CFG_TIMEOUT);
}

ec_gt pca9555_set_pin_polarity(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, pca9555_pol_et pol)
{
    if ((dev == NULL) || (pin_num > 7))
        return GMP_EC_GENERAL_ERROR;

    uint8_t reg_addr = (port == PCA9555_PORT_0) ? PCA9555_REG_POL_PORT0 : PCA9555_REG_POL_PORT1;

    /* Modify shadow register */
    if (pol == PCA9555_POL_INVERTED)
    {
        dev->shadow_pol[port] |= (1 << pin_num);
    }
    else
    {
        dev->shadow_pol[port] &= ~(1 << pin_num);
    }

    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, reg_addr, 1, dev->shadow_pol[port], 1, PCA9555_CFG_TIMEOUT);
}

ec_gt pca9555_set_pin_output(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, fast_gt state)
{
    if ((dev == NULL) || (pin_num > 7))
        return GMP_EC_GENERAL_ERROR;

    uint8_t reg_addr = (port == PCA9555_PORT_0) ? PCA9555_REG_OUT_PORT0 : PCA9555_REG_OUT_PORT1;

    /* Modify shadow register without reading from I2C */
    if (state)
    {
        dev->shadow_out[port] |= (1 << pin_num);
    }
    else
    {
        dev->shadow_out[port] &= ~(1 << pin_num);
    }

    return gmp_hal_iic_write_reg(dev->bus, dev->dev_addr, reg_addr, 1, dev->shadow_out[port], 1, PCA9555_CFG_TIMEOUT);
}

fast_gt pca9555_get_pin_input(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, ec_gt* err_code_ret)
{
    uint32_t port_val = 0;
    ec_gt ret;

    if ((dev == NULL) || (pin_num > 7))
    {
        if (err_code_ret)
            *err_code_ret = GMP_EC_GENERAL_ERROR;
        return 0;
    }

    uint8_t reg_addr = (port == PCA9555_PORT_0) ? PCA9555_REG_IN_PORT0 : PCA9555_REG_IN_PORT1;

    /* Must read actual physical state from I2C bus */
    ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, reg_addr, 1, &port_val, 1, PCA9555_CFG_TIMEOUT);

    if (err_code_ret)
    {
        *err_code_ret = ret;
    }

    /* Extract the specific bit and return as fast_gt */
    if (ret == GMP_EC_OK)
    {
        return (fast_gt)((port_val >> pin_num) & 0x01);
    }

    return 0;
}

ec_gt pca9555_get_port0_input(pca9555_dev_t* dev, uint8_t* port_data_ret)
{
    if ((dev == NULL) || (port_data_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t val = 0;
    ec_gt ret = gmp_hal_iic_read_reg(dev->bus, dev->dev_addr, PCA9555_REG_IN_PORT0, 1, &val, 1, PCA9555_CFG_TIMEOUT);

    if (ret == GMP_EC_OK)
    {
        *port_data_ret = (uint8_t)val;
    }
    return ret;
}

ec_gt pca9555_get_port1_output(pca9555_dev_t* dev, uint8_t* port_data_ret)
{
    if ((dev == NULL) || (port_data_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* Since the user asked for output state, we return the cached shadow register. 
     * This takes 0 I2C bus time! */
    *port_data_ret = dev->shadow_out[PCA9555_PORT_1];

    return GMP_EC_OK;
}

ec_gt pca9555_get_port_config(pca9555_dev_t* dev, pca9555_port_et port, uint8_t* cfg_ret, uint8_t* pol_ret)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Return from shadow cache, extremely fast */
    if (cfg_ret)
        *cfg_ret = dev->shadow_cfg[port];
    if (pol_ret)
        *pol_ret = dev->shadow_pol[port];

    return GMP_EC_OK;
}


ec_gt pca9555_flush_outputs(pca9555_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* PCA9555 Auto-Increment feature:
     * Writing 2 bytes starting at REG_OUT_PORT0 (0x02) will automatically 
     * write the first byte to Port 0 and the second byte to Port 1.
     * This turns 2 I2C transactions into just 1!
     */
    return gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, PCA9555_REG_OUT_PORT0, 1, dev->shadow_out, 2,
                                 PCA9555_CFG_TIMEOUT);
}

ec_gt pca9555_flush_all_shadows(pca9555_dev_t* dev)
{
    ec_gt ret;
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Flush Polarity */
    ret = gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, PCA9555_REG_POL_PORT0, 1, dev->shadow_pol, 2,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Flush Outputs */
    ret = gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, PCA9555_REG_OUT_PORT0, 1, dev->shadow_out, 2,
                                PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Flush Config (Direction) */
    ret = gmp_hal_iic_write_mem(dev->bus, dev->dev_addr, PCA9555_REG_CFG_PORT0, 1, dev->shadow_cfg, 2,
                                PCA9555_CFG_TIMEOUT);

    return ret;
}

ec_gt pca9555_refresh_shadows(pca9555_dev_t* dev)
{
    ec_gt ret;
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Read Config Registers */
    ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, PCA9555_REG_CFG_PORT0, 1, dev->shadow_cfg, 2,
                               PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Read Output Registers */
    ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, PCA9555_REG_OUT_PORT0, 1, dev->shadow_out, 2,
                               PCA9555_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Read Polarity Registers */
    ret = gmp_hal_iic_read_mem(dev->bus, dev->dev_addr, PCA9555_REG_POL_PORT0, 1, dev->shadow_pol, 2,
                               PCA9555_CFG_TIMEOUT);

    return ret;
}

