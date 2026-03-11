

#include <gmp_core.h>

/**
 * @brief   Helper function to map STM32 HAL status to GMP Error Codes.
 *
 * @param[in] hi2c       Pointer to the STM32 I2C handle.
 * @param[in] hal_status Status returned by STM32 HAL functions.
 *
 * @return  ec_gt        Mapped general error code.
 */
static ec_gt stm32_hal_status_to_ec(I2C_HandleTypeDef* hi2c, HAL_StatusTypeDef hal_status)
{
    switch (hal_status)
    {
    case HAL_OK:
        return GMP_EC_OK;
    case HAL_BUSY:
        return GMP_EC_BUSY;
    case HAL_TIMEOUT:
        return GMP_EC_TIMEOUT;
    case HAL_ERROR:
    default:
        /* Check if the error is specifically an Acknowledge Failure (NACK) */
        if (hi2c->ErrorCode & HAL_I2C_ERROR_AF)
        {
            return GMP_EC_NACK;
        }
        return GMP_EC_GENERAL_ERROR;
    }
}

ec_gt gmp_hal_iic_write_cmd(iic_halt h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout)
{
    I2C_HandleTypeDef* hi2c = (I2C_HandleTypeDef*)(uintptr_t)h;
    uint8_t buf[4] = {0};
    uint32_t i;

    /* Serialize command into bytes (MSB first) */
    for (i = 0; i < cmd_len; i++)
    {
        buf[i] = (uint8_t)((cmd >> ((cmd_len - 1 - i) * 8)) & 0xFF);
    }

    /* Shift the 7-bit address left by 1 for STM32 HAL */
    uint16_t stm32_addr = (uint16_t)(dev_addr << 1);

    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(hi2c, stm32_addr, buf, (uint16_t)cmd_len, (uint32_t)timeout);

    return stm32_hal_status_to_ec(hi2c, res);
}

ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout)
{
    I2C_HandleTypeDef* hi2c = (I2C_HandleTypeDef*)(uintptr_t)h;
    uint8_t buf[8] = {0}; /* Max 4 bytes addr + 4 bytes data */
    uint32_t idx = 0;
    uint32_t i;

    /* 1. Pack Register Address (MSB first) */
    for (i = 0; i < addr_len; i++)
    {
        buf[idx++] = (uint8_t)((reg_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }

    /* 2. Pack Register Data (MSB first) */
    for (i = 0; i < reg_len; i++)
    {
        buf[idx++] = (uint8_t)((reg_data >> ((reg_len - 1 - i) * 8)) & 0xFF);
    }

    uint16_t stm32_addr = (uint16_t)(dev_addr << 1);

    /* Use Master_Transmit to send continuous bytes, which acts exactly as a register write */
    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(hi2c, stm32_addr, buf, (uint16_t)idx, (uint32_t)timeout);

    return stm32_hal_status_to_ec(hi2c, res);
}

ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout)
{
    I2C_HandleTypeDef* hi2c = (I2C_HandleTypeDef*)(uintptr_t)h;
    HAL_StatusTypeDef res;
    uint16_t stm32_addr = (uint16_t)(dev_addr << 1);

    if (addr_len == 0)
    {
        /* Fallback to raw transmission if no memory address is specified */
        res = HAL_I2C_Master_Transmit(hi2c, stm32_addr, (uint8_t*)mem, (uint16_t)mem_len, (uint32_t)timeout);
    }
    else
    {
        /* Convert our size_gt to STM32's internal constants */
        uint16_t mem_add_size = (addr_len == 1) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;

        res = HAL_I2C_Mem_Write(hi2c, stm32_addr, (uint16_t)mem_addr, mem_add_size, (uint8_t*)mem, (uint16_t)mem_len,
                                (uint32_t)timeout);
    }

    return stm32_hal_status_to_ec(hi2c, res);
}

ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout)
{
    if (reg_data_ret == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    I2C_HandleTypeDef* hi2c = (I2C_HandleTypeDef*)(uintptr_t)h;
    uint8_t buf[4] = {0};
    HAL_StatusTypeDef res;
    uint16_t stm32_addr = (uint16_t)(dev_addr << 1);

    if (addr_len == 0)
    {
        /* Raw receive without setting an address pointer */
        res = HAL_I2C_Master_Receive(hi2c, stm32_addr, buf, (uint16_t)reg_len, (uint32_t)timeout);
    }
    else
    {
        uint16_t mem_add_size = (addr_len == 1) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;

        /* Mem_Read will automatically issue START -> Addr -> RESTART -> Read -> STOP */
        res = HAL_I2C_Mem_Read(hi2c, stm32_addr, (uint16_t)reg_addr, mem_add_size, buf, (uint16_t)reg_len,
                               (uint32_t)timeout);
    }

    if (res == HAL_OK)
    {
        uint32_t val = 0;
        uint32_t i;
        /* Assemble bytes into uint32_t (assuming MSB first standard) */
        for (i = 0; i < reg_len; i++)
        {
            val = (val << 8) | buf[i];
        }
        *reg_data_ret = val;
    }

    return stm32_hal_status_to_ec(hi2c, res);
}

ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout)
{
    if (mem == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    I2C_HandleTypeDef* hi2c = (I2C_HandleTypeDef*)(uintptr_t)h;
    HAL_StatusTypeDef res;
    uint16_t stm32_addr = (uint16_t)(dev_addr << 1);

    if (addr_len == 0)
    {
        res = HAL_I2C_Master_Receive(hi2c, stm32_addr, (uint8_t*)mem, (uint16_t)mem_len, (uint32_t)timeout);
    }
    else
    {
        uint16_t mem_add_size = (addr_len == 1) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
        res = HAL_I2C_Mem_Read(hi2c, stm32_addr, (uint16_t)mem_addr, mem_add_size, (uint8_t*)mem, (uint16_t)mem_len,
                               (uint32_t)timeout);
    }

    return stm32_hal_status_to_ec(hi2c, res);
}
