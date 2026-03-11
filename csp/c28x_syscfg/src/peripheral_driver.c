

#include <gmp_core.h>

/**
 * @brief   Helper macro for timeout checking to keep code clean and prevent hardware lockups.
 * @note    It utilizes the overflow-safe time checking mechanism from gmp_base.
 * * @param   start_time  The recorded start time tick.
 * @param   timeout_ms  The maximum allowed wait time in milliseconds.
 */
#define CHECK_TIMEOUT(start_time, timeout_ms)                                                                          \
    if (gmp_base_is_delay_elapsed((start_time), (timeout_ms)))                                                         \
    {                                                                                                                  \
        I2C_sendStopCondition(h);                                                                                      \
        return GMP_EC_TIMEOUT;                                                                                         \
    }

/**
 * @brief   Helper function to safely wait for the I2C bus to become idle.
 * * @param   h         I2C hardware base address.
 * @param   timeout   Maximum wait time in milliseconds.
 * @return  ec_gt     GMP_EC_OK if idle, GMP_EC_TIMEOUT if bus is locked.
 */
static ec_gt wait_bus_idle(iic_halt h, time_gt timeout)
{
    time_gt start = gmp_base_get_system_tick();
    while (I2C_isBusBusy(h))
    {
        CHECK_TIMEOUT(start, timeout);
    }
    return GMP_EC_OK;
}

/**
 * @brief   Helper function to reset FIFO and clear error status before a new transaction.
 * * @param   h         I2C hardware base address.
 */
static void reset_bus_status(iic_halt h)
{
    //I2C_resetTxFIFO(h);
    //I2C_resetRxFIFO(h);
    I2C_clearStatus(h, I2C_STS_NO_ACK | I2C_STS_ARB_LOST);
}

ec_gt gmp_hal_iic_write_cmd(iic_halt h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout)
{
    ec_gt ret = wait_bus_idle(h, timeout);
    if (ret != GMP_EC_OK)
        return ret;

    I2C_setTargetAddress(h, dev_addr);
    reset_bus_status(h);
    I2C_setDataCount(h, cmd_len);

    // Serialize command into bytes (MSB first generally used in I2C)
    // The peripheral driver can use LE16/BE16 macros to format 'cmd' beforehand.
    int32_t i;
    for (i = (int32_t)cmd_len - 1; i >= 0; i--)
    {
        I2C_putData(h, (cmd >> (i * 8)) & 0xFF);
    }

    I2C_setConfig(h, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(h);
    I2C_sendStopCondition(h);

    time_gt start = gmp_base_get_system_tick();
    while (I2C_getStopConditionStatus(h))
    {
        if (I2C_getStatus(h) & I2C_STS_NO_ACK)
        {
            I2C_sendStopCondition(h);
            I2C_clearStatus(h, I2C_STS_NO_ACK);
            return GMP_EC_NACK;
        }
        CHECK_TIMEOUT(start, timeout);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout)
{
    ec_gt ret = wait_bus_idle(h, timeout);
    if (ret != GMP_EC_OK)
        return ret;

    I2C_setTargetAddress(h, dev_addr);
    reset_bus_status(h);

    // Total bytes = address bytes + data bytes
    I2C_setDataCount(h, addr_len + reg_len);

    // 1. Put Address Bytes (MSB first)
    int32_t i;
    for (i = (int32_t)addr_len - 1; i >= 0; i--)
    {
        I2C_putData(h, (reg_addr >> (i * 8)) & 0xFF);
    }

    // 2. Put Data Bytes (MSB first)
    for (i = (int32_t)reg_len - 1; i >= 0; i--)
    {
        I2C_putData(h, (reg_data >> (i * 8)) & 0xFF);
    }

    I2C_setConfig(h, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(h);
    I2C_sendStopCondition(h);

    time_gt start = gmp_base_get_system_tick();
    while (I2C_getStopConditionStatus(h))
    {
        if (I2C_getStatus(h) & I2C_STS_NO_ACK)
        {
            I2C_sendStopCondition(h);
            I2C_clearStatus(h, I2C_STS_NO_ACK);
            return GMP_EC_NACK;
        }
        CHECK_TIMEOUT(start, timeout);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout)
{
    ec_gt ret = wait_bus_idle(h, timeout);
    if (ret != GMP_EC_OK)
        return ret;

    I2C_setTargetAddress(h, dev_addr);
    reset_bus_status(h);

    uint32_t total_bytes = addr_len + mem_len;
    I2C_setDataCount(h, total_bytes);

    // Start transmission before FIFO is full (Hardware will pull data from FIFO)
    I2C_setConfig(h, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(h);
    I2C_sendStopCondition(h);

    time_gt start = gmp_base_get_system_tick();

    // 1. Send Address
    int32_t i;
    for (i = (int32_t)addr_len - 1; i >= 0; i--)
    {
        while (I2C_getTxFIFOStatus(h) == I2C_FIFO_TX16)
        {
            CHECK_TIMEOUT(start, timeout);
        }
        I2C_putData(h, (mem_addr >> (i * 8)) & 0xFF);
    }

    // 2. Send Memory Block (Continuous Push)
    // Note: mem[i] is truncated to 8-bits, automatically resolving C2000 16-bit char issues.
    uint32_t idx = 0;
    while (idx < mem_len)
    {
        if (I2C_getTxFIFOStatus(h) != I2C_FIFO_TX16)
        {
            I2C_putData(h, mem[idx] & 0xFF);
            idx++;
        }
        else
        {
            if (I2C_getStatus(h) & I2C_STS_NO_ACK)
            {
                I2C_sendStopCondition(h);
                I2C_clearStatus(h, I2C_STS_NO_ACK);
                return GMP_EC_NACK;
            }
            CHECK_TIMEOUT(start, timeout);
        }
    }

    // Wait for physical bus transmission to end
    start = gmp_base_get_system_tick();
    while (I2C_getStopConditionStatus(h))
    {
        CHECK_TIMEOUT(start, timeout);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout)
{
    if (reg_data_ret == NULL)
        return GMP_EC_GENERAL_ERROR;

    ec_gt ret = wait_bus_idle(h, timeout);
    if (ret != GMP_EC_OK)
        return ret;

    I2C_setTargetAddress(h, dev_addr);
    reset_bus_status(h);

    // ==========================================
    // Phase 1: Write Register Address (No STOP)
    // ==========================================
    I2C_setDataCount(h, addr_len);
    int32_t i;
    for (i = (int32_t)addr_len - 1; i >= 0; i--)
    {
        I2C_putData(h, (reg_addr >> (i * 8)) & 0xFF);
    }

    I2C_setConfig(h, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(h);

    time_gt start = gmp_base_get_system_tick();
    while ((I2C_getStatus(h) & I2C_STS_REG_ACCESS_RDY) == 0)
    {
        if (I2C_getStatus(h) & I2C_STS_NO_ACK)
        {
            I2C_sendStopCondition(h);
            I2C_clearStatus(h, I2C_STS_NO_ACK);
            return GMP_EC_NACK;
        }
        CHECK_TIMEOUT(start, timeout);
    }

    // ==========================================
    // Phase 2: Read Data (Repeated START + STOP)
    // ==========================================
    I2C_setDataCount(h, reg_len);
    I2C_setConfig(h, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(h);
    I2C_sendStopCondition(h);

    uint32_t result = 0;
    start = gmp_base_get_system_tick();

    for (i = 0; i < reg_len; i++)
    {
        while (I2C_getRxFIFOStatus(h) == I2C_FIFO_RX0)
        {
            if (I2C_getStatus(h) & I2C_STS_NO_ACK)
            {
                I2C_sendStopCondition(h);
                I2C_clearStatus(h, I2C_STS_NO_ACK);
                return GMP_EC_NACK;
            }
            CHECK_TIMEOUT(start, timeout);
        }
        // Assemble bytes (MSB first assumed for standard I2C)
        result = (result << 8) | I2C_getData(h);
    }

    while (I2C_getStopConditionStatus(h))
    {
        CHECK_TIMEOUT(start, timeout);
    }

    *reg_data_ret = result;
    return GMP_EC_OK;
}

ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout)
{
    if (mem == NULL)
        return GMP_EC_GENERAL_ERROR;

    ec_gt ret = wait_bus_idle(h, timeout);
    if (ret != GMP_EC_OK)
        return ret;

    I2C_setTargetAddress(h, dev_addr);
    reset_bus_status(h);

    // ==========================================
    // Phase 1: Write Memory Address (No STOP)
    // ==========================================
    I2C_setDataCount(h, addr_len);
    int32_t i;
    for (i = (int32_t)addr_len - 1; i >= 0; i--)
    {
        I2C_putData(h, (mem_addr >> (i * 8)) & 0xFF);
    }

    I2C_setConfig(h, I2C_CONTROLLER_SEND_MODE);
    I2C_sendStartCondition(h);

    time_gt start = gmp_base_get_system_tick();
    while ((I2C_getStatus(h) & I2C_STS_REG_ACCESS_RDY) == 0)
    {
        if (I2C_getStatus(h) & I2C_STS_NO_ACK)
        {
            I2C_sendStopCondition(h);
            return GMP_EC_NACK;
        }
        CHECK_TIMEOUT(start, timeout);
    }

    // ==========================================
    // Phase 2: Read Continuous Block
    // ==========================================
    I2C_setDataCount(h, mem_len);
    I2C_setConfig(h, I2C_CONTROLLER_RECEIVE_MODE);
    I2C_sendStartCondition(h);
    I2C_sendStopCondition(h);

    start = gmp_base_get_system_tick();
    uint32_t idx = 0;
    while (idx < mem_len)
    {
        if (I2C_getRxFIFOStatus(h) != I2C_FIFO_RX0)
        {
            // Direct mapping: 1 Byte on Bus -> 1 Element in data_gt array
            mem[idx] = (data_gt)I2C_getData(h);
            idx++;
        }
        else
        {
            if (I2C_getStatus(h) & I2C_STS_NO_ACK)
            {
                I2C_sendStopCondition(h);
                return GMP_EC_NACK;
            }
            CHECK_TIMEOUT(start, timeout);
        }
    }

    while (I2C_getStopConditionStatus(h))
    {
        CHECK_TIMEOUT(start, timeout);
    }

    return GMP_EC_OK;
}
