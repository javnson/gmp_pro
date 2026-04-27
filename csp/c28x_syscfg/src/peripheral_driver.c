

#include <gmp_core.h>

/**
 * @brief   Physical GPIO implementation for TI C2000 DSP family.
 * @note    This file strictly utilizes the TI C2000ware DriverLib.
 * The opaque gpio_halt is directly cast to a 32-bit pin number.
 * WARNING: Do not use GPIO0 if your higher-layer logic treats 
 * a NULL (0) handle as an "unassigned" or "hardware-managed" pin.
 */

/* ========================================================================= */
/* ==================== PRIVATE MACROS ===================================== */
/* ========================================================================= */

/**
 * @brief Safely cast the opaque handle back to a C2000 32-bit pin number.
 * @note  uintptr_t is used to safely transition from a pointer to an integer 
 * without generating compiler warnings about size differences.
 */
#define GET_C2000_PIN(hgpio) ((uint32_t)(uintptr_t)(hgpio))

/* ========================================================================= */
/* ==================== PUBLIC API IMPLEMENTATION ========================== */
/* ========================================================================= */

ec_gt gmp_hal_gpio_set_dir(gpio_halt hgpio, gpio_dir_et dir)
{
    /* Treat NULL handle safely if it's strictly considered invalid */
    if (hgpio == NULL)
    {
        /* Return OK or Error based on whether GPIO0 is permissible in your system design */
        return GMP_EC_GENERAL_ERROR;
    }

    uint32_t pin = GET_C2000_PIN(hgpio);

    if (dir == GMP_HAL_GPIO_DIR_OUT)
    {
        /* Configure pin as push-pull output */
        GPIO_setDirectionMode(pin, GPIO_DIR_MODE_OUT);

        /* Optional but recommended: Set qualification to async for outputs */
        GPIO_setQualificationMode(pin, GPIO_QUAL_ASYNC);
    }
    else
    {
        /* Configure pin as input */
        GPIO_setDirectionMode(pin, GPIO_DIR_MODE_IN);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level)
{
    if (hgpio == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint32_t pin = GET_C2000_PIN(hgpio);

    /* C2000 DriverLib inherently handles the Set/Clear registers safely */
    if (level == GMP_HAL_GPIO_HIGH)
    {
        GPIO_writePin(pin, 1);
    }
    else
    {
        GPIO_writePin(pin, 0);
    }

    return GMP_EC_OK;
}

fast_gt gmp_hal_gpio_read(gpio_halt hgpio)
{
    /* If handle is invalid, safely return 0 (LOW) */
    if (hgpio == NULL)
        return 0;

    uint32_t pin = GET_C2000_PIN(hgpio);

    /* GPIO_readPin returns 1 if HIGH, 0 if LOW */
    return (fast_gt)GPIO_readPin(pin);
}

/* ========================================================================= */
/* ==================== INLINE STATUS FUNCTIONS ============================ */
/* ========================================================================= */

GMP_STATIC_INLINE fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    /* SCI_isTransmitterBusy returns true if the shift register is active */
    return (fast_gt)SCI_isTransmitterBusy((uint32_t)uart);
}

GMP_STATIC_INLINE size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    /* Returns the current number of words (bytes) in the RX FIFO */
    return (size_gt)SCI_getRxFIFOStatus((uint32_t)uart);
}

/* ========================================================================= */
/* ==================== SAFE BLOCKING I/O FUNCTIONS ======================== */
/* ========================================================================= */

ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt* data, size_gt length, uint32_t timeout)
{
    uint32_t base = (uint32_t)uart;
    size_gt i = 0;
    time_gt time_cnt = gmp_base_get_system_tick();

    //
    // Check the arguments.
    //
    ASSERT(SCI_isBaseValid(base));

    //
    // Check if FIFO enhancement is enabled.
    //
    if(SCI_isFIFOEnabled(base))
    {
        //
        // FIFO is enabled.
        // For loop to write (Blocking) 'length' number of characters
        //
        for(i = 0U; i < length; i++)
        {
            //
            // Wait until space is available in the transmit FIFO.
            //
            while(SCI_getTxFIFOStatus(base) == SCI_FIFO_TX16)
            {
                if(gmp_base_is_delay_elapsed(time_cnt, timeout))
                    return GMP_EC_TIMEOUT; /* 硬件卡死或波特率太低，及时止损退出 */
                DEVICE_DELAY_US(1);
            }

            //
            // Send a char.
            //
            HWREGH(base + SCI_O_TXBUF) = data[i];
        }
    }
    else
    {
        //
        // FIFO is not enabled.
        // For loop to write (Blocking) 'length' number of characters
        //
        for(i = 0U; i < length; i++)
        {
            //
            // Wait until space is available in the transmit buffer.
            //
            while(!SCI_isSpaceAvailableNonFIFO(base))
            {
                if(gmp_base_is_delay_elapsed(time_cnt, timeout))
                    return GMP_EC_TIMEOUT; /* 硬件卡死或波特率太低，及时止损退出 */
                DEVICE_DELAY_US(1);
            }

            //
            // Send a char.
            //
            HWREGH(base + SCI_O_TXBUF) = data[i];
        }
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_uart_read(uart_halt uart, data_gt* data, size_gt length, uint32_t timeout, size_gt* bytes_read)
{
    uint32_t base = (uint32_t)uart;
    size_gt i = 0;
    uint32_t time_cnt;

    for (i = 0; i < length; ++i)
    {
        time_cnt = timeout;

        /* 轮询等待 RX FIFO 中出现数据 */
        while (SCI_getRxFIFOStatus(base) == SCI_FIFO_RX0)
        {
            if (--time_cnt == 0)
            {
                if (bytes_read != NULL)
                    *bytes_read = i;
                return GMP_EC_TIMEOUT; /* 未在规定时间内等到数据 */
            }
            DEVICE_DELAY_US(1);
        }

        /* 此时 FIFO 必有数据，安全读取 */
        data[i] = (data_gt)SCI_readCharNonBlocking(base);
    }

    if (bytes_read != NULL)
        *bytes_read = length;
    return GMP_EC_OK;
}

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

/**
 * @brief   Physical SPI Bus (Layer 1) implementation for TI C2000 DSP family.
 * @note    This file strictly utilizes the TI C2000ware DriverLib. 
 * It assumes the SPI peripheral is configured for 8-bit character length.
 */

/* ========================================================================= */
/* ==================== PRIVATE MACROS ===================================== */
/* ========================================================================= */

/** * @brief Helper to safely extract the physical SPI base address (e.g., SPIA_BASE) 
 * from the abstract Layer 1 bus handle.
 */
#define GET_SPI_BASE(hspi) ((uint32_t)(hspi))

/* ========================================================================= */
/* ==================== LAYER 1: PHYSICAL BUS APIs ========================= */
/* ========================================================================= */

ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt* tx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t base = GET_SPI_BASE(hspi);
    size_gt i;

    for (i = 0; i < len; i++)
    {
        time_gt start = gmp_base_get_system_tick();

        /* 1. Wait until there is space in the TX FIFO (Not Full) */
        while (SPI_getTxFIFOStatus(base) == SPI_FIFO_TX16)
        {
            if (gmp_base_is_delay_elapsed(start, timeout))
                return GMP_EC_TIMEOUT;
        }

        /* 2. Write Data. 
         * CRITICAL NOTE FOR C2000: Data written to SPITXBUF must be left-justified. 
         * Since we are operating in 8-bit mode, we shift the 8-bit payload left by 8.
         */
        SPI_writeDataNonBlocking(base, ((uint16_t)tx_buf[i]) << 8);

        /* 3. Wait until the word has been fully received into RX FIFO.
         * SPI is full-duplex; every byte sent means a byte is received. 
         * We MUST read it out to prevent RX FIFO Overflow (ROVF).
         */
        while (SPI_getRxFIFOStatus(base) == SPI_FIFO_RX0)
        {
            if (gmp_base_is_delay_elapsed(start, timeout))
                return GMP_EC_TIMEOUT;
        }

        /* 4. Discard the dummy received byte */
        SPI_readDataNonBlocking(base);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (rx_buf == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t base = GET_SPI_BASE(hspi);
    size_gt i;

    for (i = 0; i < len; i++)
    {
        time_gt start = gmp_base_get_system_tick();

        /* 1. Wait until there is space in the TX FIFO */
        while (SPI_getTxFIFOStatus(base) == SPI_FIFO_TX16)
        {
            if (gmp_base_is_delay_elapsed(start, timeout))
                return GMP_EC_TIMEOUT;
        }

        /* 2. Send Dummy Byte (0xFF) to generate the SPI clock.
         * Left-justified for 8-bit mode.
         */
        SPI_writeDataNonBlocking(base, 0xFF00);

        /* 3. Wait for the actual data to arrive in the RX FIFO */
        while (SPI_getRxFIFOStatus(base) == SPI_FIFO_RX0)
        {
            if (gmp_base_is_delay_elapsed(start, timeout))
                return GMP_EC_TIMEOUT;
        }

        /* 4. Read Data. 
         * C2000 SPIRXBUF is right-justified, so we just mask the lower 8 bits.
         */
        rx_buf[i] = (data_gt)(SPI_readDataNonBlocking(base) & 0x00FF);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL) || (rx_buf == NULL))
        return GMP_EC_GENERAL_ERROR;

    uint32_t base = GET_SPI_BASE(hspi);
    size_gt i;

    for (i = 0; i < len; i++)
    {
        time_gt start = gmp_base_get_system_tick();

        /* 1. Wait for TX space */
        while (SPI_getTxFIFOStatus(base) == SPI_FIFO_TX16)
        {
            if (gmp_base_is_delay_elapsed(start, timeout))
                return GMP_EC_TIMEOUT;
        }

        /* 2. Write actual data to bus (Left-justified for 8-bit mode) */
        SPI_writeDataNonBlocking(base, ((uint16_t)tx_buf[i]) << 8);

        /* 3. Wait for response data */
        while (SPI_getRxFIFOStatus(base) == SPI_FIFO_RX0)
        {
            if (gmp_base_is_delay_elapsed(start, timeout))
                return GMP_EC_TIMEOUT;
        }

        /* 4. Read actual data from bus (Right-justified) */
        rx_buf[i] = (data_gt)(SPI_readDataNonBlocking(base) & 0x00FF);
    }

    return GMP_EC_OK;
}

//
///**
// * @brief 物理总线写入函数
// * @details 负责将 GMP 标准报文写入 C2000 的硬件邮箱。
// */
//ec_gt gmp_hal_can_bus_write(can_halt hcan, const gmp_can_msg_t* msg)
//{
//    uint32_t base = (uint32_t)hcan;
//    uint32_t mailbox_idx;
//    bool found_free = false;
//
//    /* 1. 在预留的发送邮箱（0~3）中寻找空闲邮箱 */
//    for (mailbox_idx = 0; mailbox_idx < 4; mailbox_idx++)
//    {
//        /* 检查该邮箱的发送请求标志 (TXRQST) 是否为 0 */
//        if (!CAN_getTransmissionRequests(base, (1UL << mailbox_idx)))
//        {
//            found_free = true;
//            break;
//        }
//    }
//
//    if (!found_free)
//        return GMP_EC_BUSY;
//
//    /* 2. 将数据从 data_32[2] 搬运到硬件 */
//    /* C2000 寄存器本身是 32 位的，通过 DriverLib 接口直接写入 */
//    CAN_setupMessageObject(base, mailbox_idx + 1, msg->id, msg->is_extended ? CAN_MSG_FRAME_EXT : CAN_MSG_FRAME_STD,
//                           CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_NO_FLAGS, msg->dlc);
//
//    /* 关键优化：直接写入 32 位寄存器，绕过逐字节处理 */
//    CAN_sendMessageRaw(base, mailbox_idx + 1, msg->dlc, (uint32_t*)(msg->data_32));
//
//    return GMP_EC_OK;
//}

/**
 * @brief C2000 CAN 中断服务程序（示例）
 */
//__interrupt void can_isr(void)
//{
//    uint32_t status = CAN_getInterruptCause(base);
//
//    if (status >= 1 && status <= 4)
//    {
//        /* 发送邮箱完成中断：清除挂起位并触发 GMP 发送泵 */
//        CAN_clearInterruptStatus(base, status);
//        gmp_can_node_tx_isr_pump(&my_can_node);
//    }
//    else if (status >= 5 && status <= 32)
//    {
//        /* 接收邮箱中断 */
//        gmp_can_msg_t rx_frame;
//        /* 从硬件读出并填入 rx_frame ... (见下文) */
//        gmp_can_node_rx_isr_router(&my_can_node, &rx_frame);
//        CAN_clearInterruptStatus(base, status);
//    }
//
//    CAN_clearGlobalInterruptStatus(base, CAN_GLOBAL_INT_CANINT0);
//    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
//}
