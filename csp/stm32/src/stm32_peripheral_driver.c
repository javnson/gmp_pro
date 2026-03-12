

#include <gmp_core.h>




#ifdef HAL_UART_MODULE_ENABLED

///**
// * @brief Setup GMP UART handle.
// * This function should be called in `peripheral_mapping.c`
// * @param huart handle of GMP handle
// * @param uart_handle STM32 handle of UART
// * @param uart_tx_dma_handle STM32 DMA handle of UART TX
// * @param uart_rx_dma_handle STM32 DMA handle of UART RX
// * @param data data buffer, DMA mode only
// * @param recv_buf data buffer, DMA mode only
// */
//void gmp_hal_uart_setup(stm32_uart_t *huart, UART_HandleTypeDef *uart_handle, DMA_HandleTypeDef *uart_tx_dma_handle,
//                        DMA_HandleTypeDef *uart_rx_dma_handle, duplex_ift *data_buffer, data_gt *recv_buf)
//{
//    huart->uart_handle = uart_handle;
//    huart->uart_tx_dma_handle = uart_tx_dma_handle;
//    huart->uart_rx_dma_handle = uart_rx_dma_handle;
//
//    huart->recv_buf = recv_buf;
//    huart->buffer = data_buffer;
//}

/**
 * @brief send data via UART
 * @param huart handle of UART
 * @param data half_duplex data interface
 */
//ec_gt gmp_hal_uart_send(stm32_uart_t *huart, half_duplex_ift *data)
//{
//    assert(huart != nullptr);
//    assert(huart->uart_handle != nullptr);
//
//    assert(data != nullptr);
//
//    HAL_UART_Transmit(huart->uart_handle, data->buf, data->length, 1);
//    return GMP_EC_OK;
//}

///**
// * @brief receive data via UART
// * @param huart handle of UART
// * @param data half_duplex data interface
// */
//ec_gt gmp_hal_uart_recv(stm32_uart_t *huart, half_duplex_ift *data)
//{
//    HAL_UART_Receive(huart->uart_handle, data->buf, data->length, 1);
//    return GMP_EC_OK;
//}
//
///**
// * @brief bind a duplex data buffer to UART channel.
// * @param huart handle of UART
// * @param data duplex data buffer
// */
//ec_gt gmp_hal_uart_bind_duplex_dma(stm32_uart_t *huart, duplex_ift *data)
//{
//    huart->buffer = data;
//    return GMP_EC_OK;
//}
//
///**
// * @brief start UART listen to receive routine
// * @param huart handle of UART
// */
//ec_gt gmp_hal_uart_listen(stm32_uart_t *huart)
//{
//    HAL_UART_Receive_DMA(huart->uart_handle, (uint8_t *)huart->recv_buf, huart->buffer->capacity);
//    return GMP_EC_OK;
//}
//
///**
// * @brief Get UART listen status, return current receive bytes number.
// * @param huart
// * @return size_gt size of received bytes.
// */
//size_gt gmp_hal_uart_get_listen_status(stm32_uart_t *huart)
//{
//    size_gt data_length = huart->buffer->capacity - __HAL_DMA_GET_COUNTER(huart->uart_rx_dma_handle);
//    return data_length;
//}
//
////
///**
// * @brief This function check receive buffer and update rx_buf via receive buffer.
// * This function should be called in UART interrupt function
// * @param huart
// */
//ec_gt gmp_hal_uart_listen_routine(stm32_uart_t *uart)
//{
//    size_gt data_length;
//
//    if (__HAL_UART_GET_FLAG(uart->uart_handle, UART_FLAG_IDLE) == SET)
//    {
//        // 清除空闲标志位
//        __HAL_UART_CLEAR_IDLEFLAG(uart->uart_handle);
//
//        // 停止DMA的传输过程
//        HAL_UART_DMAStop(uart->uart_handle);
//
//        // 计算接收到的数据长度
//        data_length = uart->buffer->capacity - __HAL_DMA_GET_COUNTER(uart->uart_rx_dma_handle);
//
//        // 判定是否真的接收到数据
//        if (data_length >= 1)
//        {
//            // 此时确实有数据收到
//            // received_flag = 1;
//            // 将数据移出接收缓存，理论上应当移动到栈中
//            memcpy(uart->buffer->rx_buf, uart->recv_buf, data_length);
//        }
//
//        // 重新启动DMA接收
//        HAL_UART_Receive_DMA(uart->uart_handle, (uint8_t *)uart->recv_buf, uart->buffer->capacity);
//
//        // 启动MDA接收使能
//        __HAL_DMA_ENABLE(uart->uart_rx_dma_handle);
//
//        // 再次启用UART空闲状态的中断
//        __HAL_UART_ENABLE_IT(uart->uart_handle, UART_IT_IDLE);
//    }
//
//    return GMP_EC_OK;
//}
//
///**
// * @brief start UART consign to transmit routine.
// * @param huart handle of UART
// */
//ec_gt gmp_hal_uart_consign(stm32_uart_t *huart)
//{
//    HAL_StatusTypeDef stat;
//
//    // judge if a buffer has bind to the object
//    assert(huart != nullptr);
//
//    if (huart->buffer == nullptr || huart->buffer->tx_buf == nullptr)
//        // ignore this error
//        return GMP_EC_OK;
//
//    // Call DMA to send these data
//    if (HAL_DMA_GetState(huart->uart_tx_dma_handle) == HAL_DMA_STATE_READY)
//    {
//        stat = HAL_UART_Transmit_DMA(huart->uart_handle, huart->buffer->tx_buf, huart->buffer->length);
//    }
//
//   return GMP_EC_OK;
//    // if (stat == HAL_OK)
//    //     return content->length;
//    // else
//    //     return 0;
//}
//
///**
// * @brief Get UART consign status, return if consign routine is free.
// * @param huart
// * @return fast_gt
// */
//fast_gt gmp_hal_uart_get_consign_status(stm32_uart_t *huart)
//{
//    if (HAL_DMA_GetState(huart->uart_tx_dma_handle) == HAL_DMA_STATE_READY)
//        return 1; // DMA has released
//    else
//        return 0; // DMA is still in using
//}
//
#endif // HAL_UART_MODULE_ENABLED


#ifdef HAL_I2C_MODULE_ENABLED

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

#endif //HAL_I2C_MODULE_ENABLED

////////////////////////////////////////////////////////////////////////
// SPI Model

#if defined HAL_SPI_MODULE_ENABLED

/**
 * @brief   Helper function to map STM32 HAL status to GMP Error Codes.
 *
 * @param[in] hal_status Status returned by STM32 HAL functions.
 * @return  ec_gt        Mapped general error code.
 */
static ec_gt stm32_hal_status_to_ec(HAL_StatusTypeDef hal_status)
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
        return GMP_EC_GENERAL_ERROR;
    }
}

/* ========================================================================= */
/* ==================== LAYER 1: PHYSICAL BUS APIs ========================= */
/* ========================================================================= */

ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt* tx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    /* Cast the opaque handle to the STM32 specific SPI handle */
    SPI_HandleTypeDef* hspi_stm32 = (SPI_HandleTypeDef*)hspi;

    /* STM32 HAL automatically handles the TX FIFO and waiting flags */
    HAL_StatusTypeDef res = HAL_SPI_Transmit(hspi_stm32, (uint8_t*)tx_buf, (uint16_t)len, (uint32_t)timeout);

    return stm32_hal_status_to_ec(res);
}

ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    SPI_HandleTypeDef* hspi_stm32 = (SPI_HandleTypeDef*)hspi;

    /* STM32 HAL_SPI_Receive automatically transmits dummy bytes (0xFF or 0x00) 
     * on the MOSI line to generate the SCK clock, while capturing data from MISO.
     */
    HAL_StatusTypeDef res = HAL_SPI_Receive(hspi_stm32, (uint8_t*)rx_buf, (uint16_t)len, (uint32_t)timeout);

    return stm32_hal_status_to_ec(res);
}

ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL) || (rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    SPI_HandleTypeDef* hspi_stm32 = (SPI_HandleTypeDef*)hspi;

    /* Full-duplex transfer. STM32 HAL handles shifting data out from tx_buf 
     * while simultaneously shifting data into rx_buf.
     */
    HAL_StatusTypeDef res =
        HAL_SPI_TransmitReceive(hspi_stm32, (uint8_t*)tx_buf, (uint8_t*)rx_buf, (uint16_t)len, (uint32_t)timeout);

    return stm32_hal_status_to_ec(res);
}

#endif // HAL_SPI_MODULE_ENABLED


////////////////////////////////////////////////////////////////////////
// CAN Model

#if defined HAL_CAN_MODULE_ENABLED

//
///**
// * @brief 物理层写入函数
// * @details 适配 STM32 的 3 邮箱发送机制。
// */
//ec_gt gmp_hal_can_bus_write(can_halt hcan, const gmp_can_msg_t* msg)
//{
//    CAN_HandleTypeDef* hcan_stm32 = (CAN_HandleTypeDef*)hcan;
//    CAN_TxHeaderTypeDef tx_header;
//    uint32_t tx_mailbox;
//
//    /* 1. 检查是否有空闲的发送邮箱 */
//    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_stm32) == 0)
//    {
//        return GMP_EC_BUSY; /* 硬件邮箱全满，返回繁忙，触发 GMP Core 入队 */
//    }
//
//    /* 2. 准备 STM32 格式的报文头 */
//    tx_header.StdId = msg->is_extended ? 0 : msg->id;
//    tx_header.ExtId = msg->is_extended ? msg->id : 0;
//    tx_header.IDE = msg->is_extended ? CAN_ID_EXT : CAN_ID_STD;
//    tx_header.RTR = msg->is_remote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
//    tx_header.DLC = msg->dlc;
//    tx_header.TransmitGlobalTime = DISABLE;
//
//    /* 3. 搬运负载数据 */
//    /* 由于 STM32 HAL 要求 uint8_t 数组，我们直接利用 data_32 的连续内存强转 */
//    uint8_t* p_payload = (uint8_t*)(msg->data_32);
//
//    /* 4. 调用 STM32 HAL 写入邮箱 */
//    if (HAL_CAN_AddTxMessage(hcan_stm32, &tx_header, p_payload, &tx_mailbox) != HAL_OK)
//    {
//        return GMP_EC_GENERAL_ERROR;
//    }
//
//    return GMP_EC_OK;
//}
//
///**
// * @brief STM32 发送完成回调
// */
//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan)
//{
//    /* 找到该硬件总线对应的 gmp_can_node 实例，并触发 Pump */
//    /* 假设我们在 BSP 层维护了从 hcan 到 gmp_node 的映射 */
//    gmp_can_node_t* node = bsp_get_can_node_from_handle(hcan);
//    if (node)
//    {
//        gmp_can_node_tx_isr_pump(node);
//    }
//}
//
///**
// * @brief STM32 接收回调
// */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
//{
//    CAN_RxHeaderTypeDef rx_header;
//    gmp_can_msg_t rx_msg;
//    uint8_t rx_raw_data[8];
//
//    /* 1. 从硬件 FIFO 中取出原始报文 */
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_raw_data) == HAL_OK)
//    {
//        /* 2. 转换为 GMP 标准格式 */
//        rx_msg.id = (rx_header.IDE == CAN_ID_STD) ? rx_header.StdId : rx_header.ExtId;
//        rx_msg.is_extended = (rx_header.IDE == CAN_ID_EXT);
//        rx_msg.is_remote = (rx_header.RTR == CAN_RTR_REMOTE);
//        rx_msg.dlc = rx_header.DLC;
//
//        /* 3. 安全拷贝负载 (考虑 32 位对齐) */
//        rx_msg.data_32[0] = ((uint32_t*)rx_raw_data)[0];
//        rx_msg.data_32[1] = ((uint32_t*)rx_raw_data)[1];
//
//        /* 4. 调用 GMP 路由引擎 */
//        gmp_can_node_t* node = bsp_get_can_node_from_handle(hcan);
//        if (node)
//        {
//            gmp_can_node_rx_isr_router(node, &rx_msg);
//        }
//    }
//}


#endif // HAL_CAN_MODULE_ENABLED
