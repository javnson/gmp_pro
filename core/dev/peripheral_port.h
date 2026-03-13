
#ifndef _FILE_PERIPHERAL_PORT_H_
#define _FILE_PERIPHERAL_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// typedef struct _tag_ring_buffer_t
//{
//     // buffer
//     data_gt *buffer;

//    // the position to read
//    size_gt read_pos;

//    // the write position
//    size_gt write_pos;

//    // length of buffer, the maximum length is `length - 1`
//    size_gt length;

//    fast_gt full;

//} ring_buffer_t;

//////////////////////////////////////////////////////////////////////////
// Virtual Port

// void gmp_hal_make_half_duplex_if(half_duplex_ift *if, data_gt *data, size_gt length);

//////////////////////////////////////////////////////////////////////////
// Async Port

// Write data to the ring buffer
//GMP_STATIC_INLINE
//void gmp_hal_buffer_write(ringbuf_t *ring, const data_gt *data, size_gt length)
//{
//    ringbuf_put_array(ring, data, length);
//}
//
//// Read data from the ring buffer
//GMP_STATIC_INLINE
//size_gt gmp_hal_buffer_read(ringbuf_t *ring, data_gt *data, size_gt length)
//{
//    size_gt max_length = ringbuf_get_valid_size(ring);
//    if (max_length == 0)
//        return 0;
//
//    ringbuf_get_array(ring, data, max_length);
//    return max_length
//}

// General Peripheral Prototype functions
// These functions may implement by CSP.

/* ========================================================================= */
/* ==================== TYPES & DEFINITIONS ================================ */
/* ========================================================================= */

/** @brief GPIO Direction Configuration */
typedef enum
{
    GMP_HAL_GPIO_DIR_IN = 0, /**< Configure pin as Input */
    GMP_HAL_GPIO_DIR_OUT = 1 /**< Configure pin as Output */
} gpio_dir_et;

/** @brief GPIO Logic Levels */
#define GMP_HAL_GPIO_LOW  (0)
#define GMP_HAL_GPIO_HIGH (1)

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief   Set the direction of the specified GPIO pin.
 * * @param[in] hgpio     The GPIO hardware handle.
 * @param[in] dir       The desired direction (Input or Output).
 * * @return  ec_gt       Error code (GMP_EC_OK on success).
 */
ec_gt gmp_hal_gpio_set_dir(gpio_halt hgpio, gpio_dir_et dir);

/**
 * @brief   Write a logic level to the specified GPIO pin.
 * * @param[in] hgpio     The GPIO hardware handle.
 * @param[in] level     Logic level to write (GMP_HAL_GPIO_LOW or GMP_HAL_GPIO_HIGH).
 * * @return  ec_gt       Error code (GMP_EC_OK on success).
 */
ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level);

/**
 * @brief   Read the current logic level of the specified GPIO pin.
 * * @param[in] hgpio     The GPIO hardware handle.
 * * @return  fast_gt     Current logic level (0 for LOW, 1 for HIGH). Returns 0 on invalid handle.
 */
fast_gt gmp_hal_gpio_read(gpio_halt hgpio);

//////////////////////////////////////////////////////////////////////////
// SPI interface

/**
 * @brief   Transmit data over the physical SPI bus without CS management.
 * * @param[in] hspi      The physical SPI bus handle.
 * @param[in] tx_buf    Pointer to the data to transmit.
 * @param[in] len       Number of data units (bytes) to transmit.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt* tx_buf, size_gt len, time_gt timeout);

/**
 * @brief   Receive data from the physical SPI bus without CS management.
 * * @param[in]  hspi     The physical SPI bus handle.
 * @param[out] rx_buf   Pointer to the buffer to store received data.
 * @param[in]  len      Number of data units (bytes) to receive.
 * @param[in]  timeout  Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt* rx_buf, size_gt len, time_gt timeout);

/**
 * @brief   Simultaneous Transmit and Receive (Full-Duplex) over the physical SPI bus.
 * * @param[in]  hspi     The physical SPI bus handle.
 * @param[in]  tx_buf   Pointer to the data to transmit.
 * @param[out] rx_buf   Pointer to the buffer to store received data.
 * @param[in]  len      Number of data units to exchange.
 * @param[in]  timeout  Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout);

/* ========================================================================= */
/* ==================== LAYER 2: LOGICAL SPI DEVICE ======================== */
/* ========================================================================= */

/**
 * @brief Structure defining a Logical SPI Device.
 * @note  A logical device binds a physical SPI bus to a specific Chip Select (CS) pin.
 */
typedef struct
{
    spi_halt bus;     /**< The physical SPI bus this device is attached to. */
    gpio_halt cs_pin; /**< The CS GPIO handle. If NULL, CS is assumed to be hardware-managed. */
} gmp_spi_dev_t;

/** @brief Handle pointer for a Logical SPI Device. */
typedef gmp_spi_dev_t* spi_device_halt;

/* -------------------------------------------------------------------------
 * Device Level Convenience APIs (Automatic CS Management)
 * ------------------------------------------------------------------------- */

/**
 * @brief   Write a 8/16/24/32-bit frame to the device.
 * @note    Automatically asserts and de-asserts the CS pin. Data is MSB-first.
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_write_8b(spi_device_halt hdev, uint8_t data, time_gt timeout)
{
    if (hdev == NULL)
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt tx_buf[1];

    tx_buf[0] = (data_gt)data;

    /* 1. Assert CS (If configured for Software CS) */
    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    /* 2. Transmit over Physical Bus */
    ret = gmp_hal_spi_bus_write(dev->bus, tx_buf, 1, timeout);

    /* 3. De-assert CS */
    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    return ret;
}

GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_write_16b(spi_device_halt hdev, uint16_t data, time_gt timeout)
{
    if (hdev == NULL)
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt tx_buf[2];

    /* Serialize to MSB First: 
     * E.g., data = 0xABCD -> tx_buf[0] = 0xAB, tx_buf[1] = 0xCD 
     */
    tx_buf[0] = (data_gt)((data >> 8) & 0xFF);
    tx_buf[1] = (data_gt)(data & 0xFF);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    ret = gmp_hal_spi_bus_write(dev->bus, tx_buf, 2, timeout);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    return ret;
}

GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_write_24b(spi_device_halt hdev, uint32_t data, time_gt timeout)
{
    if (hdev == NULL)
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt tx_buf[3];

    /* Serialize to MSB First, extracting only the lower 24 bits */
    tx_buf[0] = (data_gt)((data >> 16) & 0xFF);
    tx_buf[1] = (data_gt)((data >> 8) & 0xFF);
    tx_buf[2] = (data_gt)(data & 0xFF);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    ret = gmp_hal_spi_bus_write(dev->bus, tx_buf, 3, timeout);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    return ret;
}

GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_write_32b(spi_device_halt hdev, uint32_t data, time_gt timeout)
{
    if (hdev == NULL)
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt tx_buf[4];

    /* Serialize to MSB First */
    tx_buf[0] = (data_gt)((data >> 24) & 0xFF);
    tx_buf[1] = (data_gt)((data >> 16) & 0xFF);
    tx_buf[2] = (data_gt)((data >> 8) & 0xFF);
    tx_buf[3] = (data_gt)(data & 0xFF);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    ret = gmp_hal_spi_bus_write(dev->bus, tx_buf, 4, timeout);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    return ret;
}

/**
 * @brief   Read a 8/16/24/32-bit frame from the device.
 * @note    Automatically asserts and de-asserts the CS pin. Data is MSB-first.
 */
/* ========================================================================= */
/* ==================== READ APIs ========================================== */
/* ========================================================================= */

/**
 * @brief   Read an 8-bit frame from the logical SPI device.
 * @param   hdev        The logical SPI device handle.
 * @param   data_ret    Pointer to store the received 8-bit data.
 * @param   timeout     Maximum timeout in milliseconds.
 * @return  ec_gt       Error code (GMP_EC_OK on success).
 */
GMP_STATIC_INLINE
ec_gt gmp_hal_spi_dev_read_8b(spi_device_halt hdev, data_gt* data_ret, time_gt timeout)
{
    if ((hdev == NULL) || (data_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt rx_buf[1] = {0};

    /* 1. Assert CS (If configured for Software CS) */
    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    /* 2. Receive over Physical Bus */
    ret = gmp_hal_spi_bus_read(dev->bus, rx_buf, 1, timeout);

    /* 3. De-assert CS */
    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    /* 4. Assign data if successful */
    if (ret == GMP_EC_OK)
    {
        *data_ret = (uint8_t)rx_buf[0];
    }

    return ret;
}

/**
 * @brief   Read a 16-bit frame from the logical SPI device.
 * @note    Reconstructs the MSB-first SPI data stream into a platform-native 16-bit integer.
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_read_16b(spi_device_halt hdev, uint16_t* data_ret, time_gt timeout)
{
    if ((hdev == NULL) || (data_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt rx_buf[2] = {0};

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    ret = gmp_hal_spi_bus_read(dev->bus, rx_buf, 2, timeout);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    /* Reconstruct 16-bit word (MSB First) 
     * E.g., Received [0]=0xAB, [1]=0xCD -> Result = 0xABCD
     */
    if (ret == GMP_EC_OK)
    {
        *data_ret = (uint16_t)(((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1]);
    }

    return ret;
}

/**
 * @brief   Read a 24-bit frame from the logical SPI device.
 * @note    The 24-bit result is stored in the lower 24 bits of the 32-bit return variable.
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_read_24b(spi_device_halt hdev, uint32_t* data_ret, time_gt timeout)
{
    if ((hdev == NULL) || (data_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt rx_buf[3] = {0};

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    ret = gmp_hal_spi_bus_read(dev->bus, rx_buf, 3, timeout);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    /* Reconstruct 24-bit word (MSB First) */
    if (ret == GMP_EC_OK)
    {
        *data_ret = (uint32_t)(((uint32_t)rx_buf[0] << 16) | ((uint32_t)rx_buf[1] << 8) | (uint32_t)rx_buf[2]);
    }

    return ret;
}

/**
 * @brief   Read a 32-bit frame from the logical SPI device.
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_read_32b(spi_device_halt hdev, uint32_t* data_ret, time_gt timeout)
{
    if ((hdev == NULL) || (data_ret == NULL))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt rx_buf[4] = {0};

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);
    }

    ret = gmp_hal_spi_bus_read(dev->bus, rx_buf, 4, timeout);

    if (dev->cs_pin != NULL)
    {
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);
    }

    /* Reconstruct 32-bit word (MSB First) */
    if (ret == GMP_EC_OK)
    {
        *data_ret = (uint32_t)(((uint32_t)rx_buf[0] << 24) | ((uint32_t)rx_buf[1] << 16) | ((uint32_t)rx_buf[2] << 8) |
                               (uint32_t)rx_buf[3]);
    }

    return ret;
}

/* ========================================================================= */
/* ==================== COMPOSITE & REGISTER APIs ========================== */
/* ========================================================================= */

/**
 * @brief   Sequential Write-then-Read (Inquiry) within a single CS frame.
 * @note    Sequence: Assert CS -> Transmit Command -> Receive Data -> De-assert CS.
 * This is the ultimate function for reading sensor registers where a command/address 
 * must be sent before the sensor clocks out data.
 *
 * @param[in]  hdev      The logical SPI device handle.
 * @param[in]  cmd_buf   Pointer to the command/address bytes to send.
 * @param[in]  cmd_len   Length of the command buffer in bytes.
 * @param[out] rx_buf    Pointer to the buffer to receive data.
 * @param[in]  rx_len    Length of data to read in bytes.
 * @param[in]  timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt        Error code (GMP_EC_OK on success).
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_write_then_read(spi_device_halt hdev, const data_gt* cmd_buf, size_gt cmd_len, data_gt* rx_buf,
                                      size_gt rx_len, time_gt timeout)
{
    if ((hdev == NULL) || (cmd_buf == NULL) || (rx_buf == NULL))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);

    /* 1. Send the command/address */
    ret = gmp_hal_spi_bus_write(dev->bus, cmd_buf, cmd_len, timeout);

    /* 2. Read the response sequentially while CS is still low */
    if (ret == GMP_EC_OK)
    {
        ret = gmp_hal_spi_bus_read(dev->bus, rx_buf, rx_len, timeout);
    }

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);

    return ret;
}

/**
 * @brief   Write to a specific register on the SPI device.
 * @note    Serializes up to 4 bytes of address and 4 bytes of data (MSB-first) 
 * and transmits them sequentially within a single CS frame.
 *
 * @param[in] hdev       The logical SPI device handle.
 * @param[in] addr       Register address to write to.
 * @param[in] addr_len   Length of the address in bytes (max 4).
 * @param[in] data       Data payload to write to the register.
 * @param[in] data_len   Length of the data payload in bytes (max 4).
 * @param[in] timeout    Maximum timeout in milliseconds.
 * * @return  ec_gt        Error code (GMP_EC_OK on success).
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_write_reg(spi_device_halt hdev, addr32_gt addr, size_gt addr_len, uint32_t data, size_gt data_len,
                                time_gt timeout)
{
    if (hdev == NULL)
        return GMP_EC_GENERAL_ERROR;
    if ((addr_len > 4) || (data_len > 4))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt tx_buf[8] = {0}; /* Max 4 bytes address + 4 bytes data */
    uint32_t i;

    /* 1. Serialize Address (MSB First) */
    for (i = 0; i < addr_len; i++)
    {
        tx_buf[i] = (data_gt)((addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }

    /* 2. Serialize Data (MSB First) */
    for (i = 0; i < data_len; i++)
    {
        tx_buf[addr_len + i] = (data_gt)((data >> ((data_len - 1 - i) * 8)) & 0xFF);
    }

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);

    /* Push the combined buffer out continuously */
    ret = gmp_hal_spi_bus_write(dev->bus, tx_buf, addr_len + data_len, timeout);

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);

    return ret;
}

/**
 * @brief   Read from a specific register on the SPI device.
 * @note    Transmits the serialized address (MSB-first), then clocks out 
 * the data and reconstructs it into a 32-bit integer (MSB-first).
 *
 * @param[in]  hdev      The logical SPI device handle.
 * @param[in]  addr      Register address to read from.
 * @param[in]  addr_len  Length of the address in bytes (max 4).
 * @param[out] data      Pointer to store the reconstructed read data.
 * @param[in]  data_len  Length of the expected data payload in bytes (max 4).
 * @param[in]  timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt        Error code (GMP_EC_OK on success).
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_read_reg(spi_device_halt hdev, addr32_gt addr, size_gt addr_len, uint32_t* data, size_gt data_len,
                               time_gt timeout)
{
    if ((hdev == NULL) || (data == NULL))
        return GMP_EC_GENERAL_ERROR;
    if ((addr_len > 4) || (data_len > 4))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;
    data_gt tx_cmd_buf[4] = {0};
    data_gt rx_data_buf[4] = {0};
    uint32_t i;

    /* Serialize Address (MSB First) */
    for (i = 0; i < addr_len; i++)
    {
        tx_cmd_buf[i] = (data_gt)((addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);

    /* Send Address */
    ret = gmp_hal_spi_bus_write(dev->bus, tx_cmd_buf, addr_len, timeout);

    /* Receive Data */
    if (ret == GMP_EC_OK)
    {
        ret = gmp_hal_spi_bus_read(dev->bus, rx_data_buf, data_len, timeout);
    }

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);

    /* Reconstruct Data (MSB First) */
    if (ret == GMP_EC_OK)
    {
        uint32_t val = 0;
        for (i = 0; i < data_len; i++)
        {
            val = (val << 8) | rx_data_buf[i];
        }
        *data = val;
    }

    return ret;
}

/**
 * @brief   Simultaneous Transmit and Receive (Full-Duplex Transfer) within a single CS frame.
 * @note    Commonly used for full-duplex ADC chips or daisy-chained SPI setups.
 *
 * @param[in]  hdev      The logical SPI device handle.
 * @param[in]  tx_buf    Pointer to the data to transmit.
 * @param[out] rx_buf    Pointer to the buffer to store received data.
 * @param[in]  len       Number of bytes to exchange.
 * @param[in]  timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt        Error code (GMP_EC_OK on success).
 */
GMP_STATIC_INLINE ec_gt gmp_hal_spi_dev_transfer(spi_device_halt hdev, const data_gt* tx_buf, data_gt* rx_buf, size_gt len,
                               time_gt timeout)
{
    if ((hdev == NULL) || (tx_buf == NULL) || (rx_buf == NULL))
        return GMP_EC_GENERAL_ERROR;

    gmp_spi_dev_t* dev = (gmp_spi_dev_t*)hdev;
    ec_gt ret;

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_LOW);

    /* Trigger the physical full-duplex transfer */
    ret = gmp_hal_spi_bus_transfer(dev->bus, tx_buf, rx_buf, len, timeout);

    if (dev->cs_pin != NULL)
        gmp_hal_gpio_write(dev->cs_pin, GMP_HAL_GPIO_HIGH);

    return ret;
}

//////////////////////////////////////////////////////////////////////////
// UART interface

/**
 * @brief  Transmits a stream of characters over UART.
 * @note   This function blocks until all data is written to the hardware FIFO 
 * or the timeout expires.
 * * @param[in]  uart     The physical UART hardware handle (e.g., SCIA_BASE).
 * @param[in]  data     Pointer to the data buffer to be transmitted.
 * @param[in]  length   Number of bytes to transmit.
 * @param[in]  timeout  Maximum time to wait for transmission (in milliseconds or ticks).
 * * @return ec_gt        GMP_EC_OK if all bytes are transmitted.
 * GMP_EC_TIMEOUT if the operation timed out before completion.
 */
ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt* data, size_gt length, uint32_t timeout);

/**
 * @brief  Receives a stream of characters from UART.
 * @note   This function blocks until the requested length is received 
 * or the timeout expires.
 * * @param[in]  uart         The physical UART hardware handle.
 * @param[out] data         Pointer to the buffer to store received data.
 * @param[in]  length       Number of bytes requested to read.
 * @param[in]  timeout      Maximum time to wait for reception.
 * @param[out] bytes_read   (Optional) Pointer to store the actual number of bytes read.
 * * @return ec_gt            GMP_EC_OK if exactly 'length' bytes were read.
 * GMP_EC_TIMEOUT if fewer bytes were read before timeout.
 */
ec_gt gmp_hal_uart_read(uart_halt uart, data_gt* data, size_gt length, uint32_t timeout, size_gt* bytes_read);

size_gt gmp_base_print_default(const char* p_fmt, ...);

/* ========================================================================= */
/* ==================== STATUS CHECK APIs ================================== */
/* ========================================================================= */

/**
 * @brief  Checks if the UART Transmitter is currently busy.
 * @note   Useful for checking if the physical line is clear before entering sleep mode.
 * * @param[in]  uart     The physical UART hardware handle.
 * @return fast_gt      1 (true) if busy shifting data out, 0 (false) if idle.
 */
GMP_STATIC_INLINE fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart);

/**
 * @brief  Gets the number of unread bytes available in the hardware RX FIFO.
 * @note   Allows the user to safely call `gmp_hal_uart_read` without blocking.
 * * @param[in]  uart     The physical UART hardware handle.
 * @return size_gt      Number of bytes currently available to read.
 */
GMP_STATIC_INLINE size_gt gmp_hal_uart_get_rx_available(uart_halt uart);

// size_gt gmp_hal_uart_read_async(uart_halt uart, data_gt *data, size_gt length);
//void gmp_hal_uart_write_async(uart_halt uart, const data_gt* data, size_gt length);

// wait till transmit/receive complete.
fast_gt gmp_hal_uart_is_busy(uart_halt uart);

//////////////////////////////////////////////////////////////////////////
// IIC interface

/**
 * @brief   Write a command to the I2C device without register address.
 * * @param[in] h         I2C hardware handle.
 * @param[in] dev_addr  7-bit right-aligned device address.
 * @param[in] cmd       The command value to be sent.
 * @param[in] cmd_len   Length of the command in bytes.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code (GMP_EC_OK on success).
 */
ec_gt gmp_hal_iic_write_cmd(iic_halt h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout);

/**
 * @brief   Write a single value to a specific register of the I2C device.
 * * @param[in] h         I2C hardware handle.
 * @param[in] dev_addr  7-bit right-aligned device address.
 * @param[in] reg_addr  Register address.
 * @param[in] addr_len  Length of the register address in bytes (e.g., 1 or 2).
 * @param[in] reg_data  Data to write into the register.
 * @param[in] reg_len   Length of the data in bytes.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout);

/**
 * @brief   Write a continuous memory block to the I2C device.
 * * @param[in] h         I2C hardware handle.
 * @param[in] dev_addr  7-bit right-aligned device address.
 * @param[in] mem_addr  Starting memory/register address.
 * @param[in] addr_len  Length of the memory address in bytes.
 * @param[in] mem       Pointer to the data buffer.
 * @param[in] mem_len   Number of data_gt elements to write.
 * @param[in] timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt       Error code.
 */
ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout);

/**
 * @brief   Read a single value from a specific register of the I2C device.
 * * @param[in]  h            I2C hardware handle.
 * @param[in]  dev_addr     7-bit right-aligned device address.
 * @param[in]  reg_addr     Register address.
 * @param[in]  addr_len     Length of the register address in bytes.
 * @param[out] reg_data_ret Pointer to store the read data.
 * @param[in]  reg_len      Length of the data to read in bytes.
 * @param[in]  timeout      Maximum timeout in milliseconds.
 * * @return  ec_gt           Error code.
 */
ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout);

/**
 * @brief   Read a continuous memory block from the I2C device.
 * * @param[in]  h         I2C hardware handle.
 * @param[in]  dev_addr  7-bit right-aligned device address.
 * @param[in]  mem_addr  Starting memory/register address.
 * @param[in]  addr_len  Length of the memory address in bytes.
 * @param[out] mem       Pointer to the data buffer to store results.
 * @param[in]  mem_len   Number of data_gt elements to read.
 * @param[in]  timeout   Maximum timeout in milliseconds.
 * * @return  ec_gt        Error code.
 */
ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout);

//////////////////////////////////////////////////////////////////////////
// CAN interface

/* ========================================================================= */
/* ==================== CAN MESSAGE STRUCTURE ============================== */
/* ========================================================================= */

/**
 * @brief Standardized CAN Message Structure.
 * @note  Total size is strictly 16 bytes (assuming 32-bit CPU with 4-byte alignment).
 * The explicit layout prevents compiler padding inconsistencies.
 */
typedef struct
{
    uint32_t id;      /**< @brief CAN ID (11-bit Standard or 29-bit Extended) */
    fast_gt is_extended; /**< @brief True if the ID is 29-bit Extended */
    fast_gt is_remote;   /**< @brief True if this is a Remote Transmission Request (RTR) */
    fast_gt dlc;      /**< @brief Data Length Code (0 to 8) */

    /** * @brief 64-bit Payload.
     * @note  data_32[0] contains logical bytes 0 to 3 (Little-Endian layout).
     * data_32[1] contains logical bytes 4 to 7 (Little-Endian layout).
     */
    uint32_t data_32[2];
} gmp_can_msg_t;

/* ========================================================================= */
/* ==================== PAYLOAD ATOMIC PRIMITIVES (8-BIT) ================== */
/* ========================================================================= */

/**
 * @brief Extract a single logical byte (8-bit) from the payload.
 * @param msg    Pointer to the CAN message.
 * @param offset Byte offset (0 to 7).
 * @return       The 8-bit value at the specified offset.
 */
GMP_STATIC_INLINE uint8_t gmp_can_payload_get_u8(const gmp_can_msg_t* msg, uint8_t offset)
{
    if (offset < 4)
    {
        return (uint8_t)((msg->data_32[0] >> (offset * 8)) & 0xFFU);
    }
    else if (offset < 8)
    {
        return (uint8_t)((msg->data_32[1] >> ((offset - 4) * 8)) & 0xFFU);
    }
    return 0;
}

/**
 * @brief Inject a single logical byte (8-bit) into the payload.
 * @param msg    Pointer to the CAN message.
 * @param offset Byte offset (0 to 7).
 * @param val    The 8-bit value to inject.
 */
GMP_STATIC_INLINE void gmp_can_payload_set_u8(gmp_can_msg_t* msg, uint8_t offset, uint8_t val)
{
    if (offset < 4)
    {
        /* Clear the target 8 bits, then OR the new value */
        msg->data_32[0] &= ~(0xFFUL << (offset * 8));
        msg->data_32[0] |= ((uint32_t)val << (offset * 8));
    }
    else if (offset < 8)
    {
        msg->data_32[1] &= ~(0xFFUL << ((offset - 4) * 8));
        msg->data_32[1] |= ((uint32_t)val << ((offset - 4) * 8));
    }
}

/* ========================================================================= */
/* ==================== PAYLOAD COMPOSITE GETTERS ========================== */
/* ========================================================================= */

/**
 * @brief Read a 16-bit integer (Little-Endian) starting at any offset (0-6).
 * @note  Safe against unaligned memory access and cross-word boundaries (e.g., offset 3).
 */
GMP_STATIC_INLINE uint16_t gmp_can_payload_get_u16(const gmp_can_msg_t* msg, uint8_t offset)
{
    return (uint16_t)gmp_can_payload_get_u8(msg, offset) | ((uint16_t)gmp_can_payload_get_u8(msg, offset + 1) << 8);
}

/**
 * @brief Read a 32-bit integer (Little-Endian) starting at any offset (0-4).
 */
GMP_STATIC_INLINE uint32_t gmp_can_payload_get_u32(const gmp_can_msg_t* msg, uint8_t offset)
{
    return (uint32_t)gmp_can_payload_get_u8(msg, offset) | ((uint32_t)gmp_can_payload_get_u8(msg, offset + 1) << 8) |
           ((uint32_t)gmp_can_payload_get_u8(msg, offset + 2) << 16) |
           ((uint32_t)gmp_can_payload_get_u8(msg, offset + 3) << 24);
}

/**
 * @brief Read a 32-bit floating point number (IEEE 754) starting at any offset (0-4).
 */
GMP_STATIC_INLINE float gmp_can_payload_get_f32(const gmp_can_msg_t* msg, uint8_t offset)
{
    union {
        uint32_t u;
        float f;
    } conv;
    conv.u = gmp_can_payload_get_u32(msg, offset);
    return conv.f;
}

/* ========================================================================= */
/* ==================== PAYLOAD COMPOSITE SETTERS ========================== */
/* ========================================================================= */

/**
 * @brief Write a 16-bit integer (Little-Endian) starting at any offset (0-6).
 */
GMP_STATIC_INLINE void gmp_can_payload_set_u16(gmp_can_msg_t* msg, uint8_t offset, uint16_t val)
{
    gmp_can_payload_set_u8(msg, offset, (uint8_t)(val & 0xFFU));
    gmp_can_payload_set_u8(msg, offset + 1, (uint8_t)((val >> 8) & 0xFFU));
}

/**
 * @brief Write a 32-bit integer (Little-Endian) starting at any offset (0-4).
 */
GMP_STATIC_INLINE void gmp_can_payload_set_u32(gmp_can_msg_t* msg, uint8_t offset, uint32_t val)
{
    gmp_can_payload_set_u8(msg, offset, (uint8_t)(val & 0xFFU));
    gmp_can_payload_set_u8(msg, offset + 1, (uint8_t)((val >> 8) & 0xFFU));
    gmp_can_payload_set_u8(msg, offset + 2, (uint8_t)((val >> 16) & 0xFFU));
    gmp_can_payload_set_u8(msg, offset + 3, (uint8_t)((val >> 24) & 0xFFU));
}

/**
 * @brief Write a 32-bit floating point number (IEEE 754) starting at any offset (0-4).
 * @note  Strict-aliasing safe implementation using union.
 */
GMP_STATIC_INLINE void gmp_can_payload_set_f32(gmp_can_msg_t* msg, uint8_t offset, float val)
{
    union {
        float f;
        uint32_t u;
    } conv;
    conv.f = val;
    gmp_can_payload_set_u32(msg, offset, conv.u);
}

/**
 * @brief Static Circular Deque Structure.
 */
typedef struct
{
    gmp_can_msg_t* buffer; /**< Pointer to the static array storage. */
    uint16_t capacity;     /**< Maximum number of elements the deque can hold. */
    uint16_t head;         /**< Index of the front element. */
    uint16_t tail;         /**< Index of the next available slot at the rear. */
    uint16_t count;        /**< Current number of elements in the deque. */
} gmp_can_deque_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief Initializes the CAN message deque with a static buffer.
 * * @param[out] dq       Pointer to the deque structure.
 * @param[in]  buf      Pointer to a static array of gmp_can_msg_t.
 * @param[in]  cap      Capacity of the buffer.
 */
void gmp_can_deque_init(gmp_can_deque_t* dq, gmp_can_msg_t* buf, uint16_t cap);

/**
 * @brief Inserts a message at the end of the deque (Normal priority).
 * * @param[in,out] dq    Pointer to the deque structure.
 * @param[in]     msg   Pointer to the message to be inserted.
 * @return ec_gt        GMP_EC_OK on success, or GMP_EC_DEQUE_FULL.
 */
ec_gt gmp_can_deque_push_back(gmp_can_deque_t* dq, const gmp_can_msg_t* msg);

/**
 * @brief Inserts a message at the front of the deque (High priority).
 * * @param[in,out] dq    Pointer to the deque structure.
 * @param[in]     msg   Pointer to the message to be inserted.
 * @return ec_gt        GMP_EC_OK on success, or GMP_EC_DEQUE_FULL.
 */
ec_gt gmp_can_deque_push_front(gmp_can_deque_t* dq, const gmp_can_msg_t* msg);

/**
 * @brief Removes and retrieves a message from the front of the deque.
 * * @param[in,out] dq      Pointer to the deque structure.
 * @param[out]    msg_ret Pointer to store the retrieved message.
 * @return ec_gt          GMP_EC_OK on success, or GMP_EC_DEQUE_EMPTY.
 */
ec_gt gmp_can_deque_pop_front(gmp_can_deque_t* dq, gmp_can_msg_t* msg_ret);


/**
 * @brief CAN Node Configuration and State Object.
 */
typedef struct
{
    can_halt bus; /**< Pointer to the physical CAN bus (CSP layer). */

    /* --- Software Queues --- */
    gmp_can_deque_t tx_deque;      /**< Transmit deque (Supports push_front for high priority). */
    gmp_can_deque_t rx_slow_queue; /**< Receive queue for non-real-time messages (SDO, etc.). */

    /* --- Routing Rules --- */
    uint32_t fast_rx_mask; /**< Mask to identify high-priority IDs. */
    uint32_t fast_rx_id;   /**< Target ID pattern for fast callback. */

    /* --- User Callbacks --- */
    void (*fast_rx_callback)(const gmp_can_msg_t* msg); /**< ISR-context fast callback. */
} gmp_can_node_t;

/* ========================================================================= */
/* ==================== CORE NODE APIs ===================================== */
/* ========================================================================= */

/**
 * @brief Initializes a CAN node with provided bus and buffers.
 */
void gmp_can_node_init(gmp_can_node_t* node, can_halt bus, gmp_can_msg_t* tx_buf, uint16_t tx_cap,
                       gmp_can_msg_t* rx_buf, uint16_t rx_cap);

/**
 * @brief Sets the routing rule for high-priority reception.
 */
void gmp_can_node_set_fast_path(gmp_can_node_t* node, uint32_t id, uint32_t mask, void (*cb)(const gmp_can_msg_t*));

/* ========================================================================= */
/* ==================== ISR ENGINE (PUMPS) ================================= */
/* ========================================================================= */

/**
 * @brief The Transmit Pump: To be called in the CAN Transmit Empty Interrupt.
 */
void gmp_can_node_tx_isr_pump(gmp_can_node_t* node);

/**
 * @brief The Receive Router: To be called in the CAN Receive Interrupt.
 */
void gmp_can_node_rx_isr_router(gmp_can_node_t* node, const gmp_can_msg_t* rx_msg);

/**
 * @brief CAN message transmission priority.
 */
typedef enum
{
    GMP_CAN_PRIORITY_NORMAL = 0, /**< Normal priority: Appended to the end of the Tx deque. */
    GMP_CAN_PRIORITY_HIGH = 1    /**< High priority: Inserted at the front of the Tx deque (pre-empts others). */
} gmp_can_priority_et;

ec_gt gmp_hal_can_bus_write(can_halt hcan, const gmp_can_msg_t* msg);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PERIPHERAL_PORT_H_
