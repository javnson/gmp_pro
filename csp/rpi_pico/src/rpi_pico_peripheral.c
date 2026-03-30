
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// RP PICO library

/**
 * @brief 设定 GPIO 引脚方向
 * @note  Pico SDK 中 gpio_set_dir 会自动处理方向切换
 */
ec_gt gmp_hal_gpio_set_dir(gpio_halt hgpio, gpio_dir_et dir)
{
    /* 对于 Pico，0-29 是合法范围，简单的安全检查 */
    if (hgpio > 29)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    /* 初始化引脚（确保该引脚已受 GPIO 模块控制，而非 SIO 或其他外设） */
    gpio_init(hgpio);

    if (dir == GMP_HAL_GPIO_DIR_OUT)
    {
        /* 设置为输出 */
        gpio_set_dir(hgpio, GPIO_OUT);
        /* 默认为推挽模式，Pico 默认不开启内建上/下拉 */
        //gpio_disable_pulls(hgpio);
    }
    else
    {
        /* 设置为输入 */
        gpio_set_dir(hgpio, GPIO_IN);
        /* 对应 STM32 的 Floating 模式 */
        //gpio_disable_pulls(hgpio);
    }

    return GMP_EC_OK;
}

/**
 * @brief 写入 GPIO 电平
 */
ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level)
{
    if (hgpio > 29)
        return GMP_EC_GENERAL_ERROR;

    /* 直接调用 SDK 函数，level 非 0 即为高 */
    gpio_put(hgpio, (level == GMP_HAL_GPIO_HIGH) ? true : false);

    return GMP_EC_OK;
}

/**
 * @brief 读取 GPIO 电平
 */
fast_gt gmp_hal_gpio_read(gpio_halt hgpio)
{
    if (hgpio > 29)
        return 0;

    /* 读取当前引脚电平状态 */
    bool state = gpio_get(hgpio);

    return state ? 1 : 0;
}

//////////////////////////////////////////////////////////////////////////
// UART Peripheral

fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    if (uart == NULL)
        return 0;

    /* uart_is_writable 返回 true 表示 FIFO 未满，
     * 但 busy 状态通常指“正在移位输出”，对应 SDK 的 uart_tx_wait_blocking 内部逻辑 */
    // 检查硬件状态：如果 FR (Flag Register) 的 BUSY 位为 1，则返回忙
    return uart_get_hw(uart)->fr & UART_UARTFR_BUSY_BITS ? 1 : 0;
}

size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    if (uart == NULL)
        return 0;

    /* Pico SDK 判断 RX FIFO 是否有数据 */
    if (uart_is_readable(uart))
    {
        return 1; // 简单语义：至少有 1 字节可用
    }
    return 0;
}

ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt* data, size_gt length, uint32_t timeout)
{
    if (uart == NULL || data == NULL || length == 0)
        return GMP_EC_GENERAL_ERROR;

    // 计算截止时间
    absolute_time_t t_end = make_timeout_time_ms(timeout);

    for (size_gt i = 0; i < length; i++)
    {
        // 循环等待直到 FIFO 有空间或超时
        while (!uart_is_writable(uart))
        {
            if (time_reached(t_end))
                return GMP_EC_TIMEOUT;
            tight_loop_contents(); // 简单的 nop 占位，防止编译器过度优化
        }
        uart_putc_raw(uart, data[i]);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_uart_read(uart_halt uart, data_gt* data, size_gt length, uint32_t timeout, size_gt* bytes_read)
{
    if (uart == NULL || data == NULL || length == 0)
        return GMP_EC_GENERAL_ERROR;

    absolute_time_t t_end = make_timeout_time_ms(timeout);
    size_gt count = 0;

    for (count = 0; count < length; count++)
    {
        // 循环等待直到接收 FIFO 有数据或超时
        while (!uart_is_readable(uart))
        {
            if (time_reached(t_end))
            {
                if (bytes_read != NULL)
                    *bytes_read = count;
                return GMP_EC_TIMEOUT;
            }
            tight_loop_contents();
        }
        data[count] = uart_getc(uart);
    }

    if (bytes_read != NULL)
        *bytes_read = count;
    return GMP_EC_OK;
}

//////////////////////////////////////////////////////////////////////////
// IIC peripheral

// 映射 PICO 的返回值到 GMP 错误码
static ec_gt pico_i2c_status_to_ec(int pico_res)
{
    if (pico_res >= 0)
        return GMP_EC_OK;
    if (pico_res == PICO_ERROR_GENERIC)
        return GMP_EC_NACK; // 通常是无应答
    if (pico_res == PICO_ERROR_TIMEOUT)
        return GMP_EC_TIMEOUT;
    return GMP_EC_GENERAL_ERROR;
}

// 发送指令 (无数据段)
ec_gt gmp_hal_iic_write_cmd(iic_halt h, addr16_gt dev_addr, uint32_t cmd, size_gt cmd_len, time_gt timeout)
{
    i2c_inst_t* i2c = (i2c_inst_t*)h;
    uint8_t buf[4];
    for (uint32_t i = 0; i < cmd_len; i++)
    {
        buf[i] = (uint8_t)((cmd >> ((cmd_len - 1 - i) * 8)) & 0xFF);
    }

    // Pico SDK 地址不需要左移，且带超时机制
    int res = i2c_write_timeout_per_char_us(i2c, dev_addr, buf, cmd_len, false, timeout * 1000);
    return pico_i2c_status_to_ec(res);
}

// 写入单个寄存器数据
ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout)
{
    i2c_inst_t* i2c = (i2c_inst_t*)h;
    uint8_t buf[8];
    uint32_t idx = 0;

    for (uint32_t i = 0; i < addr_len; i++)
    {
        buf[idx++] = (uint8_t)((reg_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }
    for (uint32_t i = 0; i < reg_len; i++)
    {
        buf[idx++] = (uint8_t)((reg_data >> ((reg_len - 1 - i) * 8)) & 0xFF);
    }

    int res = i2c_write_timeout_per_char_us(i2c, dev_addr, buf, idx, false, timeout * 1000);
    return pico_i2c_status_to_ec(res);
}

// 写入连续内存块
ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout)
{
    i2c_inst_t* i2c = (i2c_inst_t*)h;
    if (addr_len == 0)
    {
        int res = i2c_write_timeout_per_char_us(i2c, dev_addr, mem, mem_len, false, timeout * 1000);
        return pico_i2c_status_to_ec(res);
    }

    // Pico 没有 Mem_Write，需要拼接地址和数据
    uint8_t addr_buf[4];
    for (uint32_t i = 0; i < addr_len; i++)
    {
        addr_buf[i] = (uint8_t)((mem_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }

    // 首先发送地址，使用 nostop=true 保持总线
    int res = i2c_write_timeout_per_char_us(i2c, dev_addr, addr_buf, addr_len, true, timeout * 1000);
    if (res < 0)
        return pico_i2c_status_to_ec(res);

    // 发送数据
    res = i2c_write_timeout_per_char_us(i2c, dev_addr, mem, mem_len, false, timeout * 1000);
    return pico_i2c_status_to_ec(res);
}

// 读取寄存器 (Start -> Write Addr -> Restart -> Read Data)
ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout)
{
    uint8_t buf[4] = {0};
    ec_gt status = gmp_hal_iic_read_mem(h, dev_addr, reg_addr, addr_len, buf, reg_len, timeout);

    if (status == GMP_EC_OK && reg_data_ret != NULL)
    {
        uint32_t val = 0;
        for (uint32_t i = 0; i < reg_len; i++)
        {
            val = (val << 8) | buf[i];
        }
        *reg_data_ret = val;
    }
    return status;
}

// 读取内存块
ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout)
{
    i2c_inst_t* i2c = (i2c_inst_t*)h;
    if (addr_len > 0)
    {
        uint8_t addr_buf[4];
        for (uint32_t i = 0; i < addr_len; i++)
        {
            addr_buf[i] = (uint8_t)((mem_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
        }
        // 发送地址，nostop=true 触发 Repeated Start
        int res = i2c_write_timeout_per_char_us(i2c, dev_addr, addr_buf, addr_len, true, timeout * 1000);
        if (res < 0)
            return pico_i2c_status_to_ec(res);
    }

    int res = i2c_read_timeout_per_char_us(i2c, dev_addr, mem, mem_len, false, timeout * 1000);
    return pico_i2c_status_to_ec(res);
}

//////////////////////////////////////////////////////////////////////////
// SPI peripheral

// 将 PICO SDK 的逻辑映射到 GMP 错误码
static ec_gt pico_spi_status_to_ec(int pico_res)
{
    if (pico_res >= 0)
        return GMP_EC_OK;
    // PICO SDK 的 SPI 阻塞函数通常返回写入/读取的字节数，
    // 硬件层面的错误（如超时）在基础 SDK 中较少直接返回，
    // 这里保留映射逻辑以对标 STM32。
    return GMP_EC_GENERAL_ERROR;
}

ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt* tx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    spi_inst_t* spi = (spi_inst_t*)hspi;

    /* Pico SDK 的 spi_write_blocking 会持续发送直到完成。
     * 由于 SDK 默认不带超时参数，这里直接调用同步接口。
     * 如果需要严格超时，通常需要配合 DMA 或手动轮询 FIFO 状态位。
     */
    int res = spi_write_blocking(spi, (const uint8_t*)tx_buf, (size_t)len);

    return (res == (int)len) ? GMP_EC_OK : GMP_EC_GENERAL_ERROR;
}

ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    spi_inst_t* spi = (spi_inst_t*)hspi;

    /* 自动发送重复的重复字节（通常为 0）以产生时钟并捕获 MISO 数据 */
    int res = spi_read_blocking(spi, 0, (uint8_t*)rx_buf, (size_t)len);

    return (res == (int)len) ? GMP_EC_OK : GMP_EC_GENERAL_ERROR;
}

ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if ((hspi == NULL) || (tx_buf == NULL) || (rx_buf == NULL))
    {
        return GMP_EC_GENERAL_ERROR;
    }

    spi_inst_t* spi = (spi_inst_t*)hspi;

    /* 同时进行发送和接收 */
    int res = spi_write_read_blocking(spi, (const uint8_t*)tx_buf, (uint8_t*)rx_buf, (size_t)len);

    return (res == (int)len) ? GMP_EC_OK : GMP_EC_GENERAL_ERROR;
}
