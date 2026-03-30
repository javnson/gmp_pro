
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// GPIO

ec_gt gmp_hal_gpio_set_dir(gpio_halt hgpio, gpio_dir_et dir)
{
    /* Zynq 7000 系列最大支持 118 个 GPIO (MIO + EMIO) */
    if (hgpio > 118)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    if (dir == GMP_HAL_GPIO_DIR_OUT)
    {
        /* 设置方向为输出 (1) */
        XGpioPs_SetDirectionPin(&Gpio, hgpio, 1);
        /* 必须开启输出使能，否则引脚不会有电平输出 */
        XGpioPs_SetOutputEnablePin(&Gpio, hgpio, 1);
    }
    else
    {
        /* 设置方向为输入 (0) */
        XGpioPs_SetDirectionPin(&Gpio, hgpio, 0);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_gpio_write(gpio_halt hgpio, fast_gt level)
{
    if (hgpio > 118)
        return GMP_EC_GENERAL_ERROR;

    /* XGpioPs_WritePin 的第三个参数：0 为低，1 为高 */
    XGpioPs_WritePin(&Gpio, hgpio, (level == GMP_HAL_GPIO_HIGH) ? 1 : 0);

    return GMP_EC_OK;
}

fast_gt gmp_hal_gpio_read(gpio_halt hgpio)
{
    if (hgpio > 118)
        return 0;

    /* 返回值为 u32，0 或 1 */
    u32 data = XGpioPs_ReadPin(&Gpio, hgpio);

    return (data == 1) ? 1 : 0;
}


// 映射 Zynq 状态到 GMP 错误码
static ec_gt zynq_uart_status_to_ec(int status)
{
    if (status == XST_SUCCESS)
        return GMP_EC_OK;
    if (status == XST_UART_FIFO_ERROR)
        return GMP_EC_GENERAL_ERROR;
    return GMP_EC_GENERAL_ERROR;
}

//////////////////////////////////////////////////////////////////////////
// UART

// 映射 Zynq 状态到 GMP 错误码
static ec_gt zynq_uart_status_to_ec(int status)
{
    if (status == XST_SUCCESS)
        return GMP_EC_OK;
    if (status == XST_UART_FIFO_ERROR)
        return GMP_EC_GENERAL_ERROR;
    return GMP_EC_GENERAL_ERROR;
}

fast_gt gmp_hal_uart_is_tx_busy(uart_halt uart)
{
    if (uart == NULL)
        return 0;

    /* 检查 TACTIVE 位：1 表示发送器正在活动（移位寄存器或 FIFO 非空） */
    u32 status = XUartPs_ReadReg(uart->Config.BaseAddress, XUARTPS_SR_OFFSET);
    return (status & XUARTPS_SR_TACTIVE) ? 1 : 0;
}

size_gt gmp_hal_uart_get_rx_available(uart_halt uart)
{
    if (uart == NULL)
        return 0;

    /* 检查 RX 空标志：如果 RXEMPTY 为 0，表示至少有 1 字节 */
    u32 status = XUartPs_ReadReg(uart->Config.BaseAddress, XUARTPS_SR_OFFSET);
    if (!(status & XUARTPS_SR_RXEMPTY))
    {
        return 1;
    }
    return 0;
}

ec_gt gmp_hal_uart_write(uart_halt uart, const data_gt* data, size_gt length, uint32_t timeout_ms)
{
    if (uart == NULL || data == NULL || length == 0)
        return GMP_EC_GENERAL_ERROR;

    XTime t_start, t_now;
    XTime_GetTime(&t_start);
    // 转换毫秒超时到系统计数器周期 (Zynq Global Timer 通常是 CPU 频率的一半)
    XTime t_timeout = (XTime)timeout_ms * (COUNTS_PER_SECOND / 1000);

    for (size_gt i = 0; i < length; i++)
    {
        /* 等待发送 FIFO 有空间 */
        while (XUartPs_IsTransmitFull(uart))
        {
            XTime_GetTime(&t_now);
            if ((t_now - t_start) > t_timeout)
                return GMP_EC_TIMEOUT;
        }
        XUartPs_WriteReg(uart->Config.BaseAddress, XUARTPS_FIFO_OFFSET, data[i]);
    }

    return GMP_EC_OK;
}

ec_gt gmp_hal_uart_read(uart_halt uart, data_gt* data, size_gt length, uint32_t timeout_ms, size_gt* bytes_read)
{
    if (uart == NULL || data == NULL || length == 0)
        return GMP_EC_GENERAL_ERROR;

    XTime t_start, t_now;
    XTime_GetTime(&t_start);
    XTime t_timeout = (XTime)timeout_ms * (COUNTS_PER_SECOND / 1000);
    size_gt count = 0;

    for (count = 0; count < length; count++)
    {
        /* 等待接收 FIFO 有数据 */
        while (!XUartPs_IsReceiveData(uart))
        {
            XTime_GetTime(&t_now);
            if ((t_now - t_start) > t_timeout)
            {
                if (bytes_read != NULL)
                    *bytes_read = count;
                return GMP_EC_TIMEOUT;
            }
        }
        data[count] = (u8)XUartPs_ReadReg(uart->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
    }

    if (bytes_read != NULL)
        *bytes_read = count;
    return GMP_EC_OK;
}

//////////////////////////////////////////////////////////////////////////
// IIC

// 将 Zynq I2C 状态映射为 GMP 错误码
static ec_gt zynq_i2c_status_to_ec(int status)
{
    if (status == XST_SUCCESS)
        return GMP_EC_OK;
    if (status == XST_IIC_ARB_LOST)
        return GMP_EC_GENERAL_ERROR;
    if (status == XST_IIC_NO_ACK)
        return GMP_EC_NACK;
    return GMP_EC_GENERAL_ERROR;
}

ec_gt gmp_hal_iic_write_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t reg_data,
                            size_gt reg_len, time_gt timeout)
{
    if (h == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint8_t buf[8];
    uint32_t idx = 0;

    // 1. 序列化寄存器地址 (MSB First)
    for (uint32_t i = 0; i < addr_len; i++)
    {
        buf[idx++] = (uint8_t)((reg_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }

    // 2. 序列化数据 (MSB First)
    for (uint32_t i = 0; i < reg_len; i++)
    {
        buf[idx++] = (uint8_t)((reg_data >> ((reg_len - 1 - i) * 8)) & 0xFF);
    }

    // Zynq SDK 的 Polled 接口不带直接超时参数，内部通过循环计数实现
    // 这里的地址不需要左移
    int status = XIicPs_MasterSendPolled(h, buf, idx, dev_addr);

    return zynq_i2c_status_to_ec(status);
}

ec_gt gmp_hal_iic_read_reg(iic_halt h, addr16_gt dev_addr, addr32_gt reg_addr, size_gt addr_len, uint32_t* reg_data_ret,
                           size_gt reg_len, time_gt timeout)
{
    if (h == NULL || reg_data_ret == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint8_t addr_buf[4];
    uint8_t data_buf[4] = {0};
    int status;

    // 1. 准备寄存器地址
    for (uint32_t i = 0; i < addr_len; i++)
    {
        addr_buf[i] = (uint8_t)((reg_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
    }

    if (addr_len > 0)
    {
        // 发送地址并保持总线 (Set Options 以开启 Repeated Start)
        XIicPs_SetOptions(h, XIICPS_REP_START_OPTION);
        status = XIicPs_MasterSendPolled(h, addr_buf, addr_len, dev_addr);
        if (status != XST_SUCCESS)
            return zynq_i2c_status_to_ec(status);
    }

    // 2. 读取数据 (清除 Options 以便在结束后发送 STOP)
    XIicPs_ClearOptions(h, XIICPS_REP_START_OPTION);
    status = XIicPs_MasterRecvPolled(h, data_buf, reg_len, dev_addr);

    if (status == XST_SUCCESS)
    {
        uint32_t val = 0;
        for (uint32_t i = 0; i < reg_len; i++)
        {
            val = (val << 8) | data_buf[i];
        }
        *reg_data_ret = val;
    }

    return zynq_i2c_status_to_ec(status);
}

ec_gt gmp_hal_iic_write_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, const data_gt* mem,
                            size_gt mem_len, time_gt timeout)
{
    if (h == NULL || (mem_len > 0 && mem == NULL))
        return GMP_EC_GENERAL_ERROR;

    int status;
    uint8_t addr_buf[4];

    // 1. 如果有地址段，先发送地址并保持总线 (Hold)
    if (addr_len > 0)
    {
        for (uint32_t i = 0; i < addr_len; i++)
        {
            addr_buf[i] = (uint8_t)((mem_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
        }

        // 设置 HOLD 位，防止发送完地址后产生 STOP
        XIicPs_SetOptions(h, XIICPS_REP_START_OPTION);

        status = XIicPs_MasterSendPolled(h, addr_buf, addr_len, dev_addr);
        if (status != XST_SUCCESS)
        {
            XIicPs_ClearOptions(h, XIICPS_REP_START_OPTION); // 发生错误需清除并释放总线
            return zynq_i2c_status_to_ec(status);
        }
    }

    // 2. 发送数据段，并在结束时释放总线 (Clear HOLD -> 发送 STOP)
    XIicPs_ClearOptions(h, XIICPS_REP_START_OPTION);
    status = XIicPs_MasterSendPolled(h, (uint8_t*)mem, mem_len, dev_addr);

    return zynq_i2c_status_to_ec(status);
}

ec_gt gmp_hal_iic_read_mem(iic_halt h, addr16_gt dev_addr, addr32_gt mem_addr, size_gt addr_len, data_gt* mem,
                           size_gt mem_len, time_gt timeout)
{
    if (h == NULL || mem == NULL)
        return GMP_EC_GENERAL_ERROR;

    int status;
    uint8_t addr_buf[4];

    // 1. 发送内存地址段
    if (addr_len > 0)
    {
        for (uint32_t i = 0; i < addr_len; i++)
        {
            addr_buf[i] = (uint8_t)((mem_addr >> ((addr_len - 1 - i) * 8)) & 0xFF);
        }

        // 核心：设置寄存器，确保发完地址后不产生 STOP，而是等待下一个操作产生 RESTART
        XIicPs_SetOptions(h, XIICPS_REP_START_OPTION);

        status = XIicPs_MasterSendPolled(h, addr_buf, addr_len, dev_addr);
        if (status != XST_SUCCESS)
        {
            XIicPs_ClearOptions(h, XIICPS_REP_START_OPTION);
            return zynq_i2c_status_to_ec(status);
        }
    }

    // 2. 读取数据段
    // 核心：在最后一次接收操作前清除 HOLD 位，这样硬件会在接收完最后一个字节后自动发送 STOP
    XIicPs_ClearOptions(h, XIICPS_REP_START_OPTION);
    status = XIicPs_MasterRecvPolled(h, (uint8_t*)mem, mem_len, dev_addr);

    return zynq_i2c_status_to_ec(status);
}

//////////////////////////////////////////////////////////////////////////
// SPI

static ec_gt zynq_spi_status_to_ec(int status)
{
    if (status == XST_SUCCESS)
        return GMP_EC_OK;
    return GMP_EC_GENERAL_ERROR;
}

ec_gt gmp_hal_spi_bus_write(spi_halt hspi, const data_gt* tx_buf, size_gt len, time_gt timeout)
{
    if (hspi == NULL || tx_buf == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* * XSpiPs_PolledTransfer 是阻塞式的。
     * 它会自动处理 FIFO 的填充和清空。
     * 第三个参数为 NULL 表示丢弃接收到的数据。
     */
    int status = XSpiPs_PolledTransfer(hspi, (u8*)tx_buf, NULL, len);

    return zynq_spi_status_to_ec(status);
}

ec_gt gmp_hal_spi_bus_read(spi_halt hspi, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if (hspi == NULL || rx_buf == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* * 第一个缓冲区传 NULL，驱动会自动发送 0x00 或上一次发送的数据（取决于硬件配置）
     * 以便从从机读取 len 长度的数据到 rx_buf。
     */
    int status = XSpiPs_PolledTransfer(hspi, NULL, (u8*)rx_buf, len);

    return zynq_spi_status_to_ec(status);
}

ec_gt gmp_hal_spi_bus_transfer(spi_halt hspi, const data_gt* tx_buf, data_gt* rx_buf, size_gt len, time_gt timeout)
{
    if (hspi == NULL || tx_buf == NULL || rx_buf == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* 同时发送 tx_buf 并接收数据到 rx_buf */
    int status = XSpiPs_PolledTransfer(hspi, (u8*)tx_buf, (u8*)rx_buf, len);

    return zynq_spi_status_to_ec(status);
}
