
#include <core/dev/driver/driver.h>

#include <core/dev/driver/encoder/as5048a.h>

/* ========================================================================= */
/* ==================== PRIVATE HELPER FUNCTIONS =========================== */
/* ========================================================================= */

/**
 * @brief Executes a single 16-bit secure register transfer.
 * @note  Handles parity calculation and frame assembly for initialization and diagnostics.
 * @param[in]  dev       Pointer to the AS5048A device structure.
 * @param[in]  tx_cmd    Command to transmit (without parity bit).
 * @param[out] rx_data   Pointer to store the received 16-bit data.
 * @return ec_gt         Error code.
 */
static ec_gt as5048a_transfer_16b(as5048a_dev_t* dev, uint16_t tx_cmd, uint16_t* rx_data)
{
    if (as5048a_calc_parity(tx_cmd))
    {
        tx_cmd |= 0x8000;
    }

    data_gt tx_buf[2], rx_buf[2];
    tx_buf[0] = (data_gt)(tx_cmd >> 8);
    tx_buf[1] = (data_gt)(tx_cmd & 0xFF);

    ec_gt ret = gmp_hal_spi_dev_transfer(dev->spi_node, tx_buf, rx_buf, 2, AS5048A_CFG_TIMEOUT);

    if (ret == GMP_EC_OK)
    {
        *rx_data = ((uint16_t)rx_buf[0] << 8) | ((uint16_t)rx_buf[1] & 0x00FFU);
    }
    return ret;
}

/**
 * @brief Synchronously reads a register from AS5048A, compensating for pipeline delay.
 * @note  AS5048A requires two frames to read a value. The first sends the address, 
 * the second (NOP) fetches the data.
 * @param[in]  dev       Pointer to the AS5048A device structure.
 * @param[in]  reg_addr  Internal register address to read.
 * @param[out] value     Pointer to store the actual register value (stripped of flags).
 * @return ec_gt         Error code.
 */
static ec_gt as5048a_read_reg_sync(as5048a_dev_t* dev, uint16_t reg_addr, uint16_t* value)
{
    uint16_t cmd = AS5048A_CMD_READ | (reg_addr & 0x3FFF);
    uint16_t dummy_rx;

    /* Frame 1: Send Target Address (Ignore received data from previous pipeline) */
    ec_gt ret = as5048a_transfer_16b(dev, cmd, &dummy_rx);
    if (ret != GMP_EC_OK)
        return ret;

    /* Frame 2: Send NOP (0x0000) to clock out the requested data */
    ret = as5048a_transfer_16b(dev, AS5048A_REG_NOP, value);
    if (ret != GMP_EC_OK)
        return ret;

    /* Validate Error Flag (Bit 14) */
    if (*value & 0x4000)
        return GMP_EC_GENERAL_ERROR;

    /* Strip parity and error bits, keep the 14-bit payload */
    *value &= 0x3FFF;
    return ret;
}

/* ========================================================================= */
/* ==================== PUBLIC API IMPLEMENTATION ========================== */
/* ========================================================================= */

ec_gt as5048a_init(as5048a_dev_t* dev, spi_device_halt spi_node)
{
    if ((dev == NULL) || (spi_node == NULL))
        return GMP_EC_GENERAL_ERROR;

    /* 1. Hardware Interface Binding */
    dev->spi_node = spi_node;

    /* 2. State Variables Reset */
    dev->raw = 0;
    dev->err_flag = 0;
    dev->diag_flags = 0;

    /* 3. Hardware Handshake: Clear any lingering error flags in the IC */
    ec_gt result = as5048a_clear_error_flag(dev);
    if (result != GMP_EC_OK)
        return result;

    /* 4. Pipeline Pre-fill
     * Must execute a dummy read to get the first actual mechanical position,
     * preventing an erroneous revolution jump on the very first step().
     */
    uint16_t initial_raw;
    result = as5048a_read_reg_sync(dev, AS5048A_REG_ANGLE, &initial_raw);
    if (result != GMP_EC_OK)
        return result;

    dev->raw = initial_raw;

    return GMP_EC_OK;
}

ec_gt as5048a_update_diagnostics(as5048a_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint16_t diag_val = 0;
    ec_gt ret = as5048a_read_reg_sync(dev, AS5048A_REG_DIAGNOSTICS, &diag_val);

    if (ret == GMP_EC_OK)
    {
        /* Cache diagnostics: OCF (Offset Compensation), COF (Cordic Overflow), COMP (AGC limits) */
        dev->diag_flags = diag_val;
    }
    return ret;
}

ec_gt as5048a_clear_error_flag(as5048a_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    uint16_t dummy;
    /* Datasheet: Reading the Clear Error Flag register clears all hardware error states */
    return as5048a_read_reg_sync(dev, AS5048A_REG_CLEAR_ERROR, &dummy);
}
