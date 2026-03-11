/**
 * @file    dac8563.c
 * @brief   Hardware-agnostic driver implementation for DAC8563.
 */

#include <gmp_core.h>

#include <core/dev/dac/dac8563.h>

/* ========================================================================= */
/* ==================== PRIVATE HAL WRAPPERS =============================== */
/* ========================================================================= */

/**
 * @brief Construct and transmit the 24-bit SPI frame.
 * @note  The frame format: [X X C2 C1 C0] [A2 A1 A0 D15 D14 ... D8] [D7 D6 ... D0]
 * We assume gmp_hal_spi_write_array exists in your HAL to send continuous bytes
 * while holding the CS (SYNC) pin low.
 */
static ec_gt dac8563_write_24b_frame(dac8563_dev_t* dev, dac8563_cmd_et cmd, dac8563_addr_et addr, uint16_t data)
{
    /* Assemble the 24-bit payload */
    uint32_t payload = (((uint32_t)cmd & 0x07) << 19) | (((uint32_t)addr & 0x07) << 16) | ((uint32_t)data & 0xFFFF);

    /* Split into 3 bytes (MSB first) */
    uint8_t tx_buf[3];
    tx_buf[0] = (uint8_t)((payload >> 16) & 0xFF);
    tx_buf[1] = (uint8_t)((payload >> 8) & 0xFF);
    tx_buf[2] = (uint8_t)(payload & 0xFF);

    /* Transmit the 3 bytes. 
     * IMPORTANT: The SPI HAL must hold the SYNC pin LOW for exactly 24 clocks.
     */
    return gmp_hal_spi_write_array(dev->bus, tx_buf, 3);
}

/* ========================================================================= */
/* ==================== PUBLIC API IMPLEMENTATION ========================== */
/* ========================================================================= */

ec_gt dac8563_init(dac8563_dev_t* dev, spi_halt bus, const dac8563_init_t* init_cfg)
{
    ec_gt ret;

    if ((dev == NULL) || (init_cfg == NULL))
        return GMP_EC_GENERAL_ERROR;

    dev->bus = bus;

    /* Calculate the LSB voltage step: Vout = Vref * Gain * (Code / 65536) */
    dev->max_voltage_V = init_cfg->vref_V * init_cfg->gain;

    /* Cache the LSB multiplier to replace floating point division with multiplication later */
    if (dev->max_voltage_V > 0.0f)
    {
        dev->lsb_voltage_V = dev->max_voltage_V / 65536.0f;
    }
    else
    {
        dev->lsb_voltage_V = 0.0f;
        return GMP_EC_GENERAL_ERROR;
    }

    /* 1. Software Reset (Recommended on power-up to clear unknown states) */
    ret = dac8563_software_reset(dev);
    if (ret != GMP_EC_OK)
        return ret;

    /* 2. Setup Internal Reference */
    /* Command 111 (0x07), Address 000 (0x00), Data = 0x0001 (Enable) or 0x0000 (Disable) */
    ret =
        dac8563_write_24b_frame(dev, DAC8563_CMD_INTERNAL_REF_SETUP, DAC8563_ADDR_DAC_A, (uint16_t)init_cfg->ref_mode);
    if (ret != GMP_EC_OK)
        return ret;

    /* 3. Ensure DACs are powered up normally (Not in power down mode) */
    /* Command 100 (0x04), Data = 0x0000 (Normal operation) */
    return dac8563_write_24b_frame(dev, DAC8563_CMD_POWER_DOWN, DAC8563_ADDR_DAC_ALL, 0x0000);
}

ec_gt dac8563_software_reset(dac8563_dev_t* dev)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Data = 0x0001 triggers reset for DAC8563 */
    return dac8563_write_24b_frame(dev, DAC8563_CMD_SOFTWARE_RESET, DAC8563_ADDR_DAC_A, 0x0001);
}

ec_gt dac8563_write_code(dac8563_dev_t* dev, dac8563_addr_et channel, uint16_t code)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Most common operation: Write to Input Register and Update Respective DAC */
    return dac8563_write_24b_frame(dev, DAC8563_CMD_WRITE_INPUT_UPDATE_DAC, channel, code);
}

ec_gt dac8563_set_voltage(dac8563_dev_t* dev, dac8563_addr_et channel, float voltage_V)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Safety clamp to prevent overflow */
    if (voltage_V < 0.0f)
        voltage_V = 0.0f;
    if (voltage_V > dev->max_voltage_V)
        voltage_V = dev->max_voltage_V;

    /* * FPU Acceleration: Code = Voltage / LSB_Voltage 
     * Using multiplication (Voltage * (1/LSB_Voltage)) is faster on some FPUs, 
     * but dividing by pre-calculated LSB is mathematically identical and very fast.
     */
    uint16_t code = (uint16_t)(voltage_V / dev->lsb_voltage_V);

    return dac8563_write_code(dev, channel, code);
}
