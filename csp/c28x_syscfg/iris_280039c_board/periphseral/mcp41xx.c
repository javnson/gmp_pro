/**
 * @file    mcp41xx.c
 * @brief   Implementation of the MCP41XXX/42XXX Digital Potentiometers driver.
 */


#include <gmp_core.h>

#include <core/dev/potentiometer/mcp41xx.h>


/* ========================================================================= */
/* ==================== PRIVATE HELPER FUNCTIONS =========================== */
/* ========================================================================= */

/**
 * @brief Internal function to format and send the 16-bit SPI command.
 */
static ec_gt mcp41xx_send_command(mcp41xx_dev_t* dev, mcp41xx_cmd_et cmd, mcp41xx_pot_et pot, uint8_t data)
{
    /* The 16-bit frame is [Command Byte] [Data Byte] */
    uint8_t cmd_byte = (uint8_t)cmd | (uint8_t)pot;

    /* Combine into a 16-bit word */
    uint16_t spi_frame = ((uint16_t)cmd_byte << 8) | (uint16_t)data;

    /* Layer 2 API auto-manages CS assertion and MSB-first endianness serialization */
    return gmp_hal_spi_dev_write_16b(dev->spi_node, spi_frame, MCP41XX_CFG_TIMEOUT);
}

/* ========================================================================= */
/* ==================== PUBLIC API IMPLEMENTATION ========================== */
/* ========================================================================= */

ec_gt mcp41xx_init(mcp41xx_dev_t* dev, spi_device_halt spi_node, uint16_t resistance_kohm)
{
    if (dev == NULL || spi_node == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    dev->spi_node = spi_node;
    dev->resistance_kohm = resistance_kohm;

    /* Power-On Reset default wiper position is mid-scale (0x80) */
    dev->cached_wiper_0 = 0x80;
    dev->cached_wiper_1 = 0x80;

    return GMP_EC_OK;
}

ec_gt mcp41xx_set_wiper(mcp41xx_dev_t* dev, mcp41xx_pot_et target, uint8_t value)
{
    if (dev == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    ec_gt ret = mcp41xx_send_command(dev, MCP41XX_CMD_WRITE, target, value);

    /* Update software shadow registers upon success */
    if (ret == GMP_EC_OK)
    {
        if (target == MCP41XX_POT_0 || target == MCP41XX_POT_BOTH)
        {
            dev->cached_wiper_0 = value;
        }
        if (target == MCP41XX_POT_1 || target == MCP41XX_POT_BOTH)
        {
            dev->cached_wiper_1 = value;
        }
    }

    return ret;
}

ec_gt mcp41xx_shutdown(mcp41xx_dev_t* dev, mcp41xx_pot_et target)
{
    if (dev == NULL)
    {
        return GMP_EC_GENERAL_ERROR;
    }

    /* Shutdown command ignores the data byte, sending 0x00 as dummy */
    return mcp41xx_send_command(dev, MCP41XX_CMD_SHUTDOWN, target, 0x00);
}
