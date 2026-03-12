/**
 * @file    mcp41xx.h
 * @brief   Hardware-Agnostic driver for Microchip MCP41XXX/42XXX Digital Potentiometers.
 */

#ifndef MCP41XX_H
#define MCP41XX_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef MCP41XX_CFG_TIMEOUT
#define MCP41XX_CFG_TIMEOUT (10U) /**< Default SPI timeout in ms */
#endif

/* ========================================================================= */
/* ==================== ENUMS & DATA STRUCTURES ============================ */
/* ========================================================================= */

/** @brief Potentiometer Selection (P1 P0 bits) */
typedef enum
{
    MCP41XX_POT_0 = 0x01,   /**< Select Potentiometer 0 */
    MCP41XX_POT_1 = 0x02,   /**< Select Potentiometer 1 (MCP42XXX only) */
    MCP41XX_POT_BOTH = 0x03 /**< Select both Potentiometers */
} mcp41xx_pot_et;

/** @brief Command Selection (C1 C0 bits shifted to bit 4,5) */
typedef enum
{
    MCP41XX_CMD_WRITE = (0x01 << 4),   /**< Write Data to Wiper */
    MCP41XX_CMD_SHUTDOWN = (0x02 << 4) /**< Put Potentiometer into Shutdown mode */
} mcp41xx_cmd_et;

/**
 * @brief Device Context Structure for MCP41XXX/42XXX
 */
typedef struct
{
    spi_device_halt spi_node; /**< Layer 2 Logical SPI Device Handle */
    uint16_t resistance_kohm; /**< Full scale resistance (e.g., 10, 50, 100) */
    uint8_t cached_wiper_0;   /**< Shadow register to track current position of Pot 0 */
    uint8_t cached_wiper_1;   /**< Shadow register to track current position of Pot 1 */
} mcp41xx_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief Initializes the MCP41XX/42XX driver context.
 * @param dev               Pointer to the device structure.
 * @param spi_node          Layer 2 SPI Logical Device handle.
 * @param resistance_kohm   Rated resistance of the specific chip variant.
 * @return ec_gt            Error code.
 */
ec_gt mcp41xx_init(mcp41xx_dev_t* dev, spi_device_halt spi_node, uint16_t resistance_kohm);

/**
 * @brief Sets the wiper position of the selected potentiometer.
 * @param dev     Pointer to the device structure.
 * @param target  Which pot to control (POT_0, POT_1, or POT_BOTH).
 * @param value   Wiper setting (0x00 to 0xFF).
 * @return ec_gt  Error code.
 */
ec_gt mcp41xx_set_wiper(mcp41xx_dev_t* dev, mcp41xx_pot_et target, uint8_t value);

/**
 * @brief Puts the selected potentiometer into hardware shutdown mode.
 * @param dev     Pointer to the device structure.
 * @param target  Which pot to shutdown.
 * @return ec_gt  Error code.
 */
ec_gt mcp41xx_shutdown(mcp41xx_dev_t* dev, mcp41xx_pot_et target);

#ifdef __cplusplus
}
#endif

#endif /* MCP41XX_H */
