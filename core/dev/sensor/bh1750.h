/**
 * @file    bh1750.h
 * @brief   Hardware-agnostic driver for ROHM BH1750FVI Ambient Light Sensor.
 * @note    This driver utilizes the GMP HAL I2C interface.
 */

#ifndef BH1750_H
#define BH1750_H

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

/** * @brief Default I2C timeout in milliseconds.
 * Note: Measurement takes max 180ms in High-Res mode. 
 * This timeout is for I2C bus transactions, not for measurement delay.
 */
#ifndef BH1750_CFG_TIMEOUT
#define BH1750_CFG_TIMEOUT (10U)
#endif

/** * @brief Helper macro to calculate the 7-bit I2C address based on the ADDR pin.
 * @param addr_pin State of the ADDR pin (0 for Low, 1 for High)
 * @return 7-bit device address (0x23 if Low, 0x5C if High)
 */
#define BH1750_CALC_ADDR(addr_pin) ((addr_pin) ? 0x5C : 0x23)

/* ========================================================================= */
/* ==================== COMMANDS =========================================== */
/* ========================================================================= */

#define BH1750_CMD_POWER_DOWN 0x00
#define BH1750_CMD_POWER_ON   0x01
#define BH1750_CMD_RESET      0x07

/* Measurement Time (MT) commands */
#define BH1750_CMD_MT_HIGH_BIT_MASK 0x40
#define BH1750_CMD_MT_LOW_BIT_MASK  0x60
#define BH1750_DEFAULT_MT           69 /**< Default MT value */

/* ========================================================================= */
/* ==================== ENUMS & DATA STRUCTURES ============================ */
/* ========================================================================= */

/** * @brief Measurement mode and resolution.
 * Combination of Continuous/One-Time and H/H2/L Resolution.
 */
typedef enum
{
    /* Continuous Modes */
    BH1750_MODE_CONT_H_RES = 0x10,  /**< Continuously H-Resolution (1 lx, ~120ms) */
    BH1750_MODE_CONT_H_RES2 = 0x11, /**< Continuously H-Resolution Mode2 (0.5 lx, ~120ms) */
    BH1750_MODE_CONT_L_RES = 0x13,  /**< Continuously L-Resolution (4 lx, ~16ms) */

    /* One-Time Modes (Device powers down automatically after measurement) */
    BH1750_MODE_ONCE_H_RES = 0x20,  /**< One-Time H-Resolution */
    BH1750_MODE_ONCE_H_RES2 = 0x21, /**< One-Time H-Resolution Mode2 */
    BH1750_MODE_ONCE_L_RES = 0x23   /**< One-Time L-Resolution */
} bh1750_mode_et;

/**
 * @brief Initialization parameters for BH1750.
 */
typedef struct
{
    bh1750_mode_et mode; /**< Measurement mode */
    uint8_t mt_reg;      /**< Measurement Time register (31 to 254, Default is 69) */
} bh1750_init_t;

/**
 * @brief Runtime Device Object for BH1750.
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    bh1750_mode_et mode; /**< Cached mode to calculate exact lux resolution */
    uint8_t mt_reg;      /**< Cached MT to calculate exact lux value */
} bh1750_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt bh1750_init(bh1750_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const bh1750_init_t* init_cfg);

ec_gt bh1750_set_mode(bh1750_dev_t* dev, bh1750_mode_et mode);

ec_gt bh1750_set_measurement_time(bh1750_dev_t* dev, uint8_t mt_val);

ec_gt bh1750_power_down(bh1750_dev_t* dev);

ec_gt bh1750_read_lux(bh1750_dev_t* dev, float* lux_ret);

#ifdef __cplusplus
}
#endif

#endif /* BH1750_H */
