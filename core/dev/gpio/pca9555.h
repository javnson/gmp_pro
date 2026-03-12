/**
 * @file    pca9555.h
 * @brief   Hardware-agnostic driver for PCA9555 16-bit I2C GPIO expander.
 * @note    This driver caches configuration and output states to avoid 
 * expensive I2C read-modify-write operations on the bus.
 */

#ifndef PCA9555_H
#define PCA9555_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

/** * @brief Default I2C timeout in milliseconds for PCA9555 operations.
 */
#ifndef PCA9555_CFG_TIMEOUT
#define PCA9555_CFG_TIMEOUT (5U)
#endif

/* ========================================================================= */
/* ==================== REGISTERS & COMMANDS =============================== */
/* ========================================================================= */

#define PCA9555_REG_IN_PORT0  0x00
#define PCA9555_REG_IN_PORT1  0x01
#define PCA9555_REG_OUT_PORT0 0x02
#define PCA9555_REG_OUT_PORT1 0x03
#define PCA9555_REG_POL_PORT0 0x04
#define PCA9555_REG_POL_PORT1 0x05
#define PCA9555_REG_CFG_PORT0 0x06
#define PCA9555_REG_CFG_PORT1 0x07

/** * @brief Helper macro to calculate the 7-bit I2C address based on hardware pins A2, A1, A0.
 * @param a2 State of pin A2 (0 or 1)
 * @param a1 State of pin A1 (0 or 1)
 * @param a0 State of pin A0 (0 or 1)
 * @return 7-bit device address (e.g., 0x20 when all are 0)
 */
#define PCA9555_CALC_ADDR(a2, a1, a0) (0x20 | (((a2) & 0x01) << 2) | (((a1) & 0x01) << 1) | ((a0) & 0x01))

/* ========================================================================= */
/* ==================== DATA STRUCTURES ==================================== */
/* ========================================================================= */

/**
 * @brief Port identification enum.
 */
typedef enum
{
    PCA9555_PORT_0 = 0,
    PCA9555_PORT_1 = 1
} pca9555_port_et;

/**
 * @brief Pin direction configuration.
 */
typedef enum
{
    PCA9555_DIR_OUTPUT = 0, /* PCA9555 defines 0 as Output */
    PCA9555_DIR_INPUT = 1   /* PCA9555 defines 1 as Input */
} pca9555_dir_et;

/**
 * @brief Pin polarity inversion configuration.
 */
typedef enum
{
    PCA9555_POL_NORMAL = 0,
    PCA9555_POL_INVERTED = 1
} pca9555_pol_et;

/**
 * @brief Initialization parameters for PCA9555.
 */
typedef struct
{
    uint8_t cfg_port0; /**< Direction config for Port 0 (1=Input, 0=Output) */
    uint8_t cfg_port1; /**< Direction config for Port 1 */
    uint8_t out_port0; /**< Initial output state for Port 0 */
    uint8_t out_port1; /**< Initial output state for Port 1 */
    uint8_t pol_port0; /**< Polarity inversion for Port 0 (1=Inverted) */
    uint8_t pol_port1; /**< Polarity inversion for Port 1 */
} pca9555_init_t;

/**
 * @brief Runtime Device Object for PCA9555.
 * Contains local shadow registers to prevent I2C read-modify-write delays.
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    data_gt shadow_cfg[2]; /**< Cache for Direction registers */
    data_gt shadow_out[2]; /**< Cache for Output registers */
    data_gt shadow_pol[2]; /**< Cache for Polarity registers */
} pca9555_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt pca9555_init(pca9555_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const pca9555_init_t* init_cfg);

ec_gt pca9555_set_pin_direction(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, pca9555_dir_et dir);
ec_gt pca9555_set_pin_polarity(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, pca9555_pol_et pol);

ec_gt pca9555_set_pin_output(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, fast_gt state);
fast_gt pca9555_get_pin_input(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num, ec_gt* err_code_ret);

ec_gt pca9555_get_port0_input(pca9555_dev_t* dev, uint8_t* port_data_ret);
ec_gt pca9555_get_port1_output(pca9555_dev_t* dev, uint8_t* port_data_ret);

ec_gt pca9555_get_port_config(pca9555_dev_t* dev, pca9555_port_et port, uint8_t* cfg_ret, uint8_t* pol_ret);

/* ========================================================================= */
/* ==================== CACHE SYNCHRONIZATION API ========================== */
/* ========================================================================= */

/**
 * @brief   Flush the local shadow output registers to the PCA9555 hardware.
 * @note    This function utilizes the auto-increment feature of PCA9555 to write
 * both Port 0 and Port 1 outputs in a single I2C transaction.
 * * @param[in,out] dev       Pointer to the device object.
 * * @return  ec_gt           GMP_EC_OK on success.
 */
ec_gt pca9555_flush_outputs(pca9555_dev_t* dev);

/**
 * @brief   Flush all local shadow registers (Config, Polarity, Output) to hardware.
 * @note    Useful for recovering device state after an unexpected reset.
 * * @param[in,out] dev       Pointer to the device object.
 * * @return  ec_gt           GMP_EC_OK on success.
 */
ec_gt pca9555_flush_all_shadows(pca9555_dev_t* dev);

/**
 * @brief   Read actual hardware registers and refresh the local shadow cache.
 * @note    Useful if another I2C master might have changed the PCA9555 state, 
 * or during system diagnosis.
 * * @param[in,out] dev       Pointer to the device object.
 * * @return  ec_gt           GMP_EC_OK on success.
 */
ec_gt pca9555_refresh_shadows(pca9555_dev_t* dev);

/**
 * @brief   Set pin output state locally WITHOUT triggering an I2C transaction.
 * @note    User MUST call pca9555_flush_outputs() later to apply changes.
 * This is an extremely fast inline function for batch processing.
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     port      Port to modify.
 * @param[in]     pin_num   Pin number (0-7).
 * @param[in]     state     Desired output state (non-zero for HIGH, 0 for LOW).
 */
GMP_STATIC_INLINE void pca9555_set_pin_output_cached(pca9555_dev_t* dev, pca9555_port_et port, uint8_t pin_num,
                                                     fast_gt state)
{
    if (dev != NULL && pin_num <= 7)
    {
        if (state)
        {
            dev->shadow_out[port] |= (1 << pin_num);
        }
        else
        {
            dev->shadow_out[port] &= ~(1 << pin_num);
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif /* PCA9555_H */
