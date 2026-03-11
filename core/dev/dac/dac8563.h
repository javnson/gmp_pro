/**
 * @file    dac8563.h
 * @brief   Hardware-agnostic driver for TI DAC8563 Dual 16-Bit DAC.
 * @note    This driver utilizes a generic SPI HAL interface. The SPI HAL must 
 * support 24-bit data framing (or 3x 8-bit consecutive transfers while holding SYNC/CS low).
 */

#ifndef DAC8563_H
#define DAC8563_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== REGISTERS & COMMANDS =============================== */
/* ========================================================================= */

/** @brief DAC Commands (Bits 21-19) */
typedef enum
{
    DAC8563_CMD_WRITE_INPUT = 0x00,            /**< Write to Input Register */
    DAC8563_CMD_UPDATE_DAC = 0x01,             /**< Update DAC Register */
    DAC8563_CMD_WRITE_INPUT_UPDATE_ALL = 0x02, /**< Write to Input Reg and Update All DACs */
    DAC8563_CMD_WRITE_INPUT_UPDATE_DAC = 0x03, /**< Write to Input Reg and Update respective DAC */
    DAC8563_CMD_POWER_DOWN = 0x04,             /**< Power Down/Up DAC */
    DAC8563_CMD_SOFTWARE_RESET = 0x05,         /**< Software Reset */
    DAC8563_CMD_LDAC_SETUP = 0x06,             /**< Set LDAC Registers */
    DAC8563_CMD_INTERNAL_REF_SETUP = 0x07      /**< Enable/Disable Internal Reference */
} dac8563_cmd_et;

/** @brief DAC Addresses (Bits 18-16) */
typedef enum
{
    DAC8563_ADDR_DAC_A = 0x00,  /**< Target DAC A */
    DAC8563_ADDR_DAC_B = 0x01,  /**< Target DAC B */
    DAC8563_ADDR_DAC_ALL = 0x07 /**< Target Both DACs */
} dac8563_addr_et;

/* ========================================================================= */
/* ==================== ENUMS & DATA STRUCTURES ============================ */
/* ========================================================================= */

/** @brief Internal Reference Mode */
typedef enum
{
    DAC8563_INT_REF_DISABLE = 0x0000, /**< Disabled (Requires external VREF) */
    DAC8563_INT_REF_ENABLE = 0x0001   /**< Enabled (Internal 2.5V is active) */
} dac8563_ref_mode_et;

/**
 * @brief Initialization parameters for DAC8563.
 */
typedef struct
{
    dac8563_ref_mode_et ref_mode;
    float vref_V; /**< Reference Voltage in Volts (e.g., 2.5f) */
    float gain;   /**< Hardware Gain (Usually 2.0f for DAC8563, 1.0f for DAC8562) */
} dac8563_init_t;

/**
 * @brief Runtime Device Object for DAC8563.
 */
typedef struct
{
    spi_halt bus;
    float lsb_voltage_V; /**< Cached LSB resolution in Volts to accelerate FPU math */
    float max_voltage_V; /**< Maximum allowed output voltage */
} dac8563_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt dac8563_init(dac8563_dev_t* dev, spi_halt bus, const dac8563_init_t* init_cfg);

/**
 * @brief  Raw write function to send a 16-bit code directly to the DAC.
 */
ec_gt dac8563_write_code(dac8563_dev_t* dev, dac8563_addr_et channel, uint16_t code);

/**
 * @brief  Sets the output voltage for the specified DAC channel.
 * @note   Leverages FPU for extreme fast calculation.
 */
ec_gt dac8563_set_voltage(dac8563_dev_t* dev, dac8563_addr_et channel, float voltage_V);

ec_gt dac8563_software_reset(dac8563_dev_t* dev);

/*
example

dac8563_dev_t my_dac;

// 1. 初始化结构体配置
dac8563_init_t init_cfg = {
    .ref_mode = DAC8563_INT_REF_ENABLE,  // 开启内部高精度 2.5V 基准源
    .vref_V   = 2.5f,                    // 内部基准为 2.5V
    .gain     = 2.0f                     // DAC8563 的输出级默认带有 2 倍增益
};

// 2. 初始化器件 (底层会自动执行复位、开启基准源、激活输出通道)
dac8563_init(&my_dac, SPIB_BASE, &init_cfg);

// 3. 极速设置电压
// 让 DAC_A 输出 3.3V
dac8563_set_voltage(&my_dac, DAC8563_ADDR_DAC_A, 3.3f);

// 让 DAC_B 输出满量程 (5.0V)
dac8563_set_voltage(&my_dac, DAC8563_ADDR_DAC_B, 5.0f);

*/

#ifdef __cplusplus
}
#endif

#endif /* DAC8563_H */
