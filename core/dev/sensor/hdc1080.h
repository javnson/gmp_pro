/**
 * @file    hdc1080.h
 * @brief   Hardware-agnostic driver for TI HDC1080 Temperature & Humidity Sensor.
 * @note    This driver utilizes the GMP HAL I2C interface.
 */

#ifndef HDC1080_H
#define HDC1080_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#ifndef HDC1080_CFG_TIMEOUT
#define HDC1080_CFG_TIMEOUT (20U) /**< General I2C timeout in ms */
#endif

#define HDC1080_I2C_ADDR_DEFAULT 0x40 /**< Default 7-bit I2C address */

/* Registers */
#define HDC1080_REG_TEMPERATURE     0x00
#define HDC1080_REG_HUMIDITY        0x01
#define HDC1080_REG_CONFIGURATION   0x02
#define HDC1080_REG_SERIAL_ID_1     0xFB
#define HDC1080_REG_SERIAL_ID_2     0xFC
#define HDC1080_REG_SERIAL_ID_3     0xFD
#define HDC1080_REG_MANUFACTURER_ID 0xFE
#define HDC1080_REG_DEVICE_ID       0xFF

/* Expected IDs */
#define HDC1080_EXPECTED_MANUF_ID  0x5449 /**< "TI" in ASCII */
#define HDC1080_EXPECTED_DEVICE_ID 0x1050

/* ========================================================================= */
/* ==================== ENUMS & DATA STRUCTURES ============================ */
/* ========================================================================= */

/* ========================================================================= */
/* ==================== ENUMS & DATA STRUCTURES ============================ */
/* ========================================================================= */

/**
 * @brief Configuration Register 0x02 Bit-field Definition
 * @note  Strictly handles C bit-field packing order based on CPU endianness 
 * to ensure cross-platform compatibility.
 */
typedef union {
    uint16_t all;

#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    /* Little-Endian: Bit-fields are packed from LSB (Bit 0) to MSB (Bit 15) */
    struct
    {
        uint16_t reserved_7_0 : 8;   /**< [7:0] Reserved, must be 0 */
        uint16_t hum_res : 2;        /**< [9:8] Humidity Resolution (00=14b, 01=11b, 10=8b) */
        uint16_t temp_res : 1;       /**< [10] Temperature Resolution (0=14b, 1=11b) */
        uint16_t battery_status : 1; /**< [11] Battery Status (Read-Only) */
        uint16_t mode : 1;           /**< [12] Acquisition Mode (0=Separate, 1=Sequence) */
        uint16_t heater : 1;         /**< [13] Heater Enable (0=Disable, 1=Enable) */
        uint16_t reserved_14 : 1;    /**< [14] Reserved, must be 0 */
        uint16_t soft_reset : 1;     /**< [15] Software Reset (1=Trigger Reset) */
    } bits;
#elif GMP_CHIP_ENDIAN == GMP_CHIP_BIG_ENDIAN
    /* Big-Endian: Bit-fields are packed from MSB (Bit 15) to LSB (Bit 0) */
    struct
    {
        uint16_t soft_reset : 1;     /**< [15] Software Reset */
        uint16_t reserved_14 : 1;    /**< [14] Reserved */
        uint16_t heater : 1;         /**< [13] Heater Enable */
        uint16_t mode : 1;           /**< [12] Acquisition Mode */
        uint16_t battery_status : 1; /**< [11] Battery Status */
        uint16_t temp_res : 1;       /**< [10] Temperature Resolution */
        uint16_t hum_res : 2;        /**< [9:8] Humidity Resolution */
        uint16_t reserved_7_0 : 8;   /**< [7:0] Reserved */
    } bits;
#else
#error("You should specify at least GMP_CHIP_LITTLE_ENDIAN or GMP_CHIP_BIG_ENDIAN")
#endif
} hdc1080_config_reg_t;

/** @brief Acquisition mode (Bit 12) */
typedef enum {
    HDC1080_MODE_SEPARATE = 0,  /**< Temp or Hum is acquired individually */
    HDC1080_MODE_SEQUENCE = 1   /**< Both are acquired in sequence (Temp first, then Hum) */
} hdc1080_mode_et;

/**
 * @brief Runtime Device Object for HDC1080.
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    hdc1080_mode_et mode; /**< Cached mode to determine reading sequence */
} hdc1080_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief   Initialize the HDC1080 device.
 * @note    Pass the config union BY VALUE since it's just a 16-bit integer.
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     bus       I2C hardware handle.
 * @param[in]     dev_addr  7-bit device address.
 * @param[in]     init_cfg  Configuration register union (passed by value).
 * * @return  ec_gt           GMP_EC_OK on success.
 */
ec_gt hdc1080_init(hdc1080_dev_t* dev, iic_halt bus, addr16_gt dev_addr, hdc1080_config_reg_t init_cfg);

ec_gt hdc1080_read_config(hdc1080_dev_t* dev, uint16_t* config_ret);

ec_gt hdc1080_read_device_ids(hdc1080_dev_t* dev, uint16_t* manuf_id_ret, uint16_t* dev_id_ret);
ec_gt hdc1080_read_serial_id(hdc1080_dev_t* dev, uint64_t* serial_ret);

ec_gt hdc1080_read_temperature(hdc1080_dev_t* dev, float* temp_c_ret);
ec_gt hdc1080_read_humidity(hdc1080_dev_t* dev, float* hum_rh_ret);

/**
 * @brief Special function to read both T and H in one transaction.
 * @note  Only valid if the device was initialized in HDC1080_MODE_SEQUENCE.
 */
ec_gt hdc1080_read_temp_hum_sequence(hdc1080_dev_t* dev, float* temp_c_ret, float* hum_rh_ret);

#ifdef __cplusplus
}
#endif

#endif /* HDC1080_H */
