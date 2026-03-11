/**
 * @file    bmp280.h
 * @brief   Hardware-agnostic driver for Bosch BMP280 Pressure and Temperature sensor.
 * @note    This driver strictly follows the Bosch compensation formulas and 
 * utilizes the GMP HAL I2C interface.
 */

#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#ifndef BMP280_CFG_TIMEOUT
#define BMP280_CFG_TIMEOUT (20U) /**< I2C transaction timeout */
#endif

/** * @brief Helper macro to calculate 7-bit I2C address based on SDO pin.
 * @param sdo_pin State of the SDO pin (0 for GND, 1 for VDDIO)
 * @return 0x76 if SDO=0, 0x77 if SDO=1
 */
#define BMP280_CALC_ADDR(sdo_pin) ((sdo_pin) ? 0x77 : 0x76)

/* ========================================================================= */
/* ==================== REGISTERS & COMMANDS =============================== */
/* ========================================================================= */

#define BMP280_REG_CALIB_START 0x88
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7 /**< Start of 6-byte data burst */

#define BMP280_EXPECTED_ID    0x58
#define BMP280_CMD_SOFT_RESET 0xB6

/* ========================================================================= */
/* ==================== ENUMS & BIT-FIELDS ================================= */
/* ========================================================================= */

/** @brief Oversampling settings for Temperature and Pressure */
typedef enum
{
    BMP280_OSRS_SKIPPED = 0x00,
    BMP280_OSRS_1X = 0x01,
    BMP280_OSRS_2X = 0x02,
    BMP280_OSRS_4X = 0x03,
    BMP280_OSRS_8X = 0x04,
    BMP280_OSRS_16X = 0x05
} bmp280_osrs_et;

/** @brief Power modes */
typedef enum
{
    BMP280_MODE_SLEEP = 0x00,
    BMP280_MODE_FORCED = 0x01,
    BMP280_MODE_NORMAL = 0x03
} bmp280_mode_et;

/** @brief Standby time in Normal mode */
typedef enum
{
    BMP280_TSB_0_5_MS = 0x00,
    BMP280_TSB_62_5_MS = 0x01,
    BMP280_TSB_125_MS = 0x02,
    BMP280_TSB_250_MS = 0x03,
    BMP280_TSB_500_MS = 0x04,
    BMP280_TSB_1000_MS = 0x05,
    BMP280_TSB_2000_MS = 0x06,
    BMP280_TSB_4000_MS = 0x07
} bmp280_standby_et;

/** @brief IIR Filter coefficient */
typedef enum
{
    BMP280_FILTER_OFF = 0x00,
    BMP280_FILTER_2 = 0x01,
    BMP280_FILTER_4 = 0x02,
    BMP280_FILTER_8 = 0x03,
    BMP280_FILTER_16 = 0x04
} bmp280_filter_et;

/**
 * @brief Control Measurement Register (0xF4) Bit-field
 */
typedef union {
    uint8_t all;
#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    struct
    {
        uint8_t mode : 2;   /**< [1:0] Power Mode */
        uint8_t osrs_p : 3; /**< [4:2] Pressure Oversampling */
        uint8_t osrs_t : 3; /**< [7:5] Temperature Oversampling */
    } bits;
#elif GMP_CHIP_ENDIAN == GMP_CHIP_BIG_ENDIAN
    struct
    {
        uint8_t osrs_t : 3;
        uint8_t osrs_p : 3;
        uint8_t mode : 2;
    } bits;
#endif
} bmp280_ctrl_meas_t;

/**
 * @brief Configuration Register (0xF5) Bit-field
 */
typedef union {
    uint8_t all;
#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    struct
    {
        uint8_t spi3w_en : 1; /**< [0] Enable 3-wire SPI (ignored for I2C) */
        uint8_t reserved : 1; /**< [1] Reserved */
        uint8_t filter : 3;   /**< [4:2] IIR Filter coefficient */
        uint8_t t_sb : 3;     /**< [7:5] Standby time */
    } bits;
#elif GMP_CHIP_ENDIAN == GMP_CHIP_BIG_ENDIAN
    struct
    {
        uint8_t t_sb : 3;
        uint8_t filter : 3;
        uint8_t reserved : 1;
        uint8_t spi3w_en : 1;
    } bits;
#endif
} bmp280_config_t;

/**
 * @brief Initialization payload combining both config registers
 */
typedef struct
{
    bmp280_ctrl_meas_t ctrl;
    bmp280_config_t config;
} bmp280_init_t;

/**
 * @brief Factory Calibration Data Structure
 */
typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data_t;

/**
 * @brief Runtime Device Object for BMP280.
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    bmp280_calib_data_t calib; /**< Cached factory calibration data */
    int32_t t_fine;            /**< Global fine temperature (needed for pressure comp) */
} bmp280_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt bmp280_init(bmp280_dev_t* dev, iic_halt bus, addr16_gt dev_addr, bmp280_init_t init_cfg);

ec_gt bmp280_reset(bmp280_dev_t* dev);

ec_gt bmp280_get_id(bmp280_dev_t* dev, uint8_t* id_ret);

ec_gt bmp280_read_temperature(bmp280_dev_t* dev, float* temp_c_ret);

ec_gt bmp280_read_pressure(bmp280_dev_t* dev, float* pressure_pa_ret);

/**
 * @brief Burst read and compensate both temperature and pressure (Most efficient way).
 */
ec_gt bmp280_read_temp_and_pressure(bmp280_dev_t* dev, float* temp_c_ret, float* pressure_pa_ret);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
