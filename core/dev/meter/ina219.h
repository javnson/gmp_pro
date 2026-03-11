/**
 * @file    ina219.h
 * @brief   Hardware-agnostic driver for TI INA219 Current/Power Monitor.
 * @note    This driver utilizes the GMP HAL I2C interface and manages the 
 * internal calibration math automatically.
 */

#ifndef INA219_H
#define INA219_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#ifndef INA219_CFG_TIMEOUT
#define INA219_CFG_TIMEOUT (10U)
#endif

/* INA219 Registers */
#define INA219_REG_CONFIG        0x00
#define INA219_REG_SHUNT_VOLTAGE 0x01
#define INA219_REG_BUS_VOLTAGE   0x02
#define INA219_REG_POWER         0x03
#define INA219_REG_CURRENT       0x04
#define INA219_REG_CALIBRATION   0x05

/* ========================================================================= */
/* ==================== ENUMS & ADDRESSING ================================= */
/* ========================================================================= */

/** @brief Address pin connection states */
typedef enum
{
    INA219_PIN_GND = 0x00,
    INA219_PIN_VS = 0x01,
    INA219_PIN_SDA = 0x02,
    INA219_PIN_SCL = 0x03
} ina219_addr_pin_et;

/** * @brief Calculate the 7-bit I2C address based on A1 and A0 pin connections.
 * @note  Base address is 0x40. Maximum is 0x4F.
 */
#define INA219_CALC_ADDR(a1, a0) (0x40 | (((a1) & 0x03) << 2) | ((a0) & 0x03))

/** @brief Bus Voltage Range (BRNG) */
typedef enum
{
    INA219_BRNG_16V = 0x00,
    INA219_BRNG_32V = 0x01
} ina219_brng_et;

/** @brief PGA Gain and Shunt Voltage Range (PG) */
typedef enum
{
    INA219_PGA_1_40MV = 0x00,  /**< +/- 40mV */
    INA219_PGA_2_80MV = 0x01,  /**< +/- 80mV */
    INA219_PGA_4_160MV = 0x02, /**< +/- 160mV */
    INA219_PGA_8_320MV = 0x03  /**< +/- 320mV */
} ina219_pga_et;

/** @brief ADC Resolution and Averaging (BADC / SADC) */
typedef enum
{
    INA219_ADC_9BIT_1S = 0x00,   /**< 9-bit, 84us */
    INA219_ADC_10BIT_1S = 0x01,  /**< 10-bit, 148us */
    INA219_ADC_11BIT_1S = 0x02,  /**< 11-bit, 276us */
    INA219_ADC_12BIT_1S = 0x03,  /**< 12-bit, 532us (Default) */
    INA219_ADC_12BIT_2S = 0x09,  /**< 12-bit, 2 samples, 1.06ms */
    INA219_ADC_12BIT_4S = 0x0A,  /**< 12-bit, 4 samples, 2.13ms */
    INA219_ADC_12BIT_16S = 0x0C, /**< 12-bit, 16 samples, 8.51ms */
    INA219_ADC_12BIT_128S = 0x0F /**< 12-bit, 128 samples, 68.10ms */
} ina219_adc_res_et;

/** @brief Operating Mode */
typedef enum
{
    INA219_MODE_POWER_DOWN = 0x00,
    INA219_MODE_SHUNT_TRIG = 0x01,
    INA219_MODE_BUS_TRIG = 0x02,
    INA219_MODE_SHUNT_BUS_TRIG = 0x03,
    INA219_MODE_ADC_OFF = 0x04,
    INA219_MODE_SHUNT_CONT = 0x05,
    INA219_MODE_BUS_CONT = 0x06,
    INA219_MODE_SHUNT_BUS_CONT = 0x07 /**< Default */
} ina219_mode_et;

/**
 * @brief Configuration Register 0x00 Bit-field Definition
 */
typedef union {
    uint16_t all;
#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    struct
    {
        uint16_t mode : 3;     /**< [2:0] Operating Mode */
        uint16_t sadc : 4;     /**< [6:3] Shunt ADC Resolution/Averaging */
        uint16_t badc : 4;     /**< [10:7] Bus ADC Resolution/Averaging */
        uint16_t pg : 2;       /**< [12:11] PGA Gain / Shunt Voltage Range */
        uint16_t brng : 1;     /**< [13] Bus Voltage Range */
        uint16_t reserved : 1; /**< [14] Reserved, always 0 */
        uint16_t rst : 1;      /**< [15] Reset Bit */
    } bits;
#elif GMP_CHIP_ENDIAN == GMP_CHIP_BIG_ENDIAN
    struct
    {
        uint16_t rst : 1;
        uint16_t reserved : 1;
        uint16_t brng : 1;
        uint16_t pg : 2;
        uint16_t badc : 4;
        uint16_t sadc : 4;
        uint16_t mode : 3;
    } bits;
#endif
} ina219_config_reg_t;

/**
 * @brief Runtime Device Object for INA219.
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    float current_lsb_mA; /**< Cached Current LSB (in mA) to calculate actual current */
    float power_lsb_mW;   /**< Cached Power LSB (in mW) to calculate actual power */
} ina219_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt ina219_init(ina219_dev_t* dev, iic_halt bus, addr16_gt dev_addr, ina219_config_reg_t init_cfg);

/**
 * @brief  Calibrate the INA219. MUST be called before reading current or power.
 * @param  dev              Pointer to the device object.
 * @param  cal_val          The computed calibration register value (0x05).
 * @param  current_lsb_mA   The chosen Current LSB in milliamperes.
 */
ec_gt ina219_calibrate(ina219_dev_t* dev, uint16_t cal_val, float current_lsb_mA);

ec_gt ina219_read_bus_voltage(ina219_dev_t* dev, float* voltage_V_ret);
ec_gt ina219_read_shunt_voltage(ina219_dev_t* dev, float* voltage_mV_ret);

ec_gt ina219_read_current(ina219_dev_t* dev, float* current_mA_ret);
ec_gt ina219_read_power(ina219_dev_t* dev, float* power_mW_ret);

#ifdef __cplusplus
}
#endif

#endif /* INA219_H */
