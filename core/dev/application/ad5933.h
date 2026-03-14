/**
 * @file    ad5933.h
 * @brief   Hardware-agnostic driver for Analog Devices AD5933 Impedance Converter.
 * @note    This driver utilizes the GMP HAL I2C interface and optimizes
 * float calculations into 64-bit integer math for embedded performance.
 */

#ifndef AD5933_H
#define AD5933_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#ifndef AD5933_CFG_TIMEOUT
#define AD5933_CFG_TIMEOUT (20U)
#endif

#define AD5933_I2C_ADDR 0x0D /**< Fixed 7-bit I2C address */

/* AD5933 Specific Block Commands */
#define AD5933_BLOCK_WRITE  0xA0
#define AD5933_BLOCK_READ   0xA1
#define AD5933_ADDR_POINTER 0xB0 /**< Crucial command to set read pointer */

/* Registers */
#define AD5933_REG_CTRL_HB         0x80
#define AD5933_REG_CTRL_LB         0x81
#define AD5933_REG_FREQ_START      0x82 /**< 24-bit (0x82, 0x83, 0x84) */
#define AD5933_REG_FREQ_INC        0x85 /**< 24-bit (0x85, 0x86, 0x87) */
#define AD5933_REG_INC_NUM         0x88 /**< 9-bit  (0x88, 0x89) */
#define AD5933_REG_SETTLING_CYCLES 0x8A /**< 9-bit + multiplier (0x8A, 0x8B) */
#define AD5933_REG_STATUS          0x8F
#define AD5933_REG_TEMP_DATA       0x92 /**< 14-bit (0x92, 0x93) */
#define AD5933_REG_REAL_DATA       0x94 /**< 16-bit (0x94, 0x95) */
#define AD5933_REG_IMAG_DATA       0x96 /**< 16-bit (0x96, 0x97) */

/* Default Clock */
#define AD5933_INTERNAL_SYS_CLK 16000000ul /**< 16MHz Internal Clock */

/* ========================================================================= */
/* ==================== ENUMS & BIT-FIELDS ================================= */
/* ========================================================================= */

/** @brief Operating Function (Control Register HB [7:4]) */
typedef enum
{
    AD5933_FUNC_NOP = 0x00,
    AD5933_FUNC_INIT_START_FREQ = 0x01,
    AD5933_FUNC_START_SWEEP = 0x02,
    AD5933_FUNC_INC_FREQ = 0x03,
    AD5933_FUNC_REPEAT_FREQ = 0x04,
    AD5933_FUNC_MEASURE_TEMP = 0x09,
    AD5933_FUNC_POWER_DOWN = 0x0A,
    AD5933_FUNC_STANDBY = 0x0B
} ad5933_func_et;

/** @brief Output Excitation Range (Control Register HB [2:1]) */
typedef enum
{
    AD5933_RANGE_2000mVpp = 0x00,
    AD5933_RANGE_200mVpp = 0x01,
    AD5933_RANGE_400mVpp = 0x02,
    AD5933_RANGE_1000mVpp = 0x03
} ad5933_range_et;

/** @brief PGA Gain (Control Register HB [0]) */
typedef enum
{
    AD5933_GAIN_X5 = 0x00,
    AD5933_GAIN_X1 = 0x01
} ad5933_gain_et;

/** @brief Clock Source (Control Register LB [3]) */
typedef enum
{
    AD5933_CLK_INTERNAL = 0x00,
    AD5933_CLK_EXTERNAL = 0x01
} ad5933_clk_src_et;

/** @brief Settling Cycles Multiplier */
typedef enum
{
    AD5933_SETTLING_X1 = 0x00,
    AD5933_SETTLING_X2 = 0x01,
    AD5933_SETTLING_X4 = 0x03
} ad5933_settling_mult_et;

/** @brief Status Register Bit Masks */
#define AD5933_STAT_TEMP_VALID 0x01
#define AD5933_STAT_DATA_VALID 0x02
#define AD5933_STAT_SWEEP_DONE 0x04

/**
 * @brief Control Register High Byte (0x80)
 */
typedef union {
    uint8_t all;
#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    struct
    {
        uint8_t pga_gain : 1;
        uint8_t range : 2;
        uint8_t reserved : 1;
        uint8_t function : 4;
    } bits;
#elif GMP_CHIP_ENDIAN == GMP_CHIP_BIG_ENDIAN
    struct
    {
        uint8_t function : 4;
        uint8_t reserved : 1;
        uint8_t range : 2;
        uint8_t pga_gain : 1;
    } bits;
#endif
} ad5933_ctrl_hb_t;

/**
 * @brief Initialization payload
 */
typedef struct
{
    ad5933_clk_src_et clk_src;
    uint32_t ext_clk_freq; /**< Set to 0 if using internal clock */
    ad5933_range_et range;
    ad5933_gain_et gain;
} ad5933_init_t;

/**
 * @brief Runtime Device Object
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    uint32_t sys_clk; /**< Cached active system clock (Hz) */
    ad5933_range_et current_range;
    ad5933_gain_et current_gain;
    float gain_factor; /**< Calibration gain factor */
} ad5933_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt ad5933_init(ad5933_dev_t* dev, iic_halt bus, const ad5933_init_t* init_cfg);

ec_gt ad5933_reset(ad5933_dev_t* dev);

ec_gt ad5933_set_function(ad5933_dev_t* dev, ad5933_func_et function);

ec_gt ad5933_config_sweep(ad5933_dev_t* dev, uint32_t start_freq_hz, uint32_t inc_freq_hz, uint16_t inc_num);

ec_gt ad5933_set_settling_time(ad5933_dev_t* dev, ad5933_settling_mult_et multiplier, uint16_t cycles);

ec_gt ad5933_read_temperature(ad5933_dev_t* dev, float* temp_c_ret);

ec_gt ad5933_get_complex_data(ad5933_dev_t* dev, int16_t* real_ret, int16_t* imag_ret);

ec_gt ad5933_calculate_gain_factor(ad5933_dev_t* dev, float cal_impedance_ohms, ad5933_func_et function);

ec_gt ad5933_calculate_impedance(ad5933_dev_t* dev, ad5933_func_et function, float* impedance_ohms_ret);

/*

ad5933_dev_t my_analyzer;
ad5933_init_t init_cfg = {
    .clk_src      = AD5933_CLK_INTERNAL,
    .ext_clk_freq = 0,
    .range        = AD5933_RANGE_2000mVpp,
    .gain         = AD5933_GAIN_X1
};

// 1. 初始化器件
ad5933_init(&my_analyzer, I2CA_BASE, &init_cfg);

// 2. 配置扫频：从 30kHz 开始，每次步进 1kHz，共 10 次
ad5933_config_sweep(&my_analyzer, 30000, 1000, 10);
ad5933_set_settling_time(&my_analyzer, AD5933_SETTLING_X1, 15); // 稳定时间 15 个周期

// 3. 校准系统增益因子 (假设此时接入了一个 200kΩ 的高精度标准电阻)
ad5933_set_function(&my_analyzer, AD5933_FUNC_INIT_START_FREQ);
// 适当延时等待模拟电路稳定...
ad5933_calculate_gain_factor(&my_analyzer, 200000.0, AD5933_FUNC_START_SWEEP);

// 4. 换上待测电阻/电容，进行扫频测量
double impedance_result;
ad5933_set_function(&my_analyzer, AD5933_FUNC_INIT_START_FREQ);
// 适当延时...
ad5933_calculate_impedance(&my_analyzer, AD5933_FUNC_START_SWEEP, &impedance_result);
// 这时 impedance_result 就是 30kHz 下的阻抗值

// 继续步进测量下一个频率点 (31kHz)
ad5933_calculate_impedance(&my_analyzer, AD5933_FUNC_INC_FREQ, &impedance_result);

*/


#ifdef __cplusplus
}
#endif

#endif /* AD5933_H */
