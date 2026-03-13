/**
 * @file    ad9834.h
 * @brief   Hardware-agnostic driver for Analog Devices AD9834 75MHz DDS.
 * @note    This driver utilizes a generic SPI HAL interface and FPU acceleration.
 */

#ifndef AD9834_H
#define AD9834_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef AD9834_CFG_TIMEOUT
#define AD9834_CFG_TIMEOUT (10U) /**< Default SPI timeout in milliseconds */
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#define AD9834_FREQ_REG0  0x4000
#define AD9834_FREQ_REG1  0x8000
#define AD9834_PHASE_REG0 0xC000
#define AD9834_PHASE_REG1 0xE000
#define AD9834_B28_MASK   0x2000

/* ========================================================================= */
/* ==================== ENUMS & BIT-FIELDS ================================= */
/* ========================================================================= */

/** @brief Output Waveform Types (Enhanced for AD9834) */
typedef enum
{
    AD9834_WAVE_SINE = 0,
    AD9834_WAVE_TRIANGLE = 1,
    AD9834_WAVE_SQUARE_MSB = 2,       /**< Square wave from Phase Accumulator MSB */
    AD9834_WAVE_SQUARE_MSB_DIV2 = 3,  /**< Square wave from Phase Accumulator MSB / 2 */
    AD9834_WAVE_SQUARE_COMPARATOR = 4 /**< Square wave from Internal Comparator (AD9834 ONLY) */
} ad9834_wave_et;

/** @brief Register Selection (FREQ0/1 or PHASE0/1) */
typedef enum
{
    AD9834_REG0 = 0,
    AD9834_REG1 = 1
} ad9834_reg_sel_et;

/** @brief Control Source Selection */
typedef enum
{
    AD9834_CTRL_SOFTWARE = 0, /**< FSELECT/PSELECT controlled by SPI register bits */
    AD9834_CTRL_HARDWARE = 1  /**< FSELECT/PSELECT controlled by external pins */
} ad9834_ctrl_src_et;

/**
 * @brief Control Register Bit-field Definition
 */
typedef union {
    uint16_t all;
#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    struct
    {
        uint16_t reserved_0 : 1;
        uint16_t mode : 1;
        uint16_t reserved_2 : 1;
        uint16_t div2 : 1;
        uint16_t sign_pib : 1;
        uint16_t opbiten : 1;
        uint16_t sleep12 : 1;
        uint16_t sleep1 : 1;
        uint16_t reset : 1;
        uint16_t pin_sw : 1; /**< [9] Pin/Software control select */
        uint16_t pselect : 1;
        uint16_t fselect : 1;
        uint16_t hlb : 1;
        uint16_t b28 : 1;
        uint16_t ctrl_bits : 2;
    } bits;
#elif GMP_CHIP_ENDIAN == GMP_CHIP_BIG_ENDIAN
    struct
    {
        uint16_t ctrl_bits : 2;
        uint16_t b28 : 1;
        uint16_t hlb : 1;
        uint16_t fselect : 1;
        uint16_t pselect : 1;
        uint16_t pin_sw : 1;
        uint16_t reset : 1;
        uint16_t sleep1 : 1;
        uint16_t sleep12 : 1;
        uint16_t opbiten : 1;
        uint16_t sign_pib : 1;
        uint16_t div2 : 1;
        uint16_t reserved_2 : 1;
        uint16_t mode : 1;
        uint16_t reserved_0 : 1;
    } bits;
#endif
} ad9834_ctrl_reg_t;

/**
 * @brief Runtime Device Object for AD9834.
 */
typedef struct
{
    spi_device_halt spi_node;      /**< Layer 2 Logical SPI Device Handle (Includes CS info) */
    float mclk_hz;                 /**< Master Clock frequency in Hz (Max 75MHz) */
    ad9834_ctrl_reg_t shadow_ctrl; /**< Cached control register for read-modify-write */
} ad9834_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt ad9834_init(ad9834_dev_t* dev, spi_device_halt spi_node, float mclk_hz);

ec_gt ad9834_set_frequency(ad9834_dev_t* dev, ad9834_reg_sel_et reg_sel, float freq_hz);
ec_gt ad9834_set_phase(ad9834_dev_t* dev, ad9834_reg_sel_et reg_sel, float phase_deg);

ec_gt ad9834_set_waveform(ad9834_dev_t* dev, ad9834_wave_et wave_type);

ec_gt ad9834_set_control_source(ad9834_dev_t* dev, ad9834_ctrl_src_et src);
ec_gt ad9834_select_output_registers(ad9834_dev_t* dev, ad9834_reg_sel_et freq_sel, ad9834_reg_sel_et phase_sel);

ec_gt ad9834_set_reset(ad9834_dev_t* dev, bool is_reset);
ec_gt ad9834_set_sleep(ad9834_dev_t* dev, bool dac_sleep, bool clock_sleep);

/* example

ad9834_dev_t my_dds;

// 1. 初始化，传入外部有源晶振频率 (如 50MHz)
ad9834_init(&my_dds, SPIB_BASE, 50000000.0f);

// 2. 预加载频率 0 (例如：载波 1MHz) 和 频率 1 (例如：调制频移 1.5MHz)
ad9834_set_frequency(&my_dds, AD9834_REG0, 1000000.0f);
ad9834_set_frequency(&my_dds, AD9834_REG1, 1500000.0f);

// 3. 配置为正弦波输出
ad9834_set_waveform(&my_dds, AD9834_WAVE_SINE);

// 4. 将控制权移交给外部硬件引脚 (FSELECT 引脚)
ad9834_set_control_source(&my_dds, AD9834_CTRL_HARDWARE);

// 5. 解除复位，开始输出
ad9834_set_reset(&my_dds, false);

// -------------------------------------------------------------
// 接下来，您的 DSP/FPGA 只需要在硬件上翻转连到 AD9834 FSELECT 的 GPIO，
// 就能在 1MHz 和 1.5MHz 之间实现纳秒级 (0次 SPI 通信延迟) 的无缝切换！
// -------------------------------------------------------------

*/

#ifdef __cplusplus
}
#endif

#endif /* AD9834_H */
