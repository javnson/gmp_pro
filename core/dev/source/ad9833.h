/**
 * @file    ad9833.h
 * @brief   Hardware-agnostic driver for Analog Devices AD9833 Waveform Generator.
 * @note    This driver utilizes a generic SPI HAL interface. Since AD9833 is 
 * write-only, all configurations are cached in the device object.
 */

#ifndef AD9833_H
#define AD9833_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Assume a generic SPI HAL header exists with similar philosophies to IIC */
#include "gmp_hal_spi.h"

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#define AD9833_FREQ_REG0  0x4000 /**< FREQ0 Register Address Mask */
#define AD9833_FREQ_REG1  0x8000 /**< FREQ1 Register Address Mask */
#define AD9833_PHASE_REG0 0xC000 /**< PHASE0 Register Address Mask */
#define AD9833_PHASE_REG1 0xE000 /**< PHASE1 Register Address Mask */
#define AD9833_B28_MASK   0x2000 /**< 28-bit write identifier */

/* ========================================================================= */
/* ==================== ENUMS & BIT-FIELDS ================================= */
/* ========================================================================= */

/** @brief Output Waveform Types */
typedef enum
{
    AD9833_WAVE_SINE = 0,
    AD9833_WAVE_TRIANGLE = 1,
    AD9833_WAVE_SQUARE = 2,     /**< Square wave (MSB) */
    AD9833_WAVE_SQUARE_DIV2 = 3 /**< Square wave (MSB / 2) */
} ad9833_wave_et;

/** @brief Register Selection (FREQ0/1 or PHASE0/1) */
typedef enum
{
    AD9833_REG0 = 0,
    AD9833_REG1 = 1
} ad9833_reg_sel_et;

/**
 * @brief Control Register Bit-field Definition
 * @note  Strictly handles C bit-field packing order based on CPU endianness.
 */
typedef union {
    uint16_t all;
#if GMP_CHIP_ENDIAN == GMP_CHIP_LITTLE_ENDIAN
    struct
    {
        uint16_t reserved_0 : 1; /**< [0] Reserved, must be 0 */
        uint16_t mode : 1;       /**< [1] Output mode (0=Sine/Square, 1=Triangle) */
        uint16_t reserved_2 : 1; /**< [2] Reserved, must be 0 */
        uint16_t div2 : 1;       /**< [3] MSB/2 output (Square wave) */
        uint16_t sign_pib : 1;   /**< [4] Comparator polarity */
        uint16_t opbiten : 1;    /**< [5] Logic output enable (Square wave) */
        uint16_t sleep12 : 1;    /**< [6] Power down DAC */
        uint16_t sleep1 : 1;     /**< [7] Power down internal clock */
        uint16_t reset : 1;      /**< [8] Reset internal phase accumulator */
        uint16_t pin_sw : 1;     /**< [9] Pin/Software control select */
        uint16_t pselect : 1;    /**< [10] Phase register select (0=PHASE0, 1=PHASE1) */
        uint16_t fselect : 1;    /**< [11] Freq register select (0=FREQ0, 1=FREQ1) */
        uint16_t hlb : 1;        /**< [12] High/Low byte select */
        uint16_t b28 : 1;        /**< [13] Two consecutive writes for 28-bit freq */
        uint16_t ctrl_bits : 2;  /**< [15:14] Must be 00 for Control Register */
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
} ad9833_ctrl_reg_t;

/**
 * @brief Runtime Device Object for AD9833.
 */
typedef struct
{
    spi_halt bus;                  /**< SPI Hardware Handle (abstracted) */
    float mclk_hz;                 /**< Master Clock frequency in Hz (e.g., 25000000.0f) */
    ad9833_ctrl_reg_t shadow_ctrl; /**< Cached control register for read-modify-write */
} ad9833_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief Initializes the AD9833 (puts it in reset state temporarily).
 */
ec_gt ad9833_init(ad9833_dev_t* dev, spi_halt bus, float mclk_hz);

/**
 * @brief Sets the frequency of a specified register.
 * @note  Leverages FPU for extreme fast calculation using float.
 */
ec_gt ad9833_set_frequency(ad9833_dev_t* dev, ad9833_reg_sel_et reg_sel, float freq_hz);

/**
 * @brief Sets the phase of a specified register.
 * @param phase_deg Phase in degrees (0.0 to 360.0).
 */
ec_gt ad9833_set_phase(ad9833_dev_t* dev, ad9833_reg_sel_et reg_sel, float phase_deg);

/**
 * @brief Sets the output waveform shape.
 */
ec_gt ad9833_set_waveform(ad9833_dev_t* dev, ad9833_wave_et wave_type);

/**
 * @brief Selects which FREQ and PHASE registers are routed to the output.
 */
ec_gt ad9833_select_output_registers(ad9833_dev_t* dev, ad9833_reg_sel_et freq_sel, ad9833_reg_sel_et phase_sel);

/**
 * @brief Software reset control.
 * @param is_reset True to hold in reset, False to release and generate output.
 */
ec_gt ad9833_set_reset(ad9833_dev_t* dev, bool is_reset);



/* example

ad9833_dev_t my_dds;

// 1. 初始化，传入外部 25MHz 有源晶振频率
ad9833_init(&my_dds, SPIA_BASE, 25000000.0f);

// 2. 利用 DSP 的 FPU 瞬间计算并写入 10.5kHz 的频率数据到 FREQ0 寄存器
ad9833_set_frequency(&my_dds, AD9833_REG0, 10500.0f);

// 3. 写入 90 度相位到 PHASE0 寄存器
ad9833_set_phase(&my_dds, AD9833_REG0, 90.0f);

// 4. 配置输出类型为正弦波
ad9833_set_waveform(&my_dds, AD9833_WAVE_SINE);

// 5. 将内部路由切换到 FREQ0 和 PHASE0，并解除复位，波形瞬间输出！
ad9833_select_output_registers(&my_dds, AD9833_REG0, AD9833_REG0);
ad9833_set_reset(&my_dds, false);

*/

#ifdef __cplusplus
}
#endif

#endif /* AD9833_H */
