/**
 * @file    ad9833.c
 * @brief   Hardware-agnostic driver implementation for AD9833.
 */

#include <gmp_core.h>

#include <core/dev/excitation/ad9833.h>

/* Helper macro for constant 2^28 used in frequency calculation */
#define AD9833_FREQ_MULTIPLIER (268435456.0f)
/* Helper macro for constant 4096/360 used in phase calculation */
#define AD9833_PHASE_MULTIPLIER (11.377777778f)

ec_gt ad9833_init(ad9833_dev_t* dev, spi_device_halt spi_node, float mclk_hz)
{
    if ((dev == NULL) || (mclk_hz <= 0.0f))
        return GMP_EC_GENERAL_ERROR;

    /* Bind the logical SPI device */
    dev->spi_node = spi_node;
    dev->mclk_hz = mclk_hz;

    /* Initialize shadow register with all 0s, then set mandatory defaults */
    dev->shadow_ctrl.all = 0;

    /* Hold device in reset, enable 28-bit writing via two consecutive writes */
    dev->shadow_ctrl.bits.reset = 1;
    dev->shadow_ctrl.bits.b28 = 1;
    dev->shadow_ctrl.bits.ctrl_bits = 0; /* Must be 00 */

    /* Automatically asserts/de-asserts CS and transmits via Layer 2 API */
    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9833_CFG_TIMEOUT);
}

ec_gt ad9833_set_frequency(ad9833_dev_t* dev, ad9833_reg_sel_et reg_sel, float freq_hz)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* 1. Hardware FPU Calculation: F_reg = (F_out * 2^28) / MCLK */
    uint32_t freq_reg = (uint32_t)((freq_hz * AD9833_FREQ_MULTIPLIER) / dev->mclk_hz);

    /* Clamp to 28-bits to be safe */
    freq_reg &= 0x0FFFFFFF;

    /* 2. Split into LSB 14-bits and MSB 14-bits */
    uint16_t lsb_14 = (uint16_t)(freq_reg & 0x3FFF);
    uint16_t msb_14 = (uint16_t)((freq_reg >> 14) & 0x3FFF);

    /* 3. Determine Register Command Mask */
    uint16_t cmd_mask = (reg_sel == AD9833_REG0) ? AD9833_FREQ_REG0 : AD9833_FREQ_REG1;

    ec_gt ret;

    /* Write LSBs (incorporates command mask) */
    ret = gmp_hal_spi_dev_write_16b(dev->spi_node, cmd_mask | lsb_14, AD9833_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    /* Write MSBs (incorporates command mask) */
    return gmp_hal_spi_dev_write_16b(dev->spi_node, cmd_mask | msb_14, AD9833_CFG_TIMEOUT);
}

ec_gt ad9833_set_phase(ad9833_dev_t* dev, ad9833_reg_sel_et reg_sel, float phase_deg)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Wrap phase between 0.0 and 360.0 degrees */
    while (phase_deg >= 360.0f)
        phase_deg -= 360.0f;
    while (phase_deg < 0.0f)
        phase_deg += 360.0f;

    /* FPU Calculation: P_reg = Phase_deg * (4096 / 360) */
    uint16_t phase_reg = (uint16_t)(phase_deg * AD9833_PHASE_MULTIPLIER) & 0x0FFF;

    uint16_t cmd_mask = (reg_sel == AD9833_REG0) ? AD9833_PHASE_REG0 : AD9833_PHASE_REG1;

    return gmp_hal_spi_dev_write_16b(dev->spi_node, cmd_mask | phase_reg, AD9833_CFG_TIMEOUT);
}

ec_gt ad9833_set_waveform(ad9833_dev_t* dev, ad9833_wave_et wave_type)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Clear all waveform related bits in the shadow register */
    dev->shadow_ctrl.bits.opbiten = 0;
    dev->shadow_ctrl.bits.mode = 0;
    dev->shadow_ctrl.bits.div2 = 0;

    switch (wave_type)
    {
    case AD9833_WAVE_SINE:
        /* Default all 0s for sine */
        break;
    case AD9833_WAVE_TRIANGLE:
        dev->shadow_ctrl.bits.mode = 1;
        break;
    case AD9833_WAVE_SQUARE:
        dev->shadow_ctrl.bits.opbiten = 1;
        break;
    case AD9833_WAVE_SQUARE_DIV2:
        dev->shadow_ctrl.bits.opbiten = 1;
        dev->shadow_ctrl.bits.div2 = 1;
        break;
    default:
        return GMP_EC_GENERAL_ERROR;
    }

    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9833_CFG_TIMEOUT);
}

ec_gt ad9833_select_output_registers(ad9833_dev_t* dev, ad9833_reg_sel_et freq_sel, ad9833_reg_sel_et phase_sel)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->shadow_ctrl.bits.fselect = (freq_sel == AD9833_REG1) ? 1 : 0;
    dev->shadow_ctrl.bits.pselect = (phase_sel == AD9833_REG1) ? 1 : 0;

    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9833_CFG_TIMEOUT);
}

ec_gt ad9833_set_reset(ad9833_dev_t* dev, bool is_reset)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->shadow_ctrl.bits.reset = is_reset ? 1 : 0;

    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9833_CFG_TIMEOUT);
}
