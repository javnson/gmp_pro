/**
 * @file    ad9834.c
 * @brief   Hardware-agnostic driver implementation for AD9834.
 */


#include <gmp_core.h>

#include <core/dev/application/ad9834.h>


#define AD9834_FREQ_MULTIPLIER  (268435456.0f)
#define AD9834_PHASE_MULTIPLIER (11.377777778f)

ec_gt ad9834_init(ad9834_dev_t* dev, spi_device_halt spi_node, float mclk_hz)
{
    if ((dev == NULL) || (mclk_hz <= 0.0f))
        return GMP_EC_GENERAL_ERROR;

    /* Bind the logical SPI device to this peripheral */
    dev->spi_node = spi_node;
    dev->mclk_hz = mclk_hz;

    dev->shadow_ctrl.all = 0;

    /* Hold device in reset, enable 28-bit writing via two consecutive writes */
    dev->shadow_ctrl.bits.reset = 1;
    dev->shadow_ctrl.bits.b28 = 1;
    dev->shadow_ctrl.bits.pin_sw = 0; /* Default to Software Control for flexibility */
    dev->shadow_ctrl.bits.ctrl_bits = 0;

    /* Automatically asserts/de-asserts CS and sends data MSB-first */
    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_set_frequency(ad9834_dev_t* dev, ad9834_reg_sel_et reg_sel, float freq_hz)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* FPU Calculation: F_reg = (F_out * 2^28) / MCLK */
    uint32_t freq_reg = (uint32_t)((freq_hz * AD9834_FREQ_MULTIPLIER) / dev->mclk_hz);
    freq_reg &= 0x0FFFFFFF;

    uint16_t lsb_14 = (uint16_t)(freq_reg & 0x3FFF);
    uint16_t msb_14 = (uint16_t)((freq_reg >> 14) & 0x3FFF);
    uint16_t cmd_mask = (reg_sel == AD9834_REG0) ? AD9834_FREQ_REG0 : AD9834_FREQ_REG1;

    ec_gt ret = gmp_hal_spi_dev_write_16b(dev->spi_node, cmd_mask | lsb_14, AD9834_CFG_TIMEOUT);
    if (ret != GMP_EC_OK)
        return ret;

    return gmp_hal_spi_dev_write_16b(dev->spi_node, cmd_mask | msb_14, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_set_phase(ad9834_dev_t* dev, ad9834_reg_sel_et reg_sel, float phase_deg)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    while (phase_deg >= 360.0f)
        phase_deg -= 360.0f;
    while (phase_deg < 0.0f)
        phase_deg += 360.0f;

    uint16_t phase_reg = (uint16_t)(phase_deg * AD9834_PHASE_MULTIPLIER) & 0x0FFF;
    uint16_t cmd_mask = (reg_sel == AD9834_REG0) ? AD9834_PHASE_REG0 : AD9834_PHASE_REG1;

    return gmp_hal_spi_dev_write_16b(dev->spi_node, cmd_mask | phase_reg, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_set_waveform(ad9834_dev_t* dev, ad9834_wave_et wave_type)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* Reset waveform related bits */
    dev->shadow_ctrl.bits.opbiten = 0;
    dev->shadow_ctrl.bits.mode = 0;
    dev->shadow_ctrl.bits.div2 = 0;
    dev->shadow_ctrl.bits.sign_pib = 0;

    switch (wave_type)
    {
    case AD9834_WAVE_SINE:
        break;
    case AD9834_WAVE_TRIANGLE:
        dev->shadow_ctrl.bits.mode = 1;
        break;
    case AD9834_WAVE_SQUARE_MSB:
        dev->shadow_ctrl.bits.opbiten = 1;
        dev->shadow_ctrl.bits.sign_pib = 1;
        dev->shadow_ctrl.bits.div2 = 1;
        break;
    case AD9834_WAVE_SQUARE_MSB_DIV2:
        dev->shadow_ctrl.bits.opbiten = 1;
        dev->shadow_ctrl.bits.sign_pib = 1;
        dev->shadow_ctrl.bits.div2 = 0;
        break;
    case AD9834_WAVE_SQUARE_COMPARATOR:
        dev->shadow_ctrl.bits.opbiten = 1;
        dev->shadow_ctrl.bits.sign_pib = 0; /* Route comparator to SIGN BIT OUT */
        break;
    default:
        return GMP_EC_GENERAL_ERROR;
    }

    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_set_control_source(ad9834_dev_t* dev, ad9834_ctrl_src_et src)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->shadow_ctrl.bits.pin_sw = (src == AD9834_CTRL_HARDWARE) ? 1 : 0;
    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_select_output_registers(ad9834_dev_t* dev, ad9834_reg_sel_et freq_sel, ad9834_reg_sel_et phase_sel)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    /* This only takes effect if pin_sw == 0 (Software control) */
    dev->shadow_ctrl.bits.fselect = (freq_sel == AD9834_REG1) ? 1 : 0;
    dev->shadow_ctrl.bits.pselect = (phase_sel == AD9834_REG1) ? 1 : 0;

    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_set_reset(ad9834_dev_t* dev, bool is_reset)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->shadow_ctrl.bits.reset = is_reset ? 1 : 0;
    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9834_CFG_TIMEOUT);
}

ec_gt ad9834_set_sleep(ad9834_dev_t* dev, bool dac_sleep, bool clock_sleep)
{
    if (dev == NULL)
        return GMP_EC_GENERAL_ERROR;

    dev->shadow_ctrl.bits.sleep12 = dac_sleep ? 1 : 0;
    dev->shadow_ctrl.bits.sleep1 = clock_sleep ? 1 : 0;
    return gmp_hal_spi_dev_write_16b(dev->spi_node, dev->shadow_ctrl.all, AD9834_CFG_TIMEOUT);
}
