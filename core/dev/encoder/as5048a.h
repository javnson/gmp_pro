
// AS5048A = AS5048 with SPI interface
//

// The AS5048 is a magnetic Hall sensor system manufactured in a CMOS process. A lateral Hall sensor array is used to
// measure the magnetic field components perpendicular to the surface of the chip.The AS5048 is uses self - calibration
// methods to eliminate signal offset and sensitivity drifts.
//
//  + SPI MODE = 1
// The 16 bit SPI Interface enables read / write access to the register blocks and is compatible to a standard micro
// controller interface. The SPI is active as soon as CSn is pulled low. The AS5048A then reads the digital value on the
// MOSI(master out slave in) input with every falling edge of CLK and writes on its MISO(master in slave out) output
// with the rising edge. After 16 clock cycles CSn has to be set back to a high status in order to reset some parts of
// the interface core.
//

// Pure read mode: write 0xFF 0xFF and meanwhile read the result.
//

// register access mode:
//

// STM32 Demo
/*
uint8_t enc_req[2] = {0xFF, 0xFF};
uint16_t enc_res = 0;
HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
HAL_SPI_TransmitReceive(&hspi2, enc_req, (uint8_t *)&enc_res, 2, 10);
HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
ctl_step_pos_encoder(&pos_enc, 0x3FFF - (gmp_l2b16(enc_res) & 0x3FFF));
*/

#include <gmp_core.h>

#include <ctl/component/motor_control/basic/encoder.h>


#ifndef _FILE_AS5048A_H_
#define _FILE_AS5048A_H_

// User should input revolution of motor
typedef struct _tag_ext_as5048a_encoder_t
{
    // output interface, encoder output interface
    rotation_ift encif;

    // input raw encoder data
    uint32_t raw;

    // offset,
    // position - offset means the true position
    ctrl_gt offset;

    // pole_pairs
    // poles*(position - offset) = Electrical position
    uint16_t pole_pairs;

    // uint32_t p.u. base value
    uint32_t position_base;

    // record last position to calculate revolutions.
    ctrl_gt last_pos;

    // physical interface
    spi_halt spi;
    gpio_halt ncs;

} ext_as5048a_encoder_t;

void ctl_init_as5048a_pos_encoder(ext_as5048a_encoder_t *enc, uint16_t poles, spi_halt spi, gpio_halt ncs);

// This function may calculate and get angle from encoder source data
GMP_STATIC_INLINE
ctrl_gt ctl_step_as5048a_pos_encoder(ext_as5048a_encoder_t *enc)
{
    uint8_t enc_req[2] = {0xFF, 0xFF};
    uint16_t enc_res = 0;

    gmp_hal_gpio_write(enc->ncs, 0);
    gmp_hal_spi_read_write(enc->spi, (data_gt *)enc_req, (data_gt *)&enc_res, 2);
    gmp_hal_gpio_write(enc->ncs, 1);

    // record raw data
    enc->raw = 0x3FFF - (gmp_l2b16(enc_res) & 0x3FFF);

    // calculate absolute mechanical position
    enc->encif.position = ctl_div(enc->raw, enc->position_base);

    // calculate electrical position
    ctrl_gt elec_pos = enc->pole_pairs * (enc->encif.position + GMP_CONST_1 - enc->offset);
    ctrl_gt elec_pos_pu = ctrl_mod_1(elec_pos);

    // record electrical position data
    enc->encif.elec_position = elec_pos_pu;

    // judge if multi turn count event has occurred.
    if (enc->encif.position - enc->last_pos > GMP_CONST_1_OVER_2)
    {
        // position has a negative step
        enc->encif.revolutions -= 1;
    }
    if (enc->last_pos - enc->encif.position > GMP_CONST_1_OVER_2)
    {
        // position has a positive step
        enc->encif.revolutions += 1;
    }

    // record last position
    enc->last_pos = enc->encif.position;

    return enc->encif.elec_position;
}

// Set offset of encoder
GMP_STATIC_INLINE
void ctl_set_as5048a_pos_encoder_offset(ext_as5048a_encoder_t *enc, uint32_t raw)
{
    enc->offset = float2ctrl(raw / enc->position_base);
}


#endif // _FILE_AS5048A_H_
