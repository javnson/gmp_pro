
#include <gmp_core.h>

#include <ext/encoder/as5048/as5048a.h>

void ctl_init_as5048a_pos_encoder(ext_as5048a_encoder_t *enc, uint16_t poles, spi_halt spi, gpio_halt ncs)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = 0x3FFF;
    enc->ncs = ncs;
    enc->spi = spi;
}
