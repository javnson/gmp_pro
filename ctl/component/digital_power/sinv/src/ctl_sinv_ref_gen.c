
#include <ctl/component/digital_power/sinv/sinv_ref_gen.h>

void ctl_init_sinv_ref_gen(ctl_sinv_ref_gen_t* gen, parameter_gt i_max, parameter_gt v_mag_min)
{
    gen->i_max = float2ctrl(i_max);
    gen->i_max_sq = ctl_mul(gen->i_max, gen->i_max);
    gen->v_mag_min = float2ctrl(v_mag_min);

    ctl_clear_sinv_ref_gen(gen);
}
