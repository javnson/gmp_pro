#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/inv/inv_gfm_virtual_impedance.h>

void ctl_init_inv_gfm_virtual_impedance(
    inv_gfm_virtual_impedance_t* impedance,
    const inv_gfm_virtual_impedance_init_t* init)
{
    gmp_ctl_assert(impedance);
    gmp_ctl_assert(init);
    gmp_ctl_assert(init->resistance_pu >= 0.0f);
    gmp_ctl_assert(init->voltage_limit >= 0.0f);

    impedance->idq = NULL;
    impedance->resistance_pu = float2ctrl(init->resistance_pu);
    impedance->reactance_pu = float2ctrl(init->reactance_pu);
    impedance->voltage_limit = float2ctrl(init->voltage_limit);
    ctl_vector2_clear(&impedance->vdq_base);
    ctl_vector2_clear(&impedance->vdq_ref);
    impedance->flag_enable = 0;
}
