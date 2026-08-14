
#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// Back stepping controller
#include <ctl/component/intrinsic/advance/back_stepping.h>

void ctl_init_backstepping(ctl_backstepping_controller_t* bc, const ctl_backstepping_init_t* init)
{
    // 1. Validate the denominator before calculating its reciprocal.
    gmp_ctl_assert(param_abs(init->K_p) > 1e-9f);

    // 2. Calculate the reciprocal in the parameter domain.
    parameter_gt inv_kp_val = 1.0f / init->K_p;

    // 3. Quantize the final coefficients into the control domain.
    bc->k1 = param2ctrl(init->k1);
    bc->tau_p = param2ctrl(init->tau_p);
    bc->inv_K_p = real2ctrl(inv_kp_val);

    // 4. Clear controller state.
    ctl_clear_backstepping(bc);
}
