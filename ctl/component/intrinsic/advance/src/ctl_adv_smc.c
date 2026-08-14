#include <ctl/math_block/gmp_math.h>

//////////////////////////////////////////////////////////////////////////
// SMC controller
#include <ctl/component/intrinsic/advance/smc.h>

void ctl_init_smc(ctl_smc_t* smc, parameter_gt eta11, parameter_gt eta12, parameter_gt eta21, parameter_gt eta22,
                  parameter_gt rho, parameter_gt lambda, parameter_gt phi)
{
    // Convert initialization gains into the control domain.
    smc->eta11 = real2ctrl(eta11);
    smc->eta12 = real2ctrl(eta12);
    smc->eta21 = real2ctrl(eta21);
    smc->eta22 = real2ctrl(eta22);
    smc->rho = real2ctrl(rho);
    smc->lambda = real2ctrl(lambda);

    // Store the boundary-layer reciprocal to avoid division in the ISR.
    if (phi < 1e-6f)
    {
        phi = CTL_PARAM_CONST_EPSILON; // Protect the reciprocal from zero.
    }
    smc->inv_phi = real2ctrl(1.0f / phi);

    ctl_clear_smc(smc);
}
