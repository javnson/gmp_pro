/**
 * @file ctl_common_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright GMP(c) 2024
 *
 */
#include <gmp_core.h>

#include <math.h>

//////////////////////////////////////////////////////////////////////////
// HCC regular

#include <ctl/component/intrinsic/lebesgue/hysteresis_controller.h>

void ctl_init_hysteresis_controller(ctl_hysteresis_controller_t* hcc, fast_gt flag_polarity, ctrl_gt half_width)
{
    hcc->flag_polarity = flag_polarity;
    hcc->half_width = half_width;
    hcc->target = 0;
    hcc->current = 0;
    // Initialize switch output to the state opposite of the upper bound state
    // to ensure predictable startup behavior.
    hcc->switch_out = 1 - flag_polarity;
}

//////////////////////////////////////////////////////////////////////////
// SMC controller
#include <ctl/component/intrinsic/lebesgue/smc.h>

void ctl_init_smc(ctl_smc_t* smc, ctrl_gt eta11, ctrl_gt eta12, ctrl_gt eta21, ctrl_gt eta22, ctrl_gt rho,
                  ctrl_gt lambda)
{
    smc->eta11 = eta11;
    smc->eta12 = eta12;
    smc->eta21 = eta21;
    smc->eta22 = eta22;
    smc->rho = rho;
    smc->lambda = lambda;

    smc->output = 0;
    smc->slide = 0;
}
