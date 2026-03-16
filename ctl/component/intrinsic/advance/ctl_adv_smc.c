//////////////////////////////////////////////////////////////////////////
// SMC controller
#include <ctl/component/intrinsic/advance/smc.h>

void ctl_init_smc(ctl_smc_t* smc, parameter_gt eta11, parameter_gt eta12, parameter_gt eta21, parameter_gt eta22,
                  parameter_gt rho, parameter_gt lambda, parameter_gt phi)
{
    // 强制转换为定点控制域
    smc->eta11 = float2ctrl(eta11);
    smc->eta12 = float2ctrl(eta12);
    smc->eta21 = float2ctrl(eta21);
    smc->eta22 = float2ctrl(eta22);
    smc->rho = float2ctrl(rho);
    smc->lambda = float2ctrl(lambda);

    // 计算并存储边界层的倒数，以避免在 ISR 中执行除法
    if (phi < 1e-6)
    {
        phi = 1e-6; // 防除零与极度高频保护
    }
    smc->inv_phi = float2ctrl(1.0 / phi);

    ctl_clear_smc(smc);
}
