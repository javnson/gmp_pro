
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Back stepping controller
#include <ctl/component/intrinsic/advance/back_stepping.h>

void ctl_init_backstepping(ctl_backstepping_controller_t* bc, const ctl_backstepping_init_t* init)
{
    // 1. 严格防呆保护，防止除以零 (移除 f 后缀)
    gmp_base_assert(fabs(init->K_p) > 1e-9f);

    // 2. 在物理浮点参数域计算倒数
    parameter_gt inv_kp_val = 1.0f / init->K_p;

    // 3. 固化到定点控制域
    bc->k1 = float2ctrl(init->k1);
    bc->tau_p = float2ctrl(init->tau_p);
    bc->inv_K_p = float2ctrl(inv_kp_val);

    // 4. 安全清零
    ctl_clear_backstepping(bc);
}
