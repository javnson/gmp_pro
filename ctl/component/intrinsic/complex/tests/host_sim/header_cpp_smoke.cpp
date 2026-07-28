#include <gmp_core.h>

#include <ctl/component/intrinsic/complex/dq_ladrc1.h>
#include <ctl/component/intrinsic/complex/dq_pi.h>

int main()
{
    ctl_dq_pi_t pi = {};
    ctl_dq_ladrc1_t ladrc = {};
    return (pi.flag_enable_feedforward != 0) || (ladrc.flag_enable_feedforward != 0);
}
