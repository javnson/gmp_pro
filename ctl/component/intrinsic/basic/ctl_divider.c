
#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// Divider

#include <ctl/component/intrinsic/basic/divider.h>

void ctl_init_divider(ctl_divider_t* obj, uint32_t counter_period)
{
    // Current counter
    obj->counter = 0;

    obj->target = counter_period;
}
