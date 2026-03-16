#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// HCC regular

#include <ctl/component/intrinsic/basic/hysteresis_controller.h>

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
