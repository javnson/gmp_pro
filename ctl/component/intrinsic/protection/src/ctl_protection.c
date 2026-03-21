
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Protection
#include <ctl/component/intrinsic/protection/protection.h>

void ctl_init_protection_monitor(ctl_protection_monitor_t* mon, const ctl_protection_item_t* item_set,
                                 uint32_t num_items)
{
    mon->item_set = item_set;
    mon->num_items = num_items;
    mon->fault_index = -1; // Initialize with no fault
}
