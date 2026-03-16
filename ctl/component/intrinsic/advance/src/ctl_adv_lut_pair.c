#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
//
#include <ctl/component/intrinsic/advance/paired_lut1d.h>

void ctl_init_paired_lut1d(ctl_paired_lut1d_t* lut, const ctl_lut1d_pair_t* table, uint32_t size)
{
    if (table != NULL && size > 0)
    {
        lut->table = table;
        lut->size = size;
    }
    else
    {
        // Safe default handling for null or empty tables
        lut->table = NULL;
        lut->size = 0;
    }
}
