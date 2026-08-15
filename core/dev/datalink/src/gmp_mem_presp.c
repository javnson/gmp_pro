/**
 * @file gmp_mem_presp.c
 * @brief Compile the Memory Perspective backend selected by the platform data unit.
 */

#include <gmp_type.h>
#include <core/dev/datalink/mem_presp.h>

static fast_gt gmp_mem_persp_facility_dispatch(
    gmp_dl_facility_t* facility, gmp_datalink_t* datalink);

#include <stdint.h>
#include <string.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/mem_presp_u8.inc>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/mem_presp_u16.inc>
#else
#error "GMP Memory Perspective supports only 1-byte and 2-byte data units"
#endif

static fast_gt gmp_mem_persp_facility_dispatch(
    gmp_dl_facility_t* facility, gmp_datalink_t* datalink)
{
    GMP_UNUSED_VAR(datalink);
    if (facility == NULL || facility->owner == NULL)
        return 0;
    return gmp_mem_persp_rx_cb((gmp_mem_persp_t*)facility->owner);
}
