/**
 * @file gmp_tunable.c
 * @brief Compile the GMP Tunable backend selected by the platform data unit.
 */

#include <gmp_type.h>
#include <core/dev/datalink/tunable.h>

static fast_gt gmp_param_tunable_facility_dispatch(
    gmp_dl_facility_t* facility, gmp_datalink_t* datalink);

#include <string.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES == 1
#include <core/dev/datalink/tunable_u8.inc>
#elif GMP_PORT_DATA_SIZE_PER_BYTES == 2
#include <core/dev/datalink/tunable_u16.inc>
#else
#error "GMP Tunable supports only 1-byte and 2-byte addressable data units"
#endif

static fast_gt gmp_param_tunable_facility_dispatch(
    gmp_dl_facility_t* facility, gmp_datalink_t* datalink)
{
    GMP_UNUSED_VAR(datalink);
    if (facility == NULL || facility->owner == NULL)
        return 0;
    return gmp_param_tunable_rx_cb((gmp_param_tunable_t*)facility->owner);
}
