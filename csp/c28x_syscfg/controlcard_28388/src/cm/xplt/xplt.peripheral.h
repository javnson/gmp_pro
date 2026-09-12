#ifndef GMP_F28388D_CM_XPLT_PERIPHERAL_H
#define GMP_F28388D_CM_XPLT_PERIPHERAL_H

#include <core/dev/datalink/datalink.h>
#include "tricore_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    XPLT_CM_DL_SEND_OK = 0,
    XPLT_CM_DL_SEND_RETRY,
    XPLT_CM_DL_SEND_DISCONNECTED,
    XPLT_CM_DL_SEND_ERROR
} xplt_cm_dl_send_result_t;

extern volatile gmp_wave_command_t cm_to_cpu2_command;
extern volatile gmp_wave_snapshot_t cpu2_to_cm_snapshot;

void xplt_cm_dl_receive(gmp_datalink_t *datalink);
xplt_cm_dl_send_result_t xplt_cm_dl_send(gmp_datalink_t *datalink);
uint32_t xplt_cm_ethercat_memory_ready(void);

#ifdef __cplusplus
}
#endif

#endif
