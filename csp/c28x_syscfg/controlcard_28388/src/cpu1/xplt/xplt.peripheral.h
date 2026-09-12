#ifndef GMP_F28388D_CPU1_XPLT_PERIPHERAL_H
#define GMP_F28388D_CPU1_XPLT_PERIPHERAL_H

#include <core/dev/datalink/datalink.h>
#include "tricore_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile gmp_wave_command_t cpu1_to_cpu2_command;
extern volatile gmp_wave_snapshot_t cpu2_to_cpu1_snapshot;

void xplt_cpu1_dl_receive(gmp_datalink_t *datalink);
fast_gt xplt_cpu1_dl_send(gmp_datalink_t *datalink);
void xplt_cpu1_toggle_status_led(void);

#ifdef __cplusplus
}
#endif

#endif
