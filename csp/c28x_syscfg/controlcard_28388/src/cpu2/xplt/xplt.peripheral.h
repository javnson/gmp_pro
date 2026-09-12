#ifndef GMP_F28388D_CPU2_XPLT_PERIPHERAL_H
#define GMP_F28388D_CPU2_XPLT_PERIPHERAL_H

#include "tricore_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile gmp_wave_command_t cpu1_to_cpu2_command;
extern volatile gmp_wave_command_t cm_to_cpu2_command;
extern volatile gmp_wave_snapshot_t cpu2_to_cpu1_snapshot;
extern volatile gmp_wave_snapshot_t cpu2_to_cm_snapshot;

#ifdef __cplusplus
}
#endif

#endif
