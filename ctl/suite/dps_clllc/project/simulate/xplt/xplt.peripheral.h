/**
 * @file xplt.peripheral.h
 * @brief Windows/Simulink platform declarations for the FSBB SIL target.
 */
#ifndef _FILE_FSBB_SIM_XPLT_PERIPHERAL_H_
#define _FILE_FSBB_SIM_XPLT_PERIPHERAL_H_

#include <gmp_core.h>
#include <ctl/component/digital_power/basic/std_sil_dp_interface.h>
#include "ctl_main.h"

#ifdef __cplusplus
extern "C" {
#endif
extern gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
extern gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;
#ifdef __cplusplus
}
#endif
#endif
