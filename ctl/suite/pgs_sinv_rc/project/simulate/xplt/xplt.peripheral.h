/** @file xplt.peripheral.h @brief Windows/Simulink peripherals for the SINV SIL target. */
#ifndef _FILE_SINV_SIM_XPLT_PERIPHERAL_H_
#define _FILE_SINV_SIM_XPLT_PERIPHERAL_H_
#include <gmp_core.h>
#include <sdpe_pgs_sinv_rc_simulate_settings.h>
#include <ctl/component/digital_power/basic/std_sil_dp_interface.h>
#include <ctl/component/interface/adc_channel.h>
#ifdef __cplusplus
extern "C" {
#endif
extern gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
extern gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;
#ifdef __cplusplus
}
#endif
#endif
