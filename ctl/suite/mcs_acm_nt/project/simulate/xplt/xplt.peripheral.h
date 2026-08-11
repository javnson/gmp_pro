/** @file xplt.peripheral.h @brief Simulated peripheral declarations. */

#ifndef _FILE_MCS_ACIM_NT_XPLT_PERIPHERAL_H_
#define _FILE_MCS_ACIM_NT_XPLT_PERIPHERAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <gmp_core.h>
#include <sdpe_mcs_acim_nt_simulate_settings.h>
#include <ctl/component/interface/adc_ptr_channel.h>
#include <ctl/component/digital_power/basic/std_sil_dp_interface.h>

extern gmp_pc_simulink_rx_buffer_t simulink_rx_buffer;
extern gmp_pc_simulink_tx_buffer_t simulink_tx_buffer;

extern tri_ptr_adc_channel_t uuvw;
extern adc_gt uuvw_src[3];
extern tri_ptr_adc_channel_t iuvw;
extern adc_gt iuvw_src[3];
extern ptr_adc_channel_t udc;
extern adc_gt udc_src;
extern ptr_adc_channel_t idc;
extern adc_gt idc_src;

#ifdef __cplusplus
}
#endif

#endif // _FILE_MCS_ACIM_NT_XPLT_PERIPHERAL_H_

