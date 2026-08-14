#ifndef GMP_MCS_PMSM_NT_F29H85X_XPLT_PERIPHERAL_H
#define GMP_MCS_PMSM_NT_F29H85X_XPLT_PERIPHERAL_H

#include <gmp_core.h>
#include <sdpe_mgr/ctrl_settings.h>
#include <ctl/component/interface/adc_ptr_channel.h>
#include <core/dev/datalink/datalink.h>

extern tri_ptr_adc_channel_t uuvw;
extern adc_gt uuvw_src[3];
extern tri_ptr_adc_channel_t iuvw;
extern adc_gt iuvw_src[3];
extern ptr_adc_channel_t udc;
extern adc_gt udc_src;
extern ptr_adc_channel_t idc;
extern adc_gt idc_src;

#endif /* GMP_MCS_PMSM_NT_F29H85X_XPLT_PERIPHERAL_H */

