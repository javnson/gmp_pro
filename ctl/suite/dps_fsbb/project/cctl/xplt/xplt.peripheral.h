#ifndef DPS_FSBB_CCTL_PERIPHERAL_H
#define DPS_FSBB_CCTL_PERIPHERAL_H

#include <gmp_core.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { FSBB_ADC_VIN, FSBB_ADC_VOUT, FSBB_ADC_IL, FSBB_ADC_IOUT, FSBB_ADC_COUNT };
extern adc_gt fsbb_adc_result[FSBB_ADC_COUNT];
extern pwm_gt fsbb_buck_compare;
extern pwm_gt fsbb_boost_compare;
extern fast_gt fsbb_enable_pending;
void fsbb_adc_interrupt(void);

#ifdef __cplusplus
}
#endif

#endif
