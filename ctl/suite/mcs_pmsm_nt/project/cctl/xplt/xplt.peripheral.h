/**
 * @file xplt.peripheral.h
 * @brief C-visible register storage for the simulated MCU peripherals.
 */
#ifndef MCS_PMSM_NT_CCTL_XPLT_PERIPHERAL_H
#define MCS_PMSM_NT_CCTL_XPLT_PERIPHERAL_H

#include <gmp_type.h>
#include <ctrl_settings.h>
#include <ctl/component/interface/adc_ptr_channel.h>

#ifdef __cplusplus
extern "C"
{
#endif

enum cctl_adc_index
{
    CCTL_ADC_UDC = 0,
    CCTL_ADC_UA,
    CCTL_ADC_UB,
    CCTL_ADC_UC,
    CCTL_ADC_IA,
    CCTL_ADC_IB,
    CCTL_ADC_IC,
    CCTL_ADC_COUNT
};

extern tri_ptr_adc_channel_t uuvw;
extern adc_gt uuvw_src[3];
extern tri_ptr_adc_channel_t iuvw;
extern adc_gt iuvw_src[3];
extern ptr_adc_channel_t udc;
extern adc_gt udc_src;
extern ptr_adc_channel_t idc;
extern adc_gt idc_src;

extern adc_gt cctl_adc_result[CCTL_ADC_COUNT];
extern uint32_t cctl_encoder_position;
extern pwm_gt cctl_pwm_compare[3];
extern uint64_t cctl_controller_tick;

/** Initialize controller-facing ADC, eQEP, ePWM, and scaling storage. */
void setup_peripheral(void);

/** Advance the controller time base by one ADC interrupt. */
void cctl_advance_controller_tick(void);

/** Execute the controller work owned by one simulated ADC interrupt. */
void cctl_adc_interrupt(void);

/** @return Current hosted system tick in milliseconds. */
time_gt gmp_base_get_system_tick(void);

#ifdef __cplusplus
}
#endif

#endif // MCS_PMSM_NT_CCTL_XPLT_PERIPHERAL_H
