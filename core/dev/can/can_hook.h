/**
 * @file can_hook.h
 * @brief Single-prefix extension hook contract for GMP CAN CSP drivers.
 */

#ifndef GMP_CORE_DEV_CAN_HOOK_H
#define GMP_CORE_DEV_CAN_HOOK_H

#include <core/dev/can/can.h>

#define GMP_CAN_PP_CAT_IMPL(a, b) a##b
#define GMP_CAN_PP_CAT(a, b) GMP_CAN_PP_CAT_IMPL(a, b)

#ifdef GMP_CAN_ENABLE_HOOK
#ifndef GMP_CAN_HOOK_PREFIX
#error "GMP_CAN_HOOK_PREFIX must be defined when GMP_CAN_ENABLE_HOOK is enabled."
#endif

#define GMP_CAN_HOOK_SYMBOL(suffix) GMP_CAN_PP_CAT(GMP_CAN_HOOK_PREFIX, suffix)

#ifdef __cplusplus
extern "C"
{
#endif

ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_get_capabilities)(can_halt hcan, gmp_can_capabilities_t* capabilities);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_configure)(can_halt hcan, const gmp_can_config_t* config);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_start)(can_halt hcan);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_stop)(can_halt hcan);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_submit_tx)(can_halt hcan, const gmp_can_frame_t* frame, uint32_t token);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_set_filter)(can_halt hcan, uint16_t filter_slot, const gmp_can_hw_filter_t* filter);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_get_state)(can_halt hcan, gmp_can_state_t* state);
ec_gt GMP_CAN_HOOK_SYMBOL(_hal_can_recover)(can_halt hcan);
void GMP_CAN_HOOK_SYMBOL(_hal_can_service)(can_halt hcan);

#ifdef __cplusplus
}
#endif
#endif /* GMP_CAN_ENABLE_HOOK */

#endif /* GMP_CORE_DEV_CAN_HOOK_H */
