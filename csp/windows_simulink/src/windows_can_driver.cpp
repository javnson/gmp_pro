/**
 * @file windows_can_driver.cpp
 * @brief Opt-in CAN hook dispatch for the Windows GMP CSP.
 */

#include <gmp_core.h>

#if defined(SPECIFY_ENABLE_GMP_CAN_SERVICE)
#include <core/dev/can/can_hook.h>

/** @brief Forward non-native Windows CAN handles to the configured hook. */
ec_gt gmp_hal_can_get_capabilities(can_halt hcan, gmp_can_capabilities_t* capabilities)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_get_capabilities)(hcan, capabilities);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(capabilities); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN configuration to the configured hook. */
ec_gt gmp_hal_can_configure(can_halt hcan, const gmp_can_config_t* config)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_configure)(hcan, config);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(config); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN start to the configured hook. */
ec_gt gmp_hal_can_start(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_start)(hcan);
#else
    GMP_UNUSED_VAR(hcan); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Forward CAN stop to the configured hook. */
ec_gt gmp_hal_can_stop(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_stop)(hcan);
#else
    GMP_UNUSED_VAR(hcan); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Submit asynchronous CAN transmission through the configured hook. */
ec_gt gmp_hal_can_submit_tx(can_halt hcan, const gmp_can_frame_t* frame, uint32_t token)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_submit_tx)(hcan, frame, token);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(frame); GMP_UNUSED_VAR(token); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Configure a CAN filter through the configured hook. */
ec_gt gmp_hal_can_set_filter(can_halt hcan, uint16_t slot, const gmp_can_hw_filter_t* filter)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_set_filter)(hcan, slot, filter);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(slot); GMP_UNUSED_VAR(filter); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Query CAN state through the configured hook. */
ec_gt gmp_hal_can_get_state(can_halt hcan, gmp_can_state_t* state)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_get_state)(hcan, state);
#else
    GMP_UNUSED_VAR(hcan); GMP_UNUSED_VAR(state); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Request CAN recovery through the configured hook. */
ec_gt gmp_hal_can_recover(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    return GMP_CAN_HOOK_SYMBOL(_hal_can_recover)(hcan);
#else
    GMP_UNUSED_VAR(hcan); return GMP_EC_NOT_IMPL;
#endif
}

/** @brief Pump asynchronous CAN work through the configured hook. */
void gmp_hal_can_service(can_halt hcan)
{
#if defined(GMP_CAN_ENABLE_HOOK)
    GMP_CAN_HOOK_SYMBOL(_hal_can_service)(hcan);
#else
    GMP_UNUSED_VAR(hcan);
#endif
}
#endif /* SPECIFY_ENABLE_GMP_CAN_SERVICE */
