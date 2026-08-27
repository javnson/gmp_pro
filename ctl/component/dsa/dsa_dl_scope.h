/**
 * @file dsa_dl_scope.h
 * @brief Packages DSA trigger/capture state as a GMP Data Link Scope facility.
 */

#ifndef _FILE_CTL_DSA_DL_SCOPE_H_
#define _FILE_CTL_DSA_DL_SCOPE_H_

#include <ctl/math_block/gmp_math.h>
#include <core/dev/datalink/scope.h>
#include <ctl/component/dsa/dsa_scope.h>
#include <ctl/component/dsa/dsa_trigger.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Number of channels exported by every suite hardware scope. */
#ifndef CTL_DSA_DL_SCOPE_CHANNELS
#define CTL_DSA_DL_SCOPE_CHANNELS 4U
#endif // CTL_DSA_DL_SCOPE_CHANNELS

/** @brief Default capture depth when a project does not provide one. */
#ifndef GMP_DL_SCOPE_DEPTH
#define GMP_DL_SCOPE_DEPTH 100U
#endif // GMP_DL_SCOPE_DEPTH

/** @brief Default command allocated to the independent Scope service. */
#define CTL_DSA_DL_SCOPE_DEFAULT_CMD 0x60U

/** @brief Workspace elements required for published and pre-trigger storage. */
#define CTL_DSA_DL_SCOPE_STORAGE_ELEMENTS(channels, depth) \
    (2UL * (uint32_t)(channels) * (uint32_t)(depth))

/** @brief Acquisition state and storage owned by one Data Link Scope. */
typedef struct
{
    gmp_scope_service_t service;       /**< Data Link command service. */
    gmp_scope_resource_t resource;     /**< Discoverable scope resource. */
    ctl_dsa_trigger_t trigger;         /**< Trigger detector. */
    ctl_dsa_scope_t recorder;          /**< One-to-four-channel post-trigger recorder. */
    ctrl_gt* buffer;                   /**< Published structure-of-arrays buffer. */
    ctrl_gt* history;                  /**< Circular pre-trigger history buffer. */
    uint32_t depth;                    /**< Samples stored per channel. */
    uint32_t sample_rate_hz;           /**< Undivided control sampling rate. */
    volatile uint32_t generation;      /**< Completed-capture generation counter. */
    volatile uint32_t history_write;   /**< Next circular-history write index. */
    volatile uint32_t history_count;   /**< Valid samples in circular history. */
    volatile uint16_t position_permille; /**< Requested pre-trigger position. */
    volatile fast16_gt trigger_channel;  /**< Zero-based trigger source. */
    volatile ctl_dsa_trigger_option_t trigger_mode; /**< Active trigger mode. */
    volatile uint32_t auto_timeout_ms; /**< Auto-trigger timeout. */
    volatile uint16_t sample_divider;  /**< Sampling divider; zero samples every tick. */
    volatile uint16_t sample_divider_counter; /**< Control ticks skipped before sampling. */
    volatile parameter_gt trigger_level; /**< Trigger comparison level. */
    volatile gmp_scope_capture_state_t state; /**< Current acquisition state. */
} ctl_dsa_dl_scope_t;

/**
 * @brief Initialize a fixed four-channel DSA Data Link Scope.
 * @param scope Scope object to initialize.
 * @param dl Data Link object that owns the Scope command.
 * @param command Scope service command, normally 0x60.
 * @param name Stable English resource name; NULL selects "Control Scope".
 * @param buffer Published sample buffer with four times @p depth elements.
 * @param history Pre-trigger history buffer with four times @p depth elements.
 * @param depth Number of samples per channel; must be nonzero.
 * @param sample_rate_hz Sampling frequency in hertz; must be nonzero.
 */
void ctl_init_dsa_dl_scope(ctl_dsa_dl_scope_t* scope, gmp_datalink_t* dl,
                           uint16_t command, const char* name,
                           ctrl_gt* buffer, ctrl_gt* history,
                           uint32_t depth, uint32_t sample_rate_hz);

/**
 * @brief Initialize a Scope from one user-owned workspace.
 * @details The first half becomes the published capture and the second half
 *          becomes private pre-trigger history. Initial trigger settings are
 *          rising edge, channel zero, level zero, 50% trigger position.
 * @return Nonzero when the workspace and configuration are valid.
 */
fast_gt ctl_init_dsa_dl_scope_workspace(
    ctl_dsa_dl_scope_t* scope, gmp_datalink_t* dl, uint16_t command,
    const char* name, ctrl_gt* workspace, uint32_t workspace_elements,
    uint16_t channels, uint32_t sample_rate_hz);

/** @brief Return the service descriptor appended to a Data Link. */
GMP_STATIC_INLINE gmp_dl_facility_t* ctl_dsa_dl_scope_facility(
    ctl_dsa_dl_scope_t* scope)
{
    return (scope == NULL) ? NULL : &scope->service.facility;
}

/** @brief Process one array containing one through four channel samples. */
void ctl_step_dsa_dl_scope(ctl_dsa_dl_scope_t* scope,
                           const ctrl_gt* channels, uint16_t channel_count);

/** @brief Process one two-channel sample tuple. */
void ctl_step_dsa_dl_scope_2ch(ctl_dsa_dl_scope_t* scope,
                               ctrl_gt channel_0, ctrl_gt channel_1);

/**
 * @brief Process one four-channel sample in the real-time interrupt.
 * @param scope Initialized scope object.
 * @param channel_0 First channel and default trigger source.
 * @param channel_1 Second channel.
 * @param channel_2 Third channel.
 * @param channel_3 Fourth channel.
 */
void ctl_step_dsa_dl_scope_4ch(ctl_dsa_dl_scope_t* scope,
                               ctrl_gt channel_0, ctrl_gt channel_1,
                               ctrl_gt channel_2, ctrl_gt channel_3);

/**
 * @brief Dispatch the current Data Link packet to the Scope service.
 * @param scope Initialized scope object.
 * @return Nonzero when the Scope service handled the packet.
 */
fast_gt ctl_dsa_dl_scope_rx_cb(ctl_dsa_dl_scope_t* scope);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_CTL_DSA_DL_SCOPE_H_ */
