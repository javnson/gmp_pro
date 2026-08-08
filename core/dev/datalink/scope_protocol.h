/**
 * @file scope_protocol.h
 * @brief Platform-neutral API for the GMP Data Link Scope service.
 */

#ifndef _FILE_GMP_SCOPE_PROTOCOL_H
#define _FILE_GMP_SCOPE_PROTOCOL_H

/** @brief Current wire-format version returned by Scope discovery. */
#define GMP_SCOPE_PROTOCOL_VERSION 1U

/** @brief Scope request operation codes carried by one service command. */
typedef enum
{
    GMP_SCOPE_OP_DISCOVER = 0,
    GMP_SCOPE_OP_CONFIGURE,
    GMP_SCOPE_OP_ARM,
    GMP_SCOPE_OP_STATUS,
    GMP_SCOPE_OP_READ
} gmp_scope_operation_t;

/** @brief Sample formats reported by a scope resource. */
typedef enum
{
    GMP_SCOPE_SAMPLE_RAW = 0,
    GMP_SCOPE_SAMPLE_U8,
    GMP_SCOPE_SAMPLE_I8,
    GMP_SCOPE_SAMPLE_U16,
    GMP_SCOPE_SAMPLE_I16,
    GMP_SCOPE_SAMPLE_U32,
    GMP_SCOPE_SAMPLE_I32,
    GMP_SCOPE_SAMPLE_F32,
    GMP_SCOPE_SAMPLE_F64
} gmp_scope_sample_type_t;

/** @brief Multi-channel sample layouts supported by the host viewer. */
typedef enum
{
    GMP_SCOPE_LAYOUT_LINEAR = 0,
    GMP_SCOPE_LAYOUT_SOA,
    GMP_SCOPE_LAYOUT_INTERLEAVED
} gmp_scope_layout_t;

/** @brief Capture states returned by a registered scope resource. */
typedef enum
{
    GMP_SCOPE_STATE_WAITING = 0,
    GMP_SCOPE_STATE_CAPTURING,
    GMP_SCOPE_STATE_READY
} gmp_scope_capture_state_t;

/** @brief Trigger configuration delivered to a target scope callback. */
typedef struct
{
    fast16_gt mode;                 /**< Target-defined trigger mode, conventionally 0 through 4 */
    fast16_gt channel;              /**< Zero-based trigger source channel */
    uint16_t position_permille;     /**< Samples before the trigger, in tenths of a percent */
    parameter_gt level;             /**< Trigger comparison level */
    uint32_t auto_timeout_ms;       /**< Auto-trigger timeout in milliseconds */
} gmp_scope_config_t;

/** @brief Apply a new trigger configuration to one target scope. */
typedef fast_gt (*gmp_scope_configure_cb_t)(void* user_context,
                                            const gmp_scope_config_t* config);
/** @brief Arm one target scope and clear its previous acquisition state. */
typedef fast_gt (*gmp_scope_arm_cb_t)(void* user_context);
/** @brief Return capture state and generation for one target scope. */
typedef gmp_scope_capture_state_t (*gmp_scope_status_cb_t)(
    void* user_context, uint32_t* generation);

/** @brief One discoverable waveform buffer and its target control callbacks. */
typedef struct
{
    const char* name;                     /**< Stable English display name */
    const void* buffer;                   /**< Native base pointer of the sample buffer */
    uint32_t byte_length;                 /**< Buffer capacity expressed in protocol bytes */
    gmp_scope_sample_type_t sample_type;  /**< Numeric type of each sample */
    gmp_scope_layout_t layout;            /**< Channel arrangement in the buffer */
    uint16_t channels;                    /**< Number of waveform channels */
    uint32_t depth;                       /**< Samples stored per channel */
    uint32_t sample_rate_hz;              /**< Effective sample rate per channel */
    gmp_scope_configure_cb_t configure;   /**< Optional configuration callback */
    gmp_scope_arm_cb_t arm;               /**< Optional arm callback */
    gmp_scope_status_cb_t get_status;     /**< Optional state callback */
    void* user_context;                   /**< Opaque context forwarded to callbacks */
} gmp_scope_resource_t;

/** @brief Data Link binding and resource table for one Scope service. */
typedef struct
{
    gmp_datalink_t* dl_ctx;                 /**< Bound Data Link instance */
    uint16_t base_cmd;                      /**< Single command owned by the service */
    const gmp_scope_resource_t* resources;  /**< Registered scope resource array */
    fast16_gt resource_count;               /**< Number of registered resources */
} gmp_scope_service_t;

/**
 * @brief Initialize one Data Link Scope service.
 * @param ctx Scope service to initialize.
 * @param dl Data Link instance used for requests and responses.
 * @param base_cmd Single command identifier assigned to the service.
 * @param resources Discoverable scope resource array.
 * @param resource_count Number of entries in @p resources, limited to 255.
 */
void gmp_scope_init(gmp_scope_service_t* ctx, gmp_datalink_t* dl,
                    uint16_t base_cmd, const gmp_scope_resource_t* resources,
                    fast16_gt resource_count);

/**
 * @brief Handle a received Scope request when its command matches this service.
 * @param ctx Scope service to dispatch.
 * @return Nonzero when the current request was claimed and answered.
 */
fast_gt gmp_scope_rx_cb(gmp_scope_service_t* ctx);

#endif // _FILE_GMP_SCOPE_PROTOCOL_H
