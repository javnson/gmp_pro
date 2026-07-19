/**
 * @file can.h
 * @brief Static asynchronous CAN service for GMP applications.
 */

#ifndef GMP_CORE_DEV_CAN_H
#define GMP_CORE_DEV_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef GMP_CAN_MAX_DATA_BYTES
#define GMP_CAN_MAX_DATA_BYTES 64U
#endif

#define GMP_CAN_PAYLOAD_WORD_COUNT ((GMP_CAN_MAX_DATA_BYTES + 3U) / 4U)

/* Reserved application handle range for hook-owned controllers such as SPI CAN. */
#ifndef GMP_CAN_EXTENSION_HANDLE_BASE
#define GMP_CAN_EXTENSION_HANDLE_BASE 0xEC000000UL
#endif
#define GMP_CAN_EXTENSION_HANDLE(index) ((can_halt)(uintptr_t)(GMP_CAN_EXTENSION_HANDLE_BASE + (uint32_t)(index)))

#define GMP_CAN_FRAME_EXTENDED_ID (1U << 0)
#define GMP_CAN_FRAME_REMOTE      (1U << 1)
#define GMP_CAN_FRAME_FD          (1U << 2)
#define GMP_CAN_FRAME_BITRATE_SW  (1U << 3)
#define GMP_CAN_FRAME_ESI         (1U << 4)

#define GMP_CAN_CAP_CLASSIC          (1UL << 0)
#define GMP_CAN_CAP_FD               (1UL << 1)
#define GMP_CAN_CAP_BRS              (1UL << 2)
#define GMP_CAN_CAP_RTR              (1UL << 3)
#define GMP_CAN_CAP_LOOPBACK         (1UL << 4)
#define GMP_CAN_CAP_LISTEN_ONLY      (1UL << 5)
#define GMP_CAN_CAP_HW_TIMESTAMP     (1UL << 6)
#define GMP_CAN_CAP_HW_FILTER        (1UL << 7)
#define GMP_CAN_CAP_TX_EVENT         (1UL << 8)
#define GMP_CAN_CAP_AUTO_RECOVERY    (1UL << 9)
#define GMP_CAN_CAP_REQUIRES_SERVICE (1UL << 10)
#define GMP_CAN_CAP_REMOTE_TRANSPORT (1UL << 11)

/** Normalized Classic CAN or CAN FD frame. */
typedef struct
{
    uint32_t id;
    uint16_t flags;
    uint16_t length;
    uint32_t data_32[GMP_CAN_PAYLOAD_WORD_COUNT];
} gmp_can_frame_t;

/** Optional metadata captured with a received frame. */
typedef struct
{
    uint32_t timestamp;
    uint16_t filter_index;
    uint16_t flags;
} gmp_can_rx_meta_t;

/** CAN controller capabilities reported by a CSP or hook implementation. */
typedef struct
{
    uint32_t feature_flags;
    uint16_t hardware_tx_slots;
    uint16_t hardware_rx_fifos;
    uint16_t hardware_filter_slots;
    uint16_t max_data_length;
    uint16_t timestamp_width;
    uint16_t reserved;
    uint32_t nominal_bitrate_max;
    uint32_t data_bitrate_max;
} gmp_can_capabilities_t;

/** CAN operating mode. */
typedef enum
{
    GMP_CAN_MODE_NORMAL = 0,
    GMP_CAN_MODE_LISTEN_ONLY,
    GMP_CAN_MODE_LOOPBACK
} gmp_can_mode_et;

/** Portable CAN controller configuration. */
typedef struct
{
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t nominal_sample_point_permille;
    uint16_t data_sample_point_permille;
    uint16_t flags;
    gmp_can_mode_et mode;
} gmp_can_config_t;

/** Portable hardware acceptance-filter request. */
typedef struct
{
    uint32_t id;
    uint32_t id_mask;
    uint16_t flags;
    uint16_t flags_mask;
} gmp_can_hw_filter_t;

/** CAN controller state. */
typedef enum
{
    GMP_CAN_STATE_STOPPED = 0,
    GMP_CAN_STATE_ERROR_ACTIVE,
    GMP_CAN_STATE_ERROR_WARNING,
    GMP_CAN_STATE_ERROR_PASSIVE,
    GMP_CAN_STATE_BUS_OFF,
    GMP_CAN_STATE_SLEEP
} gmp_can_bus_state_et;

/** CAN state and diagnostic counters. */
typedef struct
{
    gmp_can_bus_state_et bus_state;
    uint16_t tx_error_count;
    uint16_t rx_error_count;
    uint32_t rx_overflow_count;
    uint32_t tx_drop_count;
    uint32_t bus_off_count;
    uint32_t last_error;
} gmp_can_state_t;

/** Options stored with a queued asynchronous transmission. */
typedef struct
{
    uint32_t token;
    time_gt deadline;
    uint16_t queue;
    uint16_t flags;
} gmp_can_tx_options_t;

/** Transmission completion event. */
typedef struct
{
    uint32_t token;
    ec_gt status;
    uint32_t timestamp;
} gmp_can_tx_event_t;

/** Received frame event. */
typedef struct
{
    gmp_can_frame_t frame;
    gmp_can_rx_meta_t metadata;
} gmp_can_rx_event_t;

/** CAN state transition event. */
typedef struct
{
    gmp_can_state_t state;
    uint32_t timestamp;
} gmp_can_state_event_t;

/** Internal queued transmission entry exposed for static storage allocation. */
typedef struct
{
    gmp_can_frame_t frame;
    gmp_can_tx_options_t options;
} gmp_can_tx_entry_t;

/** User-provided static storage for one CAN node. */
typedef struct
{
    gmp_can_tx_entry_t* tx_entries;
    uint16_t tx_capacity;
    gmp_can_rx_event_t* rx_events;
    uint16_t rx_capacity;
    gmp_can_tx_event_t* tx_events;
    uint16_t tx_event_capacity;
    gmp_can_state_event_t* state_events;
    uint16_t state_event_capacity;
} gmp_can_storage_t;

/** Initialize and register a CAN handle with the static queue service. */
ec_gt gmp_can_node_init(can_halt hcan, const gmp_can_storage_t* storage);

/** Query the CSP or hook capabilities for a CAN handle. */
ec_gt gmp_can_get_capabilities(can_halt hcan, gmp_can_capabilities_t* capabilities);

/** Configure a CAN controller through its CSP or hook backend. */
ec_gt gmp_can_configure(can_halt hcan, const gmp_can_config_t* config);

/** Start a configured CAN controller. */
ec_gt gmp_can_start(can_halt hcan);

/** Stop a CAN controller. */
ec_gt gmp_can_stop(can_halt hcan);

/** Configure one hardware acceptance-filter slot. */
ec_gt gmp_can_set_filter(can_halt hcan, uint16_t filter_slot, const gmp_can_hw_filter_t* filter);

/** Read the current controller state and diagnostic counters. */
ec_gt gmp_can_get_state(can_halt hcan, gmp_can_state_t* state);

/** Request recovery from a recoverable controller fault or bus-off state. */
ec_gt gmp_can_recover(can_halt hcan);

/** Queue a frame for asynchronous transmission. */
ec_gt gmp_can_send(can_halt hcan, const gmp_can_frame_t* frame, const gmp_can_tx_options_t* options);

/** Pop one received frame event. */
ec_gt gmp_can_receive(can_halt hcan, gmp_can_rx_event_t* event);

/** Pop one transmission completion event. */
ec_gt gmp_can_get_tx_event(can_halt hcan, gmp_can_tx_event_t* event);

/** Pop one controller state event. */
ec_gt gmp_can_get_state_event(can_halt hcan, gmp_can_state_event_t* event);

/** Advance a CAN backend and pump its software transmit queue. */
void gmp_can_service(can_halt hcan);

/** Copy a received frame into the foreground receive queue. */
ec_gt gmp_can_rx_commit(can_halt hcan, const gmp_can_frame_t* frame, const gmp_can_rx_meta_t* metadata);

/** ISR-safe alias for gmp_can_rx_commit. */
ec_gt gmp_can_rx_commit_isr(can_halt hcan, const gmp_can_frame_t* frame, const gmp_can_rx_meta_t* metadata);

/** Commit an asynchronous transmission completion event. */
ec_gt gmp_can_tx_complete(can_halt hcan, uint32_t token, ec_gt status, uint32_t timestamp);

/** ISR-safe alias for gmp_can_tx_complete. */
ec_gt gmp_can_tx_complete_isr(can_halt hcan, uint32_t token, ec_gt status, uint32_t timestamp);

/** Commit a controller state change event. */
ec_gt gmp_can_state_commit(can_halt hcan, const gmp_can_state_t* state, uint32_t timestamp);

/** ISR-safe alias for gmp_can_state_commit. */
ec_gt gmp_can_state_commit_isr(can_halt hcan, const gmp_can_state_t* state, uint32_t timestamp);

/** Return one logical payload octet as a 16-bit value for C28x compatibility. */
uint16_t gmp_can_frame_get_u8(const gmp_can_frame_t* frame, uint16_t offset);

/** Store one logical payload octet from a 16-bit value. */
ec_gt gmp_can_frame_set_u8(gmp_can_frame_t* frame, uint16_t offset, uint16_t value);

/** Validate identifier, flag and payload-length combinations. */
fast_gt gmp_can_frame_is_valid(const gmp_can_frame_t* frame);

/** @name Direct CSP entry points
 * The selected CSP provides these symbols without a runtime driver table.
 * @{ */
ec_gt gmp_hal_can_get_capabilities(can_halt hcan, gmp_can_capabilities_t* capabilities);
ec_gt gmp_hal_can_configure(can_halt hcan, const gmp_can_config_t* config);
ec_gt gmp_hal_can_start(can_halt hcan);
ec_gt gmp_hal_can_stop(can_halt hcan);
ec_gt gmp_hal_can_submit_tx(can_halt hcan, const gmp_can_frame_t* frame, uint32_t token);
ec_gt gmp_hal_can_set_filter(can_halt hcan, uint16_t filter_slot, const gmp_can_hw_filter_t* filter);
ec_gt gmp_hal_can_get_state(can_halt hcan, gmp_can_state_t* state);
ec_gt gmp_hal_can_recover(can_halt hcan);
void gmp_hal_can_service(can_halt hcan);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* GMP_CORE_DEV_CAN_H */
