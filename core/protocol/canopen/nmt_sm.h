/**
 * @file nmt_sm.h
 * @brief CANopen NMT state, heartbeat producer and heartbeat consumer services.
 */

#ifndef _FILE_GMP_CANOPEN_NMT_SM_H_
#define _FILE_GMP_CANOPEN_NMT_SM_H_

#include <core/protocol/canopen/can_if.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief CiA 301 NMT state values transmitted in heartbeat frames. */
typedef enum
{
    GMP_CANOPEN_NMT_INITIALIZATION = 0, /**< Initialization/boot-up value. */
    GMP_CANOPEN_NMT_STOPPED = 4, /**< Stopped state. */
    GMP_CANOPEN_NMT_OPERATIONAL = 5, /**< Operational state. */
    GMP_CANOPEN_NMT_PRE_OPERATIONAL = 127 /**< Pre-operational state. */
} gmp_canopen_nmt_state_t;

/** @brief Supported NMT command specifiers. */
typedef enum
{
    GMP_CANOPEN_NMT_CMD_START = 1, /**< Enter Operational. */
    GMP_CANOPEN_NMT_CMD_STOP = 2, /**< Enter Stopped. */
    GMP_CANOPEN_NMT_CMD_PRE_OPERATIONAL = 128, /**< Enter Pre-operational. */
    GMP_CANOPEN_NMT_CMD_RESET_NODE = 129, /**< Request full node reset. */
    GMP_CANOPEN_NMT_CMD_RESET_COMMUNICATION = 130 /**< Request communication reset. */
} gmp_canopen_nmt_command_t;

/** @brief Event returned to the application after applying an NMT command. */
typedef enum
{
    GMP_CANOPEN_NMT_EVENT_NONE = 0, /**< Frame ignored or command unsupported. */
    GMP_CANOPEN_NMT_EVENT_STATE_CHANGED, /**< State changed without reset. */
    GMP_CANOPEN_NMT_EVENT_RESET_NODE, /**< Application must reset the node. */
    GMP_CANOPEN_NMT_EVENT_RESET_COMMUNICATION /**< Reset communication services. */
} gmp_canopen_nmt_event_t;

/** @brief State and timing context for one CANopen node. */
typedef struct
{
    uint16_t node_id; /**< Node ID in 1..127. */
    volatile gmp_canopen_nmt_state_t state; /**< Current externally observable state. */
    uint32_t heartbeat_period_ms; /**< Zero disables periodic production. */
    uint32_t heartbeat_elapsed_ms; /**< Accumulated producer time. */
    fast_gt bootup_pending; /**< Non-zero until the Boot-up frame is emitted. */
} gmp_canopen_nmt_t;

/** @brief Watchdog state for one remote heartbeat producer. */
typedef struct
{
    uint16_t node_id; /**< Remote node ID. */
    volatile gmp_canopen_nmt_state_t state; /**< Last valid reported state. */
    uint32_t timeout_ms; /**< Consumer timeout. */
    uint32_t elapsed_ms; /**< Time since the last valid heartbeat. */
    fast_gt has_heartbeat; /**< At least one valid heartbeat was received. */
    fast_gt timed_out; /**< Timeout latch, cleared by a valid heartbeat. */
} gmp_canopen_heartbeat_consumer_t;

/**
 * @brief Initialize one NMT node and schedule its Boot-up frame.
 * @param nmt Context to initialize.
 * @param node_id Node ID in 1..127.
 * @param heartbeat_period_ms Producer period; zero disables periodic heartbeat.
 * @return Non-zero for valid arguments.
 */
fast_gt gmp_canopen_nmt_init(gmp_canopen_nmt_t* nmt, uint16_t node_id,
                             uint32_t heartbeat_period_ms);
/**
 * @brief Apply one already-decoded NMT command.
 * @param nmt Node context.
 * @param command Command to apply.
 * @return State/reset event for the application.
 */
gmp_canopen_nmt_event_t gmp_canopen_nmt_apply_command(
    gmp_canopen_nmt_t* nmt, gmp_canopen_nmt_command_t command);
/**
 * @brief Validate and consume a broadcast or node-addressed NMT frame.
 * @param nmt Node context.
 * @param frame Candidate NMT frame.
 * @return State/reset event, or `GMP_CANOPEN_NMT_EVENT_NONE` when ignored.
 */
gmp_canopen_nmt_event_t gmp_canopen_nmt_receive(
    gmp_canopen_nmt_t* nmt, const gmp_canopen_frame_t* frame);
/**
 * @brief Advance producer time and emit Boot-up/heartbeat when due.
 * @param nmt Node context.
 * @param elapsed_ms Time elapsed since the prior call.
 * @param heartbeat Output frame when due.
 * @return Non-zero only when `heartbeat` contains a frame to send.
 */
fast_gt gmp_canopen_nmt_tick(gmp_canopen_nmt_t* nmt, uint32_t elapsed_ms,
                             gmp_canopen_frame_t* heartbeat);
/**
 * @brief Build a heartbeat frame for the current state without advancing time.
 * @param nmt Node context.
 * @param heartbeat Output frame.
 * @return Non-zero for valid arguments.
 */
fast_gt gmp_canopen_nmt_build_heartbeat(const gmp_canopen_nmt_t* nmt,
                                        gmp_canopen_frame_t* heartbeat);

/**
 * @brief Initialize a remote-node heartbeat watchdog.
 * @param consumer Consumer context.
 * @param node_id Remote node ID in 1..127.
 * @param timeout_ms Non-zero watchdog timeout.
 * @return Non-zero for valid arguments.
 */
fast_gt gmp_canopen_heartbeat_consumer_init(
    gmp_canopen_heartbeat_consumer_t* consumer, uint16_t node_id,
    uint32_t timeout_ms);
/**
 * @brief Validate and consume one heartbeat for the configured remote node.
 * @param consumer Consumer context.
 * @param heartbeat Candidate heartbeat frame.
 * @return Non-zero when the frame was accepted.
 */
fast_gt gmp_canopen_heartbeat_consumer_receive(
    gmp_canopen_heartbeat_consumer_t* consumer,
    const gmp_canopen_frame_t* heartbeat);
/**
 * @brief Advance watchdog time and return the timeout latch.
 * @param consumer Consumer context.
 * @param elapsed_ms Time elapsed since the prior call.
 * @return Current timeout latch.
 */
fast_gt gmp_canopen_heartbeat_consumer_tick(
    gmp_canopen_heartbeat_consumer_t* consumer, uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_NMT_SM_H_ */
