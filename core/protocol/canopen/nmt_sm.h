/** @file nmt_sm.h @brief CANopen NMT state and heartbeat services. */

#ifndef _FILE_GMP_CANOPEN_NMT_SM_H_
#define _FILE_GMP_CANOPEN_NMT_SM_H_

#include <core/protocol/canopen/can_if.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    GMP_CANOPEN_NMT_INITIALIZATION = 0,
    GMP_CANOPEN_NMT_STOPPED = 4,
    GMP_CANOPEN_NMT_OPERATIONAL = 5,
    GMP_CANOPEN_NMT_PRE_OPERATIONAL = 127
} gmp_canopen_nmt_state_t;

typedef enum
{
    GMP_CANOPEN_NMT_CMD_START = 1,
    GMP_CANOPEN_NMT_CMD_STOP = 2,
    GMP_CANOPEN_NMT_CMD_PRE_OPERATIONAL = 128,
    GMP_CANOPEN_NMT_CMD_RESET_NODE = 129,
    GMP_CANOPEN_NMT_CMD_RESET_COMMUNICATION = 130
} gmp_canopen_nmt_command_t;

typedef enum
{
    GMP_CANOPEN_NMT_EVENT_NONE = 0,
    GMP_CANOPEN_NMT_EVENT_STATE_CHANGED,
    GMP_CANOPEN_NMT_EVENT_RESET_NODE,
    GMP_CANOPEN_NMT_EVENT_RESET_COMMUNICATION
} gmp_canopen_nmt_event_t;

typedef struct
{
    uint16_t node_id;
    volatile gmp_canopen_nmt_state_t state;
    uint32_t heartbeat_period_ms;
    uint32_t heartbeat_elapsed_ms;
    fast_gt bootup_pending;
} gmp_canopen_nmt_t;

typedef struct
{
    uint16_t node_id;
    volatile gmp_canopen_nmt_state_t state;
    uint32_t timeout_ms;
    uint32_t elapsed_ms;
    fast_gt has_heartbeat;
    fast_gt timed_out;
} gmp_canopen_heartbeat_consumer_t;

fast_gt gmp_canopen_nmt_init(gmp_canopen_nmt_t* nmt, uint16_t node_id,
                             uint32_t heartbeat_period_ms);
gmp_canopen_nmt_event_t gmp_canopen_nmt_apply_command(
    gmp_canopen_nmt_t* nmt, gmp_canopen_nmt_command_t command);
gmp_canopen_nmt_event_t gmp_canopen_nmt_receive(
    gmp_canopen_nmt_t* nmt, const gmp_canopen_frame_t* frame);
fast_gt gmp_canopen_nmt_tick(gmp_canopen_nmt_t* nmt, uint32_t elapsed_ms,
                             gmp_canopen_frame_t* heartbeat);
fast_gt gmp_canopen_nmt_build_heartbeat(const gmp_canopen_nmt_t* nmt,
                                        gmp_canopen_frame_t* heartbeat);

fast_gt gmp_canopen_heartbeat_consumer_init(
    gmp_canopen_heartbeat_consumer_t* consumer, uint16_t node_id,
    uint32_t timeout_ms);
fast_gt gmp_canopen_heartbeat_consumer_receive(
    gmp_canopen_heartbeat_consumer_t* consumer,
    const gmp_canopen_frame_t* heartbeat);
fast_gt gmp_canopen_heartbeat_consumer_tick(
    gmp_canopen_heartbeat_consumer_t* consumer, uint32_t elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* _FILE_GMP_CANOPEN_NMT_SM_H_ */
