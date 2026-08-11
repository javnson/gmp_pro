/**
 * @file datalink_u8.h
 * @brief GMP Data Link backend for byte-addressed CPUs.
 *
 * This backend uses uint8_t directly for every protocol byte. Buffer lengths,
 * pointer increments, CRC input lengths, and hardware-transfer lengths are
 * therefore all physical byte counts, independent of data_gt.
 */

#ifndef _FILE_GMP_DATALINK_U8_H
#define _FILE_GMP_DATALINK_U8_H

#if GMP_PORT_DATA_SIZE_PER_BYTES != 1
#error "datalink_u8.h requires GMP_PORT_DATA_SIZE_PER_BYTES == 1"
#endif

#define GMP_DL_BACKEND_U8 1
#ifndef GMP_DL_MTU
#define GMP_DL_MTU 256
#endif

#ifndef GMP_DL_RX_FIFO_SIZE
#define GMP_DL_RX_FIFO_SIZE 256
#endif

#ifndef GMP_DL_OVERTIME
#define GMP_DL_OVERTIME 50
#endif

#define GMP_DL_SOF      ((uint8_t)0x7B)
#define GMP_DL_EOF      ((uint8_t)0x7D)
#define GMP_DL_ESC      ((uint8_t)0x25)
#define GMP_DL_XOR      ((uint8_t)0x20)
#define GMP_DL_HDR_SIZE 6

#define GMP_DL_CMD_ECHO  0x00
#define GMP_DL_CMD_NACK  0x01
#define GMP_DL_CMD_STRAY 0xFF

/** @brief Receive parser states. */
typedef enum
{
    GMP_DL_STATE_WAIT_SYNC = 0,
    GMP_DL_STATE_HEADER_RECV,
    GMP_DL_STATE_HEADER_ESCAPE,
    GMP_DL_STATE_PAYLOAD_RECV
} gmp_dl_rx_state_t;

/** @brief Transmit buffer ownership states. */
typedef enum
{
    GMP_DL_TX_STATE_IDLE = 0,
    GMP_DL_TX_STATE_BUILDING,
    GMP_DL_TX_STATE_READY_TO_WARP,
    GMP_DL_TX_STATE_PENDING_HW,
    GMP_DL_TX_STATE_PENDING_HW_HDR,
    GMP_DL_TX_STATE_PENDING_HW_PLD
} gmp_dl_tx_state_t;

/** @brief Events returned by gmp_dev_dl_loop_cb(). */
typedef enum
{
    GMP_DL_EVENT_IDLE = 0,
    GMP_DL_EVENT_RX_OK,
    GMP_DL_EVENT_TX_RDY
} gmp_dl_event_t;

/** @brief Logical sequence and command header fields. */
typedef struct
{
    uint16_t seq_id;
    uint16_t cmd;
} gmp_dl_head;

/** @brief Complete state and storage for one byte-addressed Data Link. */
typedef struct
{
    uint8_t rx_fifo[GMP_DL_RX_FIFO_SIZE];
    volatile uint16_t rx_fifo_head;
    volatile uint16_t rx_fifo_tail;
    volatile fast_gt rx_reset_pending;

    gmp_dl_rx_state_t rx_state;
    time_gt last_rx_tick;
    uint16_t rx_hdr_idx;
    uint8_t rx_hdr_buf[16];

    gmp_dl_head rx_head;
    uint16_t expected_payload_len;
    uint16_t payload_idx;
    uint8_t payload_buf[GMP_DL_MTU + 2];

    fast_gt flag_reply_handled;

    volatile gmp_dl_tx_state_t tx_state;
    gmp_dl_head tx_head;
    size_gt tx_hdr_len;
    uint8_t tx_hdr_buf[16];
    size_gt tx_len;
    uint8_t tx_buf[GMP_DL_MTU + 3];

    uint32_t err_fifo_ovf_cnt;
    uint32_t err_hdr_crc_cnt;
    uint32_t err_pld_crc_cnt;
    uint32_t err_timeout_cnt;
} gmp_datalink_t;

/** @brief Initialize one Data Link instance. */
void gmp_dev_dl_init(gmp_datalink_t* ctx);
/** @brief Queue one received protocol octet, typically from an ISR. */
void gmp_dev_dl_push_byte(gmp_datalink_t* ctx, uint8_t raw_data);
/** @brief Queue a sequence of received protocol octets. */
void gmp_dev_dl_push_str(gmp_datalink_t* ctx, const uint8_t* str, size_gt size);
/**
 * @brief Request asynchronous receive-state recovery.
 * @details The protocol task discards queued input and resets its parser before
 *          processing another frame. This function is safe to call from a UART
 *          error interrupt without modifying parser-owned state directly.
 */
void gmp_dev_dl_request_rx_reset(gmp_datalink_t* ctx);
/** @brief Advance the state machine and return at most one application event. */
gmp_dl_event_t gmp_dev_dl_loop_cb(gmp_datalink_t* ctx);
/** @brief Apply the default ECHO, NACK, stray, and unsupported-command policy. */
void gmp_dev_dl_default_rx_handler(gmp_datalink_t* ctx);

/** @brief Start building a response with the specified sequence and command. */
void gmp_dev_dl_tx_request_cmd(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd);
/** @brief Append raw payload octets to a response under construction. */
void gmp_dev_dl_tx_append_payload(gmp_datalink_t* ctx, const uint8_t* data,
                                  size_gt actual_payload_len);
/** @brief Append one unsigned 8-bit value. */
void gmp_dev_dl_tx_append_u8(gmp_datalink_t* ctx, uint8_t val);
/** @brief Append one little-endian unsigned 16-bit value. */
void gmp_dev_dl_tx_append_u16(gmp_datalink_t* ctx, uint16_t val);
/** @brief Append one little-endian unsigned 32-bit value. */
void gmp_dev_dl_tx_append_u32(gmp_datalink_t* ctx, uint32_t val);
/** @brief Mark a response ready for framing. */
void gmp_dev_dl_tx_ready(gmp_datalink_t* ctx);
/** @brief Build a complete response from an existing payload. */
void gmp_dev_dl_tx_request(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd,
                           size_gt actual_payload_len, const uint8_t* data);

/** @brief Return the remaining payload capacity. */
size_gt gmp_dev_dl_get_tx_capacity(gmp_datalink_t* ctx);
/** @brief Return the writable payload insertion pointer. */
uint8_t* gmp_dev_dl_get_tx_payload_ptr(gmp_datalink_t* ctx);
/** @brief Return the framed header and optionally its octet length. */
const uint8_t* gmp_dev_dl_get_tx_hw_hdr(gmp_datalink_t* ctx, size_gt* out_len);
/** @brief Return the framed payload and optionally its octet length. */
const uint8_t* gmp_dev_dl_get_tx_hw_pld(gmp_datalink_t* ctx, size_gt* out_len);

GMP_STATIC_INLINE const uint8_t* gmp_dev_dl_get_tx_hw_hdr_ptr(gmp_datalink_t* ctx)
{
    return ctx->tx_hdr_buf;
}

GMP_STATIC_INLINE size_gt gmp_dev_dl_get_tx_hw_hdr_size(gmp_datalink_t* ctx)
{
    return ctx->tx_hdr_len;
}

GMP_STATIC_INLINE const uint8_t* gmp_dev_dl_get_tx_hw_pld_ptr(gmp_datalink_t* ctx)
{
    return ctx->tx_buf;
}

GMP_STATIC_INLINE size_gt gmp_dev_dl_get_tx_hw_pld_size(gmp_datalink_t* ctx)
{
    return ctx->tx_len;
}

/** @brief Begin an empty ACK response for the current request. */
void gmp_dev_dl_reply_ack(gmp_datalink_t* ctx);
/** @brief Send an empty ACK response for the current request. */
void gmp_dev_dl_reply_ack_null(gmp_datalink_t* ctx);
/** @brief Send a NACK response for the current request. */
void gmp_dev_dl_reply_nack(gmp_datalink_t* ctx, uint16_t error_code);
/** @brief Mark the current request as handled by the application. */
void gmp_dev_dl_msg_handled(gmp_datalink_t* ctx);

/** @brief Release transmit buffers after hardware transmission completes. */
void gmp_dev_dl_tx_state_done(gmp_datalink_t* ctx);
/** @brief Advance split hardware transmit state and return the new state. */
gmp_dl_tx_state_t gmp_dev_dl_tx_state_next(gmp_datalink_t* ctx);
/** @brief Append CRC fields and escape the transmit header. */
void gmp_dev_dl_tx_warp(gmp_datalink_t* ctx);

#endif // _FILE_GMP_DATALINK_U8_H
