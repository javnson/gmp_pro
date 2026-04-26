/**
 * @file datalink.h
 * @brief GMP Data Link Layer - Zero-Callback, Event-Driven Async Protocol.
 * * @details
 * ============================================================================
 * HOW TO USE THIS MODULE
 * ============================================================================
 * * 1. Initialization & RX Interrupt Context:
 * - Call gmp_dev_dl_init() during system setup.
 * - Inside your UART/Bus RX ISR, call gmp_dev_dl_push_byte() or gmp_dev_dl_push_str() 
 * to feed raw data into the decoupling FIFO.
 * * 2. Main Loop Dispatcher:
 * - Periodically call gmp_dev_dl_loop_cb() in your main loop or task.
 * - Switch on the returned gmp_dl_event_t to handle RX data or execute TX hardware sending.
 * * 3. Master Mode / Active Transmission (3-Step Builder Pattern):
 * To actively send a frame, you must follow the creation, payload, and warp stages:
 * - Step 1 (Create): gmp_dev_dl_tx_request_cmd() to lock TX and set Seq/CMD.
 * - Step 2 (Payload): gmp_dev_dl_tx_append_payload(), or use helpers like 
 * gmp_dev_dl_tx_append_u16() to push data safely.
 * - Step 3 (Seal): gmp_dev_dl_tx_ready(). 
 * The loop_cb will automatically warp the frame and return GMP_DL_EVENT_TX_RDY.
 * * 4. Slave Mode / Reactive Transmission:
 * When loop_cb returns GMP_DL_EVENT_RX_OK, process `ctx->payload_buf`.
 * You MUST close the transaction by calling ONE of the following:
 * - gmp_dev_dl_reply_ack(): Opens a builder with mirrored Seq/CMD for you to append data.
 * - gmp_dev_dl_reply_ack_null(): Instantly replies with an empty ACK frame.
 * - gmp_dev_dl_reply_nack(): Instantly replies with an error code payload.
 * * ============================================================================
 * HARDWARE TRANSMISSION EXAMPLES (Handling GMP_DL_EVENT_TX_RDY)
 * ============================================================================
 * * Example A: Single-Stage TX (Polling / Blocking)
 * @code
 * if (event == GMP_DL_EVENT_TX_RDY) {
 * size_gt hdr_len, pld_len;
 * const data_gt* hdr = gmp_dev_dl_get_tx_hw_hdr(&ctx, &hdr_len);
 * const data_gt* pld = gmp_dev_dl_get_tx_hw_pld(&ctx, &pld_len);
 * * HW_UART_Send(hdr, hdr_len);
 * if (pld_len > 0) HW_UART_Send(pld, pld_len);
 * * gmp_dev_dl_tx_state_done(&ctx); // Release TX channel
 * }
 * @endcode
 * * Example B: Two-Stage TX (DMA / Interrupt Driven)
 * @code
 * // 1. In Main Loop:
 * if (event == GMP_DL_EVENT_TX_RDY) {
 * size_gt hdr_len;
 * const data_gt* hdr = gmp_dev_dl_get_tx_hw_hdr(&ctx, &hdr_len);
 * gmp_dev_dl_tx_state_next(&ctx); // Shift to PENDING_HW_HDR
 * HW_DMA_Start(hdr, hdr_len);     // Trigger DMA for Header
 * }
 * * // 2. In DMA TX Complete ISR:
 * void DMA_ISR(void) {
 * gmp_dl_tx_state_t next_state = gmp_dev_dl_tx_state_next(&ctx);
 * * if (next_state == GMP_DL_TX_STATE_PENDING_HW_PLD) {
 * size_gt pld_len;
 * const data_gt* pld = gmp_dev_dl_get_tx_hw_pld(&ctx, &pld_len);
 * HW_DMA_Start(pld, pld_len); // Trigger DMA for Payload
 * }
 * // If next_state is IDLE, transmission is fully complete!
 * }
 * @endcode
 */

#ifndef _FILE_GMP_DATALINK_H
#define _FILE_GMP_DATALINK_H

// =========================================================
// 1. Configurable Macros
// =========================================================
#ifndef GMP_DL_MTU
#define GMP_DL_MTU 256 ///< Maximum Payload Size (Bytes/Words) allowed in a single frame.
#endif

#ifndef GMP_DL_RX_FIFO_SIZE
#define GMP_DL_RX_FIFO_SIZE                                                                                            \
    256 ///< Internal ISR decoupling FIFO size. Must be sufficient to buffer data between loop executions.
#endif

#ifndef GMP_DL_OVERTIME
#define GMP_DL_OVERTIME 50 ///< Watchdog timeout threshold in milliseconds to reset a stuck RX FSM.
#endif

// =========================================================
// 2. Protocol Framing & System Commands
// =========================================================
#define GMP_DL_SOF      '{'  ///< Start of Frame boundary character (0x7B)
#define GMP_DL_EOF      '}'  ///< End of Frame boundary character (0x7D)
#define GMP_DL_ESC      '%'  ///< Escape Character for header transparency (0x25)
#define GMP_DL_XOR      0x20 ///< XOR mask applied to restore escaped bytes
#define GMP_DL_HDR_SIZE 6    ///< Unescaped Header Size: Seq(1) + CMD(1) + LEN(2) + H_CRC(2)

// Reserved System Commands
#define GMP_DL_CMD_ECHO  0x00 ///< Built-in ECHO request/response for connection testing
#define GMP_DL_CMD_NACK  0x01 ///< Negative Acknowledgment (Error Response)
#define GMP_DL_CMD_STRAY 0xFF ///< Stray/Foreign characters routing (e.g., CLI bypass)

// =========================================================
// 3. FSM & Event Enumerations
// =========================================================

/** 
 * @brief State machine enumerations for the RX parsing process. 
 */
typedef enum
{
    GMP_DL_STATE_WAIT_SYNC = 0, ///< Waiting for the SOF '{' character
    GMP_DL_STATE_HEADER_RECV,   ///< Receiving and validating the Header bytes
    GMP_DL_STATE_HEADER_ESCAPE, ///< Restoring the next byte that was escaped by '%'
    GMP_DL_STATE_PAYLOAD_RECV   ///< Blindly receiving Payload data and P_CRC without escape checks
} gmp_dl_rx_state_t;

/** 
 * @brief State machine enumerations for the TX building and sending process. 
 */
typedef enum
{
    GMP_DL_TX_STATE_IDLE = 0,       ///< TX channel is free. Ready for a new tx_request.
    GMP_DL_TX_STATE_BUILDING,       ///< Payload is being appended. Hardware sending is blocked.
    GMP_DL_TX_STATE_READY_TO_WARP,  ///< Payload construction done, waiting for loop_cb to calculate CRC and escape.
    GMP_DL_TX_STATE_PENDING_HW,     ///< Frame is physically warped and pending hardware DMA/FIFO execution.
    GMP_DL_TX_STATE_PENDING_HW_HDR, ///< Header is under transmitting, not mandatory to go through this state.
    GMP_DL_TX_STATE_PENDING_HW_PLD  ///< Payload is under transmitting, not mandatory to go through this state.
} gmp_dl_tx_state_t;

/** 
 * @brief Main loop event triggers returned by gmp_dev_dl_loop_cb(). 
 */
typedef enum
{
    GMP_DL_EVENT_IDLE = 0, ///< FSM is idle or waiting for more data. No action needed.
    GMP_DL_EVENT_RX_OK,    ///< A valid frame or stray byte was received and requires application processing.
    GMP_DL_EVENT_TX_RDY    ///< A physical TX frame is fully packed and ready for hardware transmission.
} gmp_dl_event_t;

/** 
 * @brief Decoded header information structure.
 */
typedef struct
{
    uint16_t seq_id; ///< 0-255, Frame Sequence Number for transaction tracking/retries
    uint16_t cmd;    ///< 0-255, Command ID indicating the payload's purpose
} gmp_dl_head;

// =========================================================
// 4. Context Structure
// =========================================================

/**
 * @brief The core Datalink Context maintaining all FSMs, buffers, and diagnostic data.
 */
typedef struct
{
    // --- ISR Decoupling FIFO ---
    data_gt rx_fifo[GMP_DL_RX_FIFO_SIZE]; ///< Ring buffer storing raw bytes from the hardware RX interrupt
    volatile uint16_t rx_fifo_head;       ///< Index where the next ISR byte will be written
    volatile uint16_t rx_fifo_tail;       ///< Index where the loop_cb will read the next byte

    // --- RX State Machine ---
    gmp_dl_rx_state_t rx_state; ///< Current active state of the RX Finite State Machine
    time_gt last_rx_tick;       ///< Timestamp of the last valid RX byte, used for watchdog timeouts

    uint16_t rx_hdr_idx;    ///< Current byte index during unescaped header reconstruction
    data_gt rx_hdr_buf[16]; ///< Temporary buffer for unescaped header verification

    // --- Parsed RX Frame Info ---
    gmp_dl_head rx_head;                 ///< Decoded Sequence and Command of the current incoming frame
    uint16_t expected_payload_len;       ///< Expected payload length extracted from the header
    uint16_t payload_idx;                ///< Current byte index while receiving the blind payload
    data_gt payload_buf[GMP_DL_MTU + 2]; ///< Buffer holding the pristine received payload and appended P_CRC

    // --- Transaction Control ---
    fast_gt
        flag_reply_handled; ///< Flag (1/0) indicating if the application has explicitly replied to the current RX transaction

    // --- TX State Machine & Buffers (Split for Zero-Copy Memmove avoidance) ---
    volatile gmp_dl_tx_state_t tx_state; ///< Current phase of the TX lifecycle (IDLE, BUILDING, WARPING, PENDING)

    gmp_dl_head tx_head; ///< Registered Sequence and Command for the frame currently being built

    size_gt tx_hdr_len;     ///< Length of the physically prepared header buffer
    data_gt tx_hdr_buf[16]; ///< Buffer containing the escaped physical header (Starts with SOF)

    size_gt tx_len;                 ///< Length of the physical payload buffer (Payload + P_CRC + EOF)
    data_gt tx_buf[GMP_DL_MTU + 3]; ///< Buffer containing Payload. tx_warp will append P_CRC and EOF here.

    // --- Diagnostics ---
    uint32_t err_fifo_ovf_cnt; ///< Counter for bytes lost due to ISR RX FIFO overflows
    uint32_t err_hdr_crc_cnt;  ///< Counter for frames dropped due to Header CRC failures
    uint32_t err_pld_crc_cnt;  ///< Counter for frames dropped due to Payload CRC failures
    uint32_t err_timeout_cnt;  ///< Counter for RX FSM aborts triggered by the watchdog timeout
} gmp_datalink_t;

// =========================================================
// 5. API Declarations
// =========================================================

/** * @brief Initialize the datalink context. Clears memory and resets FSMs.
 * @param ctx Pointer to the datalink context instance.
 */
void gmp_dev_dl_init(gmp_datalink_t* ctx);

/** * @brief Fast ISR handler to push a single byte into the decoupling FIFO.
 * @note  Safe to call inside high-priority interrupts. Contains no loop logic.
 * @param ctx Pointer to the datalink context.
 * @param raw_data The raw byte received from the UART/Bus hardware.
 */
void gmp_dev_dl_push_byte(gmp_datalink_t* ctx, data_gt raw_data);

/** * @brief Fast ISR handler to push a block of data into the decoupling FIFO.
 * @param ctx Pointer to the datalink context.
 * @param str Pointer to the raw data block.
 * @param size Number of bytes in the block.
 */
void gmp_dev_dl_push_str(gmp_datalink_t* ctx, const data_gt* str, size_gt size);

/** * @brief Core event dispatcher. Must be called periodically in the main loop.
 * @details Evaluates the TX FSM (priority 1) and RX FSM (priority 2).
 * @param ctx Pointer to the datalink context.
 * @return gmp_dl_event_t Actionable event requiring application intervention.
 */
gmp_dl_event_t gmp_dev_dl_loop_cb(gmp_datalink_t* ctx);

// ---------------------------------------------------------
// Active TX APIs (Master Mode - Builder Pattern)
// ---------------------------------------------------------

/**
 * @brief  Initiate a new TX frame building process. Locks the TX state.
 * @param  ctx Pointer to the datalink context.
 * @param  seq Sequence number for this transaction.
 * @param  cmd Command ID to execute.
 */
void gmp_dev_dl_tx_request_cmd(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd);

/**
 * @brief  Append data to the payload being built. Can be called multiple times.
 * @param  ctx Pointer to the datalink context.
 * @param  actual_payload_len Length of the data chunk being appended.
 * @param  data Pointer to the data chunk.
 */
void gmp_dev_dl_tx_append_payload(gmp_datalink_t* ctx, size_gt actual_payload_len, const data_gt* data);

/**
 * @brief Safely append an 8-bit unsigned integer to the TX payload.
 * @note  Applies an 0xFF mask to ensure safe truncation on 16/32-bit word architectures.
 * @param ctx Pointer to the datalink context.
 * @param val The 8-bit value to append.
 */
void gmp_dev_dl_tx_append_u8(gmp_datalink_t* ctx, data_gt val);

/**
 * @brief Safely append a 16-bit unsigned integer to the TX payload (Little-Endian).
 * @param ctx Pointer to the datalink context.
 * @param val The 16-bit value to append.
 */
void gmp_dev_dl_tx_append_u16(gmp_datalink_t* ctx, uint16_t val);

/**
 * @brief Safely append a 32-bit unsigned integer to the TX payload (Little-Endian).
 * @param ctx Pointer to the datalink context.
 * @param val The 32-bit value to append.
 */
void gmp_dev_dl_tx_append_u32(gmp_datalink_t* ctx, uint32_t val);

/**
 * @brief  Seal the payload and arm the TX state. 
 * @details In the next loop_cb, gmp_dev_dl_tx_warp() will be invoked automatically.
 * @param  ctx Pointer to the datalink context.
 */
void gmp_dev_dl_tx_ready(gmp_datalink_t* ctx);

/**
 * @brief  Convenience wrapper: Request + Append + Ready in one shot.
 * @param  ctx Pointer to the datalink context.
 * @param  seq Sequence number.
 * @param  cmd Command ID.
 * @param  actual_payload_len Length of the data.
 * @param  data Pointer to the data to be copied.
 */
void gmp_dev_dl_tx_request(gmp_datalink_t* ctx, uint16_t seq, uint16_t cmd, size_gt actual_payload_len,
                           const data_gt* data);

// ---------------------------------------------------------
// TX Buffer Exposure APIs (For Zero-Copy & Hardware Send)
// ---------------------------------------------------------

/**
 * @brief  Get the remaining capacity of the TX payload buffer.
 * @param  ctx Pointer to the datalink context.
 * @return size_gt Available bytes. Returns 0 if TX is not in BUILDING state.
 */
size_gt gmp_dev_dl_get_tx_capacity(gmp_datalink_t* ctx);

/**
 * @brief  Get a direct pointer to the current append position in the TX payload buffer.
 * @note   Useful for direct snprintf() writing to avoid local stack arrays.
 * @param  ctx Pointer to the datalink context.
 * @return data_gt* Pointer to the buffer. NULL if TX is not in BUILDING state.
 */
data_gt* gmp_dev_dl_get_tx_payload_ptr(gmp_datalink_t* ctx);

/**
 * @brief  Retrieve the physical Header buffer for hardware transmission.
 * @note   Only valid when loop_cb returns GMP_DL_EVENT_TX_RDY.
 * @param  ctx Pointer to the datalink context.
 * @param  out_len Pointer to store the resulting physical header length.
 * @return data_gt* Pointer to the header buffer (Starts with SOF).
 */
const data_gt* gmp_dev_dl_get_tx_hw_hdr(gmp_datalink_t* ctx, size_gt* out_len);

/**
 * @brief  Retrieve the physical Payload buffer for hardware transmission.
 * @note   Only valid when loop_cb returns GMP_DL_EVENT_TX_RDY.
 * @param  ctx Pointer to the datalink context.
 * @param  out_len Pointer to store the resulting physical payload length (Payload + P_CRC + EOF).
 * @return data_gt* Pointer to the payload buffer.
 */
const data_gt* gmp_dev_dl_get_tx_hw_pld(gmp_datalink_t* ctx, size_gt* out_len);

// ---------------------------------------------------------
// Reactive TX APIs (Slave Mode - Auto-Routing)
// ---------------------------------------------------------

/** 
 * @brief  Acknowledge an RX frame successfully. Sets flag_reply_handled.
 * User may append more information, and call @ref gmp_dev_dl_tx_ready to start transmition.
 * @details Automatically targets the current rx_head.seq_id and rx_head.cmd.
 * @param  ctx Pointer to the datalink context.
 */
void gmp_dev_dl_reply_ack(gmp_datalink_t* ctx);

/** 
 * @brief  Instantly reply to a received frame with an empty ACK payload.
 * @details Automatically targets the current rx_head, seals the frame, and arms it. 
 * No further appending is allowed.
 * @param  ctx Pointer to the datalink context.
 */
void gmp_dev_dl_reply_ack_null(gmp_datalink_t* ctx);

/** 
 * @brief  Explicitly reject an RX frame. Sets flag_reply_handled.
 * @details Sends an error response targeting the current seq_id with CMD_NACK.
 * @param  ctx Pointer to the datalink context.
 * @param  error_code 16-bit code describing the rejection reason.
 */
void gmp_dev_dl_reply_nack(gmp_datalink_t* ctx, uint16_t error_code);

// ---------------------------------------------------------
// Hardware Transmission State Control
// ---------------------------------------------------------

/**
 * @brief Directly conclude the TX hardware transmission and release the channel.
 * @details Forces the TX state from GMP_DL_TX_STATE_PENDING_HW back to IDLE.
 * To be used when transmitting the entire frame in a single blocking/polling routine.
 * @param ctx Pointer to the datalink context.
 */
void gmp_dev_dl_tx_state_done(gmp_datalink_t* ctx);

/**
 * @brief Advance the TX hardware transmission state machine for DMA/Interrupt driven sending.
 * @details Evaluates the current state and payload length to determine the next phase:
 * - PENDING_HW -> PENDING_HW_HDR
 * - PENDING_HW_HDR -> PENDING_HW_PLD (if payload exists) or IDLE (if empty payload)
 * - PENDING_HW_PLD -> IDLE
 * @param ctx Pointer to the datalink context.
 * @return gmp_dl_tx_state_t The new state after advancement. 
 */
gmp_dl_tx_state_t gmp_dev_dl_tx_state_next(gmp_datalink_t* ctx);

// ---------------------------------------------------------
// Internal Processing (Usually not called directly by user)
// ---------------------------------------------------------

/**
 * @brief  Pack up the TX message. Appends CRCs and executes escape routines.
 * @note   This function is called automatically within gmp_dev_dl_loop_cb().
 * @param  ctx Pointer to the datalink context.
 */
void gmp_dev_dl_tx_warp(gmp_datalink_t* ctx);

#endif // _FILE_GMP_DATALINK_H
