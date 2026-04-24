/**
 * @file gmp_tunable.h
 * @brief GMP Tunable Module - A robust lockstep and parameter tuning framework.
 * @details Implements a 3-state SLIP-like FSM for UART/DMA communication, 
 * providing safe memory access and simulation step control.
 */

#include <core/dev/ring_buf.h>

#ifndef _FILE_GMP_TUNABLE_H
#define _FILE_GMP_TUNABLE_H

// ==========================================
// 1. Protocol Boundaries & Escape Characters
// ==========================================
#define GMP_TUNE_SOF '{'  ///< 0x7B, Start of Frame
#define GMP_TUNE_EOF '}'  ///< 0x7D, End of Frame
#define GMP_TUNE_ESC '%'  ///< 0x25, Escape Character
#define GMP_TUNE_XOR 0x20 ///< 0x20, XOR mask for escape restoration

// ==========================================
// 2. Memory & Capacity Constraints
// ==========================================
#ifndef GMP_TUNABLE_MTU_SIZE
#define GMP_TUNABLE_MTU_SIZE 256 ///< Maximum payload length to prevent buffer overflow
#endif                           // GMP_TUNABLE_MTU_SIZE

#ifndef GMP_TUNABLE_TX_BUFFER_SIZE
#define GMP_TUNABLE_TX_BUFFER_SIZE 256 ///< Maximum transmit buffer length
#endif                                 // GMP_TUNABLE_TX_BUFFER_SIZE

// ==========================================
// 3. Command Pruning Macros
// ==========================================
// User may enable these macros to disable specified utilities to save ROM.
//#define TUNE_DISABLE_CMD_VAR_READ
//#define TUNE_DISABLE_CMD_VAR_WRITE
//#define TUNE_DISABLE_CMD_MEM_READ
//#define TUNE_DISABLE_CMD_MEM_WRITE
//#define TUNE_DISABLE_CMD_SIM_STEP


// ==========================================
// 4. internal Command Codes Definition
// ==========================================
#define GMP_TUNABLE_CMD_VAR_READ  0x11
#define GMP_TUNABLE_CMD_VAR_WRITE 0x12
#define GMP_TUNABLE_CMD_MEM_READ  0x21
#define GMP_TUNABLE_CMD_MEM_WRITE 0x22
#define GMP_TUNABLE_CMD_SIM_STEP  0x30

/**
 * @brief Enumeration for the RX Finite State Machine (FSM).
 */
typedef enum
{
    GMP_TUNE_STATE_WAIT_SYNC = 0, ///< Waiting for Start of Frame (SOF)
    GMP_TUNE_STATE_RECEIVE,       ///< Receiving normal payload data
    GMP_TUNE_STATE_ESCAPE         ///< Escape character received, awaiting next byte
} gmp_tune_rx_state_t;

/**
 * @brief Context handle for the GMP Tunable module.
 */
typedef struct
{
    // --- RX Path ---
    ringbuf_t* rb_rx; ///< Pointer to the RX ring buffer (Written by UART ISR, read by this module)

    // --- TX Path (DMA Friendly) ---
    data_gt tx_buffer[GMP_TUNABLE_TX_BUFFER_SIZE]; ///< Linear buffer containing the fully framed TX data
    size_gt tx_buffer_len;                         ///< Valid length of the data pending in tx_buffer
    fast_gt tx_buffer_cmpt; ///< Flag indicating if the TX operation is completed/idle (1: Idle, 0: Busy)

    // --- FSM Context ---
    gmp_tune_rx_state_t rx_state;    ///< Current state of the RX FSM
    uint16_t rx_idx;                 ///< Current index of the RX buffer
    data_gt rx_buffer[GMP_TUNABLE_MTU_SIZE]; ///< Buffer for the reconstructed, un-escaped frame

    // --- Diagnostic Counters ---
    uint32_t err_overflow_cnt; ///< Number of MTU overflow events (Missing EOF)
    uint32_t err_crc_cnt;      ///< Number of CRC verification failures
} gmp_tunable_t;

/**
 * @brief Function pointer signature for command handlers.
 * @param ctx Pointer to the tunable module context.
 * @param seq Sequence number of the received frame.
 * @param payload Pointer to the payload data array.
 * @param len Length of the payload data.
 */
typedef void (*gmp_tune_handler_t)(gmp_tunable_t* ctx, uint8_t seq, const data_gt* payload, size_gt len);

/**
 * @brief Structure for a command registry entry.
 */
typedef struct
{
    uint8_t cmd_code;           ///< Command code identifier
    gmp_tune_handler_t handler; ///< Pointer to the execution callback
} gmp_tune_cmd_entry_t;

/**
 * @brief Initializes the GMP tunable context.
 * @param ctx Pointer to the tunable module context.
 * @param rx_buf Pointer to the initialized RX ring buffer.
 */
void gmp_tunable_init(gmp_tunable_t* ctx, ringbuf_t* rx_buf);

/**
 * @brief Core polling task for the tunable module.
 * @details Extracts data from the RX ring buffer, drives the FSM, 
 * and dispatches commands upon successful frame reception.
 * Should be placed in the main loop or an RTOS task.
 * @param ctx Pointer to the tunable module context.
 */
void gmp_tunable_poll(gmp_tunable_t* ctx);

// ==========================================
// External Dependencies & Callbacks
// ==========================================

/**
 * @brief User-provided CRC16 calculation function.
 * @note Must accept data_gt* to maintain compatibility with 16-bit DSP architectures.
 */
extern uint16_t Calculate_CRC16(const data_gt* data, uint16_t len);

// User-implemented command callbacks
extern void gmp_tune_handle_mem_write(gmp_tunable_t* ctx, uint8_t seq, const data_gt* payload, uint16_t len);
extern void gmp_tune_handle_sim_step(gmp_tunable_t* ctx, uint8_t seq, const data_gt* payload, uint16_t len);

// ==========================================
// Static Command Registry
// ==========================================
static const gmp_tune_cmd_entry_t g_cmd_registry[] = {
#ifndef TUNE_DISABLE_CMD_MEM_WRITE
    {GMP_TUNABLE_CMD_MEM_WRITE, gmp_tune_handle_mem_write},
#endif
#ifndef TUNE_DISABLE_CMD_SIM_STEP
    {GMP_TUNABLE_CMD_SIM_STEP, gmp_tune_handle_sim_step},
#endif
    // Additional user commands can be appended here
};

#define CMD_REGISTRY_SIZE (sizeof(g_cmd_registry) / sizeof(g_cmd_registry[0]))

// ==========================================
// Internal Private Function Declarations
// ==========================================
static void process_complete_frame(gmp_tunable_t* ctx);
static void dispatch_command(gmp_tunable_t* ctx, uint8_t seq, uint8_t cmd, const data_gt* payload,
                             uint16_t payload_len);

// ==========================================
// API Implementations
// ==========================================

void gmp_tunable_init(gmp_tunable_t* ctx, ringbuf_t* rx_buf)
{
    if (!ctx)
        return;

    ctx->rb_rx = rx_buf;
    ctx->tx_buffer_len = 0;
    ctx->tx_buffer_cmpt = 1; // Mark as idle initially

    ctx->rx_state = GMP_TUNE_STATE_WAIT_SYNC;
    ctx->rx_idx = 0;
    ctx->err_overflow_cnt = 0;
    ctx->err_crc_cnt = 0;
}

void gmp_tunable_poll(gmp_tunable_t* ctx)
{
    data_gt raw_data;

    // Continuously extract data from the RingBuffer until empty
    while (ringbuf_get_one(ctx->rb_rx, &raw_data))
    {
        // Safe downcast to uint8_t for FSM logic processing
        uint8_t byte = (uint8_t)(raw_data & 0xFF);

        // Rule 1: SOF resets the state machine unconditionally
        if (byte == GMP_TUNE_SOF)
        {
            ctx->rx_state = GMP_TUNE_STATE_RECEIVE;
            ctx->rx_idx = 0;
            continue;
        }

        // Rule 2: EOF triggers frame finalization if in RECEIVE state
        if (byte == GMP_TUNE_EOF)
        {
            if (ctx->rx_state == GMP_TUNE_STATE_RECEIVE && ctx->rx_idx > 0)
            {
                process_complete_frame(ctx);
            }
            ctx->rx_state = GMP_TUNE_STATE_WAIT_SYNC;
            continue;
        }

        // FSM Transitions
        switch (ctx->rx_state)
        {
        case GMP_TUNE_STATE_WAIT_SYNC:
            // Discard garbage data outside of frame boundaries
            break;

        case GMP_TUNE_STATE_RECEIVE:
            if (byte == GMP_TUNE_ESC)
            {
                ctx->rx_state = GMP_TUNE_STATE_ESCAPE;
            }
            else
            {
                if (ctx->rx_idx < GMP_TUNE_MTU)
                {
                    ctx->rx_buffer[ctx->rx_idx++] = (data_gt)byte;
                }
                else
                {
                    // MTU Overflow: Dropped EOF, discard the entire packet
                    ctx->err_overflow_cnt++;
                    ctx->rx_state = GMP_TUNE_STATE_WAIT_SYNC;
                }
            }
            break;

        case GMP_TUNE_STATE_ESCAPE:
            // Restore escaped character (XOR with 0x20)
            if (ctx->rx_idx < GMP_TUNE_MTU)
            {
                ctx->rx_buffer[ctx->rx_idx++] = (data_gt)(byte ^ GMP_TUNE_XOR);
            }
            ctx->rx_state = GMP_TUNE_STATE_RECEIVE; // Revert to normal receive state
            break;
        }
    }
}

/**
 * @brief Parses a complete, un-escaped frame. Validates length and CRC.
 * @param ctx Pointer to the tunable module context.
 */
static void process_complete_frame(gmp_tunable_t* ctx)
{
    // Minimum frame length: SEQ(1) + CMD(1) + LEN(2) + CRC(2) = 6 bytes
    if (ctx->rx_idx < 6)
        return;

    // Isolate CRC (Last 2 bytes).
    // Masking with 0xFF ensures safety on 16-bit DSP architectures.
    uint16_t rx_crc = ((uint16_t)(ctx->rx_buffer[ctx->rx_idx - 2] & 0xFF) << 8) |
                      ((uint16_t)(ctx->rx_buffer[ctx->rx_idx - 1] & 0xFF));

    uint16_t data_len_for_crc = ctx->rx_idx - 2;

    // Verify CRC
    uint16_t calc_crc = Calculate_CRC16(ctx->rx_buffer, data_len_for_crc);
    if (calc_crc != rx_crc)
    {
        ctx->err_crc_cnt++;
        // Verification failed. Silently drop to let the ARQ timeout mechanism handle it.
        return;
    }

    // Extract Headers
    uint8_t seq = (uint8_t)(ctx->rx_buffer[0] & 0xFF);
    uint8_t cmd = (uint8_t)(ctx->rx_buffer[1] & 0xFF);
    uint16_t payload_len = ((uint16_t)(ctx->rx_buffer[2] & 0xFF) << 8) | ((uint16_t)(ctx->rx_buffer[3] & 0xFF));

    // Secondary Length Verification to prevent malformed payload definitions
    if (payload_len != (data_len_for_crc - 4))
        return;

    // Dispatch Command
    dispatch_command(ctx, seq, cmd, &ctx->rx_buffer[4], payload_len);
}

/**
 * @brief Iterates through the command registry and triggers the mapped callback.
 * @param ctx Pointer to the tunable module context.
 * @param seq Sequence number.
 * @param cmd Command code.
 * @param payload Pointer to the payload data.
 * @param payload_len Length of the payload data.
 */
static void dispatch_command(gmp_tunable_t* ctx, uint8_t seq, uint8_t cmd, const data_gt* payload, uint16_t payload_len)
{
    for (int i = 0; i < CMD_REGISTRY_SIZE; i++)
    {
        if (g_cmd_registry[i].cmd_code == cmd)
        {
            g_cmd_registry[i].handler(ctx, seq, payload, payload_len);
            return;
        }
    }

    // Unregistered/Disabled Command
    // Future implementation: Construct and send a NACK(ERROR_UNSUPPORTED_CMD) back to the host
}

#endif // _FILE_GMP_TUNABLE_H
