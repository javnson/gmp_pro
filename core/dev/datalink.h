/**
 * @file datalink.h
 * @brief GMP Data Link Layer - Asynchronous FSM & Hardware Agnostic Protocol.
 * * HOW TO USE THIS MODULE:
 * 1. Initialization:
 * Call gmp_datalink_init() once during system setup to inject your hardware 
 * callbacks (TX, TX_Ready, RX_App_Callback, Bypass_Callback).
 * 2. Interrupt Service Routine (ISR) Context:
 * Call gmp_datalink_feed_byte() or gmp_datalink_feed_str() inside your UART/Bus RX interrupt.
 * This function is extremely short O(1) and only pushes data to an internal FIFO.
 * 3. Main Loop / RTOS Task Context:
 * Call gmp_datalink_tick() periodically. This function handles 3 events:
 * - Pops data from the FIFO and drives the FSM parsing.
 * - Checks the watchdog timer to recover from deadlocks.
 * - Pushes pending TX data to the hardware if the bus is idle.
 * 4. Application Context:
 * Call gmp_datalink_send() to pack and queue a frame for transmission.
 */

#ifndef _FILE_GMP_DATALINK_H
#define _FILE_GMP_DATALINK_H

// ---------------------------------------------------------
// 1. Configurable Macros (Can be overridden in project config)
// ---------------------------------------------------------
#ifndef GMP_DL_MTU
#define GMP_DL_MTU 256          ///< Maximum Payload Size (Bytes/Words)
#endif

#ifndef GMP_DL_RX_FIFO_SIZE
#define GMP_DL_RX_FIFO_SIZE 256 ///< Internal ISR decoupling FIFO size (Must be power of 2 for best perf)
#endif

#ifndef GMP_DL_OVERTIME
#define GMP_DL_OVERTIME 50      ///< Watchdog timeout threshold in milliseconds
#endif 

#ifndef GMP_DL_DEBUG_CLEAR_MEM
#define GMP_DL_DEBUG_CLEAR_MEM 1 ///< Set to 1 to memset buffers to 0 upon frame reset (helps debugging in IDE)
#endif

// ---------------------------------------------------------
// 2. Types Definitions (Assuming defined in GMP base)
// ---------------------------------------------------------

// Framing Boundaries
#define GMP_DL_SOF '{'          ///< 0x7B, Start of Frame
#define GMP_DL_EOF '}'          ///< 0x7D, End of Frame
#define GMP_DL_ESC '%'          ///< 0x25, Escape Character
#define GMP_DL_XOR 0x20         ///< 0x20, XOR mask for escape restoration
#define GMP_DL_HDR_SIZE 6       ///< TargetID(1) + CMD(1) + LEN(2) + H_CRC(2)

// ---------------------------------------------------------
// 3. FSM States Definition
// ---------------------------------------------------------
/** @brief State machine enumerations for the parser */
typedef enum {
    GMP_DL_STATE_WAIT_SYNC = 0,   ///< Waiting for '{'
    GMP_DL_STATE_HEADER_RECV,     ///< Receiving Header
    GMP_DL_STATE_HEADER_ESCAPE,   ///< Restoring escaped Header byte
    GMP_DL_STATE_PAYLOAD_RECV,    ///< Blindly receiving pure Payload & P_CRC
    GMP_DL_STATE_BYPASS_RECV      ///< Blindly consuming/ignoring a bypassed frame
} gmp_dl_state_t;

// ---------------------------------------------------------
// 4. Callbacks (Dependency Injection)
// ---------------------------------------------------------
/** @brief Hardware TX function to push formatted physical frame to UART/DMA */
typedef void (*gmp_dl_hw_tx_cb)(const data_gt* raw_bytes, size_gt len);

/** @brief Hardware Status Query. Returns 1 if TX is free, 0 if busy. */
typedef fast_gt (*gmp_dl_hw_tx_is_ready_cb)(void);

/** @brief App RX function triggered when a PERFECT payload is received */
typedef void (*gmp_dl_app_rx_cb)(uint16_t target_id, uint16_t cmd, const data_gt* payload, size_gt len);

/** @brief Called when a byte is rejected by the Datalink FSM (e.g., for CLI routing) */
typedef void (*gmp_dl_bypass_cb)(data_gt byte);

// ---------------------------------------------------------
// 5. Context Structure
// ---------------------------------------------------------

/** * @brief Datalink Context holding FSM states, buffers, and injected callbacks.
 * @details This structure maintains the complete lifecycle of the data link layer,
 * including RX/TX state machines, hardware decoupling FIFOs, and diagnostics.
 */
typedef struct {
    // --- Configuration ---
    uint16_t local_id;                 ///< ID of this specific node (0xFF for Broadcast)

    // --- Injected Callbacks ---
    gmp_dl_hw_tx_cb          tx_func;       ///< Hardware TX function to push formatted frame
    gmp_dl_hw_tx_is_ready_cb tx_ready_func; ///< Hardware Status Query function (returns 1 if TX is free)
    gmp_dl_app_rx_cb         rx_func;       ///< App RX function triggered when a valid payload is received
    gmp_dl_bypass_cb         bypass_func;   ///< Optional CLI forwarder for bytes not belonging to datalink

    // --- ISR Decoupling FIFO ---
    data_gt  rx_fifo[GMP_DL_RX_FIFO_SIZE];  ///< Internal ring buffer for ISR to Main-Loop decoupling
    uint16_t rx_fifo_head;             ///< ISR write index for the RX FIFO
    uint16_t rx_fifo_tail;             ///< Main loop read index for the RX FIFO

    // --- RX State Machine ---
    gmp_dl_state_t rx_state;           ///< Current state of the RX Finite State Machine
    time_gt        last_rx_tick;       ///< Watchdog timestamp of the last received valid byte
    
    uint16_t hdr_idx;                  ///< Current index for un-escaped header parsing
    data_gt  hdr_buf[16];              ///< Buffer for un-escaped header parsing
    
    uint16_t current_cmd;              ///< Command ID extracted from the current frame header
    uint16_t current_target_id;        ///< Target ID extracted from the current frame header
    uint16_t expected_payload_len;     ///< Expected payload length extracted from the current frame header
    uint16_t payload_idx;              ///< Current index for payload reception
    data_gt  payload_buf[GMP_DL_MTU + 2]; ///< Buffer for payload data (MTU + 2 bytes for Payload CRC)

    // --- TX State Machine & Buffer ---
    fast_gt  tx_pending;               ///< Flag indicating if data is waiting to be sent (1 = pending, 0 = idle)
    size_gt  tx_len;                   ///< Length of the pending TX physical frame
    data_gt  tx_buf[GMP_DL_MTU + 32];  ///< Buffer used to compose the physical frame before transmission

    // --- Diagnostics ---
    uint32_t err_fifo_ovf_cnt;         ///< Number of ISR FIFO overflows (data lost due to full buffer)
    uint32_t err_hdr_crc_cnt;          ///< Number of Header CRC verification failures
    uint32_t err_pld_crc_cnt;          ///< Number of Payload CRC verification failures
    uint32_t err_timeout_cnt;          ///< Number of Watchdog timeout resets triggered
} gmp_datalink_t;

// ---------------------------------------------------------
// 6. API Declarations
// ---------------------------------------------------------

/**
 * @brief  Initialize the datalink context and inject hardware dependencies.
 * @param  ctx           Pointer to the datalink context instance.
 * @param  local_id      The node ID assigned to this device.
 * @param  tx_cb         Callback for hardware UART/Bus transmission.
 * @param  tx_ready_cb   Callback to query if the hardware TX is idle (can be NULL).
 * @param  rx_cb         Callback triggered when a pristine frame payload is received.
 * @param  bypass_cb     Callback triggered to forward unhandled characters (e.g., CLI).
 */
void gmp_datalink_init(gmp_datalink_t* ctx, uint16_t local_id, 
                       gmp_dl_hw_tx_cb tx_cb, gmp_dl_hw_tx_is_ready_cb tx_ready_cb,
                       gmp_dl_app_rx_cb rx_cb, gmp_dl_bypass_cb bypass_cb);

/**
 * @brief  Extremely fast ISR handler to push a single byte into the internal FIFO.
 * @note   This function contains NO parsing logic, making it safe for high-frequency interrupts.
 * @param  ctx           Pointer to the datalink context instance.
 * @param  raw_data      The raw byte received from the hardware.
 */
void gmp_datalink_feed_byte(gmp_datalink_t* ctx, data_gt raw_data);

/**
 * @brief  Extremely fast ISR handler to push a block of data into the internal FIFO.
 * @note   Ideal for DMA receive complete interrupts or UART IDLE interrupts.
 * @param  ctx           Pointer to the datalink context instance.
 * @param  str           Pointer to the data array (block) to be fed.
 * @param  size          Number of elements to feed into the FIFO.
 */
void gmp_datalink_feed_str(gmp_datalink_t* ctx, const data_gt *str, size_gt size);

/**
 * @brief  Core event processor. Must be called periodically in the main loop or RTOS task.
 * @details Handles FSM parsing, Watchdog timeouts, and queued TX transmissions non-blockingly.
 * @param  ctx           Pointer to the datalink context instance.
 */
void gmp_datalink_tick(gmp_datalink_t* ctx);

/**
 * @brief  Formats, escapes, appends CRCs, and queues a physical frame for transmission.
 * @param  ctx           Pointer to the datalink context instance.
 * @param  target_id     The ID of the target device to receive this frame.
 * @param  cmd           The command identifier for the payload.
 * @param  payload       Pointer to the raw data payload.
 * @param  len           Length of the payload in bytes/words.
 * @return fast_gt       1 if successfully buffered, 0 if the TX queue is currently busy.
 */
fast_gt gmp_datalink_send(gmp_datalink_t* ctx, uint16_t target_id, uint16_t cmd, 
                          const data_gt* payload, size_gt len);

#endif // _FILE_GMP_DATALINK_H