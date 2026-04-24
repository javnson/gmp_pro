/**
 * @file gmp_datalink.h
 * @brief GMP Data Link Layer - Asynchronous FSM & Hardware Agnostic Protocol.
 * * @details 
 * HOW TO USE THIS MODULE:
 * * 1. Initialization:
 * Call gmp_datalink_init() once during system setup to inject your hardware 
 * callbacks (TX, TX_Ready, RX_App_Callback, Time_Tick, Bypass_Callback).
 * * 2. Interrupt Service Routine (ISR) Context:
 * Call gmp_datalink_feed_byte() inside your UART/Bus RX interrupt.
 * This function is extremely short O(1) and only pushes data to an internal FIFO.
 * * 3. Main Loop / RTOS Task Context:
 * Call gmp_datalink_tick() periodically. This function handles 3 events:
 * - Pops data from the FIFO and drives the FSM parsing.
 * - Checks the watchdog timer to recover from deadlocks.
 * - Pushes pending TX data to the hardware if the bus is idle.
 * * 4. Application Context:
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

/** @brief Time fetcher for watchdog timeout calculation */
typedef time_gt (*gmp_dl_get_tick_cb)(void);

/** @brief Called when a byte is rejected by the Datalink FSM (e.g., for CLI routing) */
typedef void (*gmp_dl_bypass_cb)(data_gt byte);

// ---------------------------------------------------------
// 5. Context Structure
// ---------------------------------------------------------
/** @brief Datalink Context holding FSM states, buffers, and injected callbacks */
typedef struct {
    // --- Configuration ---
    uint16_t local_id;                 ///< ID of this specific node (0xFF for Broadcast)

    // --- Injected Callbacks ---
    gmp_dl_hw_tx_cb          tx_func;
    gmp_dl_hw_tx_is_ready_cb tx_ready_func; 
    gmp_dl_app_rx_cb         rx_func;
    gmp_dl_get_tick_cb       get_time_func;
    gmp_dl_bypass_cb         bypass_func;  ///< Optional CLI forwarder

    // --- ISR Decoupling FIFO ---
    data_gt  rx_fifo[GMP_DL_RX_FIFO_SIZE];
    uint16_t rx_fifo_head;             ///< ISR write index
    uint16_t rx_fifo_tail;             ///< Main loop read index

    // --- RX State Machine ---
    gmp_dl_state_t rx_state;
    time_gt        last_rx_tick;       ///< Watchdog timestamp
    
    uint16_t hdr_idx;
    data_gt  hdr_buf[16];              ///< Buffer for un-escaped header parsing
    
    uint16_t current_cmd;
    uint16_t current_target_id;
    uint16_t expected_payload_len;
    uint16_t payload_idx;
    data_gt  payload_buf[GMP_DL_MTU + 2]; ///< MTU + 2 bytes for Payload CRC

    // --- TX State Machine & Buffer ---
    fast_gt  tx_pending;               ///< 1 if data is waiting to be sent, 0 if idle
    size_gt  tx_len;                   ///< Length of pending TX frame
    data_gt  tx_buf[GMP_DL_MTU + 32];  ///< Buffer to compose the physical frame

    // --- Diagnostics ---
    uint32_t err_fifo_ovf_cnt;         ///< Number of ISR FIFO overflows
    uint32_t err_hdr_crc_cnt;          ///< Number of Header CRC failures
    uint32_t err_pld_crc_cnt;          ///< Number of Payload CRC failures
    uint32_t err_timeout_cnt;          ///< Number of Watchdog timeout resets
} gmp_datalink_t;

// ---------------------------------------------------------
// 6. API Declarations
// ---------------------------------------------------------
void gmp_datalink_init(gmp_datalink_t* ctx, uint16_t local_id, 
                       gmp_dl_hw_tx_cb tx_cb, gmp_dl_hw_tx_is_ready_cb tx_ready_cb,
                       gmp_dl_app_rx_cb rx_cb, gmp_dl_bypass_cb bypass_cb, 
                       gmp_dl_get_tick_cb time_cb);

void gmp_datalink_feed_byte(gmp_datalink_t* ctx, data_gt raw_data);

void gmp_datalink_tick(gmp_datalink_t* ctx);

fast_gt gmp_datalink_send(gmp_datalink_t* ctx, uint16_t target_id, uint16_t cmd, 
                          const data_gt* payload, size_gt len);

// Extern provided CRC calculation & Time functions
extern uint16_t gmp_base_calculate_crc16(const data_gt* data, size_gt len);
extern fast_gt gmp_base_is_delay_elapsed(time_gt t0, uint32_t delay_t);

#endif // _FILE_GMP_DATALINK_H
