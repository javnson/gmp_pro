#include <gmp_core.h>

#include <core/dev/datalink.h>
#include <core/std/checksum/crc16.h>


/**
 * @brief Initialize the datalink context and inject dependencies.
 */
void gmp_datalink_init(gmp_datalink_t* ctx, uint16_t local_id, 
                       gmp_dl_hw_tx_cb tx_cb, gmp_dl_hw_tx_is_ready_cb tx_ready_cb,
                       gmp_dl_app_rx_cb rx_cb, gmp_dl_bypass_cb bypass_cb, 
                       gmp_dl_get_tick_cb time_cb) 
{
    memset(ctx, 0, sizeof(gmp_datalink_t));
    ctx->local_id = local_id;
    ctx->tx_func = tx_cb;
    ctx->tx_ready_func = tx_ready_cb;
    ctx->rx_func = rx_cb;
    ctx->bypass_func = bypass_cb;
    ctx->get_time_func = time_cb;
    
    ctx->rx_fifo_head = 0;
    ctx->rx_fifo_tail = 0;
    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
    ctx->tx_pending = 0;
}

/**
 * @brief  Extremely fast ISR handler to push data into the internal FIFO.
 * @note   This function contains NO parsing logic, making it safe for high-freq interrupts.
 * @param  ctx Pointer to the datalink context.
 * @param  raw_data The byte received from the hardware.
 */
void gmp_datalink_feed_byte(gmp_datalink_t* ctx, data_gt raw_data) 
{
    uint16_t next_head = (ctx->rx_fifo_head + 1) % GMP_DL_RX_FIFO_SIZE;
    
    if (next_head != ctx->rx_fifo_tail) {
        ctx->rx_fifo[ctx->rx_fifo_head] = raw_data & 0xFF; // Safe downcast
        ctx->rx_fifo_head = next_head;
    } else {
        ctx->err_fifo_ovf_cnt++; // FIFO is full, data dropped
    }
}

/**
 * @brief  Core event processor. Must be called periodically in the main loop/RTOS task.
 * @details Handles FSM parsing, Watchdog timeouts, and queued TX transmissions.
 * @param  ctx Pointer to the datalink context.
 */
void gmp_datalink_tick(gmp_datalink_t* ctx) 
{
    // =========================================================
    // EVENT 1: Parse RX FIFO (FSM Execution)
    // =========================================================
    while (ctx->rx_fifo_tail != ctx->rx_fifo_head) 
    {
        data_gt byte = ctx->rx_fifo[ctx->rx_fifo_tail];
        ctx->rx_fifo_tail = (ctx->rx_fifo_tail + 1) % GMP_DL_RX_FIFO_SIZE;

        // Refresh watchdog tick ONLY when actively processing data
        if (ctx->get_time_func) {
            ctx->last_rx_tick = ctx->get_time_func();
        }

        // --- State 0: Wait Sync (Gateway) ---
        if (ctx->rx_state == GMP_DL_STATE_WAIT_SYNC) {
            if (byte == GMP_DL_SOF) {
                ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
                ctx->hdr_idx = 0;
            } else if (ctx->bypass_func) {
                // Character does not belong to datalink, forward to CLI/Terminal
                ctx->bypass_func(byte); 
            }
            continue; // Move to next byte in FIFO
        }

        // --- Hard Reset Protection ---
        if ((ctx->rx_state == GMP_DL_STATE_HEADER_RECV || 
             ctx->rx_state == GMP_DL_STATE_HEADER_ESCAPE) && byte == GMP_DL_SOF) {
            ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
            ctx->hdr_idx = 0;
            continue;
        }

        // --- FSM Transitions ---
        switch (ctx->rx_state) {
            
            case GMP_DL_STATE_HEADER_RECV:
                if (byte == GMP_DL_ESC) {
                    ctx->rx_state = GMP_DL_STATE_HEADER_ESCAPE;
                } 
                else if (byte == GMP_DL_EOF) {
                    if (ctx->hdr_idx != GMP_DL_HDR_SIZE) {
                        ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
                        break;
                    }

                    // Header CRC Check (Little Endian mapping)
                    uint16_t h_crc_rcv = (ctx->hdr_buf[4] & 0xFF) | ((ctx->hdr_buf[5] & 0xFF) << 8);
                    uint16_t h_crc_calc = gmp_base_calculate_crc16(ctx->hdr_buf, 4);

                    if (h_crc_calc != h_crc_rcv) {
                        ctx->err_hdr_crc_cnt++;
                        ctx->rx_state = GMP_DL_STATE_WAIT_SYNC; 
                        break;
                    }

                    // Extract Routing info
                    ctx->current_target_id    = ctx->hdr_buf[0] & 0xFF;
                    ctx->current_cmd          = ctx->hdr_buf[1] & 0xFF;
                    ctx->expected_payload_len = (ctx->hdr_buf[2] & 0xFF) | ((ctx->hdr_buf[3] & 0xFF) << 8);

                    if (ctx->expected_payload_len > GMP_DL_MTU) {
                        ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
                        break;
                    }

                    if (ctx->current_target_id == ctx->local_id || ctx->current_target_id == 0xFF) {
                        ctx->rx_state = GMP_DL_STATE_PAYLOAD_RECV;
                    } else {
                        ctx->rx_state = GMP_DL_STATE_BYPASS_RECV;
                    }
                    ctx->payload_idx = 0;
                } 
                else {
                    if (ctx->hdr_idx < sizeof(ctx->hdr_buf)/sizeof(ctx->hdr_buf[0])) {
                        ctx->hdr_buf[ctx->hdr_idx++] = byte;
                    }
                }
                break;

            case GMP_DL_STATE_HEADER_ESCAPE:
                if (ctx->hdr_idx < sizeof(ctx->hdr_buf)/sizeof(ctx->hdr_buf[0])) {
                    ctx->hdr_buf[ctx->hdr_idx++] = byte ^ GMP_DL_XOR;
                }
                ctx->rx_state = GMP_DL_STATE_HEADER_RECV;
                break;

            case GMP_DL_STATE_PAYLOAD_RECV:
                ctx->payload_buf[ctx->payload_idx++] = byte;
                
                if (ctx->payload_idx == (ctx->expected_payload_len + 2)) {
                    uint16_t p_crc_rcv = (ctx->payload_buf[ctx->expected_payload_len] & 0xFF) | 
                                         ((ctx->payload_buf[ctx->expected_payload_len + 1] & 0xFF) << 8);
                    uint16_t p_crc_calc = gmp_base_calculate_crc16(ctx->payload_buf, ctx->expected_payload_len);

                    if (p_crc_calc == p_crc_rcv) {
                        if (ctx->rx_func) {
                            ctx->rx_func(ctx->current_target_id, ctx->current_cmd, 
                                         ctx->payload_buf, ctx->expected_payload_len);
                        }
                    } else {
                        ctx->err_pld_crc_cnt++;
                    }
                    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
                }
                break;

            case GMP_DL_STATE_BYPASS_RECV:
                // Blindly count and ignore foreign payload
                ctx->payload_idx++;
                if (ctx->payload_idx == (ctx->expected_payload_len + 2)) {
                    ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
                }
                break;
                
            default:
                ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
                break;
        }
    }

    // =========================================================
    // EVENT 2: Check RX Watchdog Timeout
    // =========================================================
    if (ctx->rx_state != GMP_DL_STATE_WAIT_SYNC && ctx->get_time_func) {
        if (gmp_base_is_delay_elapsed(ctx->last_rx_tick, GMP_DL_OVERTIME)) {
            // FSM is stuck (e.g., missing EOF or cable disconnected), force reset
            ctx->rx_state = GMP_DL_STATE_WAIT_SYNC;
            ctx->err_timeout_cnt++;
        }
    }

    // =========================================================
    // EVENT 3: Handle Pending TX Transmissions
    // =========================================================
    if (ctx->tx_pending && ctx->tx_func) {
        // Assume hardware is ready if no callback is provided
        fast_gt hw_ready = (ctx->tx_ready_func != NULL) ? ctx->tx_ready_func() : 1;
        
        if (hw_ready) {
            ctx->tx_func(ctx->tx_buf, ctx->tx_len);
            ctx->tx_pending = 0; // Release the TX lock
        }
    }
}

/**
 * @brief  Formats and queues a physical frame for transmission.
 * @return 1 if successfully buffered, 0 if the TX queue is currently busy.
 */
fast_gt gmp_datalink_send(gmp_datalink_t* ctx, uint16_t target_id, uint16_t cmd, 
                          const data_gt* payload, size_gt len) 
{
    // Ensure thread/concurrency safety: block new requests if buffer is full
    if (ctx->tx_pending) {
        return 0; // BUSY
    }

    size_gt tx_idx = 0;
    
    // 1. Header Composition
    data_gt raw_hdr[4];
    raw_hdr[0] = target_id & 0xFF;
    raw_hdr[1] = cmd & 0xFF;
    raw_hdr[2] = len & 0xFF;
    raw_hdr[3] = (len >> 8) & 0xFF;
    uint16_t h_crc = gmp_base_calculate_crc16(raw_hdr, 4);

    // 2. Escape Header 
    ctx->tx_buf[tx_idx++] = GMP_DL_SOF;
    for (int i = 0; i < 6; i++) {
        data_gt b;
        if (i < 4) b = raw_hdr[i];
        else if (i == 4) b = h_crc & 0xFF;
        else b = (h_crc >> 8) & 0xFF;

        if (b == GMP_DL_SOF || b == GMP_DL_EOF || b == GMP_DL_ESC) {
            ctx->tx_buf[tx_idx++] = GMP_DL_ESC;
            ctx->tx_buf[tx_idx++] = b ^ GMP_DL_XOR;
        } else {
            ctx->tx_buf[tx_idx++] = b;
        }
    }
    ctx->tx_buf[tx_idx++] = GMP_DL_EOF;

    // 3. Blind Payload Composition (Zero Escape)
    if (len > 0 && payload != NULL) {
        for (size_gt i = 0; i < len; i++) {
            ctx->tx_buf[tx_idx++] = payload[i] & 0xFF;
        }
        uint16_t p_crc = gmp_base_calculate_crc16(payload, len);
        ctx->tx_buf[tx_idx++] = p_crc & 0xFF;
        ctx->tx_buf[tx_idx++] = (p_crc >> 8) & 0xFF;
    } else {
        ctx->tx_buf[tx_idx++] = 0x00;
        ctx->tx_buf[tx_idx++] = 0x00;
    }

    // 4. Mark as pending and trigger immediate TX evaluation
    ctx->tx_len = tx_idx;
    ctx->tx_pending = 1;

    // Fast-path execution
    gmp_datalink_tick(ctx);

    return 1; // SUCCESS
}
