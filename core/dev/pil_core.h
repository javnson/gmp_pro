/**
 * @file tunable_sim.h
 * @brief GMP Tunable Simulation Layer - PIL Engine with 8-bit Stream Logic.
 * @details Optimized for DSP/ARM with weak-linkage callbacks and dynamic masking.
 * Provides a standardized memory-aligned interface for Processor-in-the-Loop simulations.
 * * HOW TO USE:
 * 1. Declare a `gmp_pil_sim_t` instance and call `gmp_tunable_sim_init()`.
 * 2. In your Datalink RX_OK event dispatcher, pass the context to `gmp_tunable_sim_rx_cb()`.
 * 3. Implement the user-level algorithm inside `gmp_sim_step()`.
 */

#ifndef _FILE_GMP_TUNABLE_SIM_H
#define _FILE_GMP_TUNABLE_SIM_H

#include <core/dev/datalink.h> // Dependency on the Datalink layer

// =========================================================
// 1. Simulation Command Offsets
// =========================================================

/** @brief Offset 1: Request to set TX/RX transmission masks */
#define GMP_PIL_OFFSET_SIM_SET_MASK_REQ 1
/** @brief Offset 2: Unpack inputs -> Execute step -> Pack outputs and reply */
#define GMP_PIL_OFFSET_SIM_STEP_REQ 2
/** @brief Offset 3: Unpack inputs only (Override RX buffer), NO step execution */
#define GMP_PIL_OFFSET_SIM_SET_INPUT_REQ 3
/** @brief Offset 4: Pack current outputs and reply, NO step execution */
#define GMP_PIL_OFFSET_SIM_GET_OUTPUT_REQ 4

// =========================================================
// 2. Types & Mask Definitions
// =========================================================

/**
 * @brief Bit-field mask for transmission (MCU to PC).
 * @details Determines which conditional fields in the TX buffer are actively serialized and sent.
 */
typedef union {
    uint32_t all; ///< 32-bit raw mask value
    struct
    {
        uint32_t pwm_cmp : 8;  ///< Bits 0-7:   Enable flags for pwm_cmp[0..7]
        uint32_t dac : 8;      ///< Bits 8-15:  Enable flags for dac[0..7]
        uint32_t monitor : 16; ///< Bits 16-31: Enable flags for monitor[0..15]
    } bit;
} gmp_sim_mask_tx_t;

/**
 * @brief Bit-field mask for reception (PC to MCU).
 * @details Determines which conditional fields in the RX buffer are expected to be unpacked.
 */
typedef union {
    uint32_t all; ///< 32-bit raw mask value
    struct
    {
        uint32_t adc_result : 24; ///< Bits 0-23:  Enable flags for adc_result[0..23]
        uint32_t panel : 8;       ///< Bits 24-31: Enable flags for panel[0..7]
    } bit;
} gmp_sim_mask_rx_t;

/**
 * @brief Safe Type-Punning Union for Strict-Aliasing compliance.
 * @details Used to safely convert IEEE754 floats to uint32 arrays for byte-stream serialization.
 */
typedef union {
    ctrl_gt f_val;  ///< Float viewpoint (32-bit)
    uint32_t u_val; ///< Unsigned integer viewpoint (32-bit)
} gmp_safe_pun_t;

// =========================================================
// 3. Simulation Buffers
// =========================================================

/** * @brief TX Buffer: Data sent FROM MCU TO PC (Algorithm Output).
 * @details Represents the output of the control algorithm. Perfectly aligned to 32-bit boundaries.
 */
typedef struct
{
    uint32_t digital_out; ///< Base state: Always transmitted (e.g., relay states, GPIOs)
    uint16_t pwm_cmp[8];  ///< Conditionally transmitted: PWM compare values
    uint16_t dac[8];      ///< Conditionally transmitted: Digital-to-Analog output values
    ctrl_gt monitor[16];  ///< Conditionally transmitted: Generic monitoring variables
} gmp_sim_tx_buf_t;

/** * @brief RX Buffer: Data received BY MCU FROM PC (Algorithm Input).
 * @details Represents the input/feedback to the control algorithm. Perfectly aligned to 32-bit boundaries.
 */
typedef struct
{
    uint32_t isr_ticks;      ///< Base state: Always received (Simulation time or tick count)
    uint32_t digital_input;  ///< Base state: Always received (e.g., fault signals, button states)
    uint16_t adc_result[24]; ///< Conditionally received: Simulated ADC sampling results
    ctrl_gt panel[8];        ///< Conditionally received: Simulated user panel inputs (knobs, references)
} gmp_sim_rx_buf_t;

// =========================================================
// 4. Module Context
// =========================================================

/**
 * @brief PIL Simulation Module Context Structure.
 * @details Encapsulates datalink binding, internal buffers, and current mask configurations.
 */
typedef struct
{
    gmp_datalink_t* dl_ctx; ///< Pointer to the bound datalink instance for auto-reply
    uint16_t base_cmd;      ///< Shared base command offset for the tunable subsystem

    gmp_sim_mask_tx_t mask_tx; ///< Dynamic TX configuration mask (Filters outgoing data)
    gmp_sim_mask_rx_t mask_rx; ///< Dynamic RX configuration mask (Filters incoming data)

    gmp_sim_tx_buf_t tx_buf; ///< Internal buffer for algorithm outputs
    gmp_sim_rx_buf_t rx_buf; ///< Internal buffer for algorithm inputs
} gmp_pil_sim_t;

// =========================================================
// 5. API Declarations
// =========================================================

/**
 * @brief  Initialize the tunable simulation module.
 * @param  ctx      Pointer to the simulation context to initialize.
 * @param  dl_ctx   Pointer to the datalink instance used for payload unpacking and replying.
 * @param  base_cmd The base command offset assigned to this subsystem (e.g., 0x10).
 */
void gmp_pil_sim_init(gmp_pil_sim_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd);

/**
 * @brief  Receive callback dispatcher for the simulation module.
 * @details Checks if the current command in `dl_ctx` belongs to this PIL subsystem. 
 * If matched, it unpacks data, executes steps, and queues replies accordingly.
 * @param  ctx      Pointer to the initialized simulation context.
 * @return fast_gt  1 (Handled) if the command belongs to this module, 0 (Pass) if not.
 */
fast_gt gmp_pil_sim_rx_cb(gmp_pil_sim_t* ctx);

/**
 * @brief  User-Implemented algorithm step function.
 * @details This function is invoked automatically upon receiving a STEP_REQ. 
 * It bridges the gap between the communication layer and the user's control algorithm.
 * @note   Implement this function in your application code.
 * @param  rx Pointer to the fully unpacked RX buffer (Inputs from PC).
 * @param  tx Pointer to the TX buffer to be populated (Outputs to PC).
 */
extern void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx);

#endif // _FILE_GMP_TUNABLE_SIM_H
