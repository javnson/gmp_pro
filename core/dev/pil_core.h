/**
 * @file tunable_sim.h
 * @brief GMP Tunable Simulation Layer - PIL Engine with 8-bit Stream Logic.
 * @details Optimized for DSP/ARM with weak-linkage callbacks and dynamic masking.
 * Provides a standardized memory-aligned interface for Processor-in-the-Loop simulations.
 */

#ifndef _FILE_GMP_TUNABLE_SIM_H
#define _FILE_GMP_TUNABLE_SIM_H

#include <core/dev/tunable.h>

// ---------------------------------------------------------
// 1. Types & Mask Definitions
// ---------------------------------------------------------

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

// ---------------------------------------------------------
// 2. Simulation Buffers (Cleaned)
// ---------------------------------------------------------

/** * @brief TX Buffer: Data sent FROM MCU TO PC.
 * @details Represents the output of the control algorithm. Perfectly aligned to 32-bit boundaries.
 */
typedef struct
{
    uint32_t digital_out; ///< Base state: Always transmitted (e.g., relay states, GPIOs)
    uint16_t pwm_cmp[8];  ///< Conditionally transmitted: PWM compare values
    uint16_t dac[8];      ///< Conditionally transmitted: Digital-to-Analog output values
    ctrl_gt monitor[16];  ///< Conditionally transmitted: Generic monitoring variables
} gmp_sim_tx_buf_t;

/** * @brief RX Buffer: Data received BY MCU FROM PC.
 * @details Represents the input/feedback to the control algorithm. Perfectly aligned to 32-bit boundaries.
 */
typedef struct
{
    uint32_t isr_ticks;      ///< Base state: Always received (Simulation time or tick count)
    uint32_t digital_input;  ///< Base state: Always received (e.g., fault signals, button states)
    uint16_t adc_result[24]; ///< Conditionally received: Simulated ADC sampling results
    ctrl_gt panel[8];        ///< Conditionally received: Simulated user panel inputs (knobs, references)
} gmp_sim_rx_buf_t;

// ---------------------------------------------------------
// 3. Module Context & Weak Callback
// ---------------------------------------------------------

// Command Offsets for Simulation
#define GMP_TUNABLE_OFFSET_SIM_SET_MASK_REQ 10 ///< Request to set TX/RX transmission masks
#define GMP_TUNABLE_OFFSET_SIM_SET_MASK_ACK 11 ///< Acknowledge mask configuration
#define GMP_TUNABLE_OFFSET_SIM_STEP_REQ     12 ///< Unpack inputs -> Execute step -> Pack outputs
#define GMP_TUNABLE_OFFSET_SIM_STEP_ACK     13 ///< Acknowledge step execution with output payload

#define GMP_TUNABLE_OFFSET_SIM_SET_INPUT_REQ  14 ///< Unpack inputs only (Override RX buffer), no step execution
#define GMP_TUNABLE_OFFSET_SIM_SET_INPUT_ACK  15 ///< Acknowledge input override
#define GMP_TUNABLE_OFFSET_SIM_GET_OUTPUT_REQ 16 ///< Pack outputs only (Fetch TX buffer), no step execution
#define GMP_TUNABLE_OFFSET_SIM_GET_OUTPUT_ACK 17 ///< Return packed output payload

/**
 * @brief PIL Simulation Module Context Structure.
 * @details Encapsulates datalink binding, internal buffers, and current mask configurations.
 */
typedef struct
{
    gmp_datalink_t* dl_ctx; ///< Pointer to the bound datalink instance
    uint16_t base_cmd;      ///< Shared base command offset for the tunable subsystem

    gmp_sim_mask_tx_t mask_tx; ///< Dynamic TX configuration mask
    gmp_sim_mask_rx_t mask_rx; ///< Dynamic RX configuration mask

    gmp_sim_tx_buf_t tx_buf; ///< Internal buffer for algorithm outputs
    gmp_sim_rx_buf_t rx_buf; ///< Internal buffer for algorithm inputs
} gmp_tunable_sim_t;

/**
 * @brief  Weak-linkage callback for the simulation step.
 * @details This function is invoked automatically upon receiving a STEP_REQ. 
 * It bridges the gap between the communication layer and the user's control algorithm.
 * @note   Override this function in your application code without using function pointers.
 * @param  rx Pointer to the fully unpacked RX buffer (Inputs from PC).
 * @param  tx Pointer to the TX buffer to be populated (Outputs to PC).
 */
extern void gmp_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx);

/**
 * @brief  Initialize the tunable simulation module.
 * @param  ctx      Pointer to the simulation context to initialize.
 * @param  dl_ctx   Pointer to the datalink instance used for sending ACKs.
 * @param  base_cmd The base command offset assigned to this subsystem.
 */
void gmp_tunable_sim_init(gmp_tunable_sim_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd);

/**
 * @brief  Receive callback dispatcher for the simulation module.
 * @param  ctx       Pointer to the initialized simulation context.
 * @param  target_id The destination ID of the received frame.
 * @param  cmd       The command byte extracted from the frame header.
 * @param  payload   Pointer to the payload data array.
 * @param  len       Length of the payload in data_gt elements.
 * @return fast_gt   GMP_TUNABLE_HANDLED (0) if processed, GMP_TUNABLE_PASS (1) if ignored.
 */
fast_gt gmp_tunable_sim_rx_cb(gmp_tunable_sim_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                              size_gt len);

#endif // _FILE_GMP_TUNABLE_SIM_H
