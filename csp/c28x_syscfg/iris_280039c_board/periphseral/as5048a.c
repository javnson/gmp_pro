
// AS5048A = AS5048 with SPI interface
//

// The AS5048 is a magnetic Hall sensor system manufactured in a CMOS process. A lateral Hall sensor array is used to
// measure the magnetic field components perpendicular to the surface of the chip.The AS5048 is uses self - calibration
// methods to eliminate signal offset and sensitivity drifts.
//
//  + SPI MODE = 1
// The 16 bit SPI Interface enables read / write access to the register blocks and is compatible to a standard micro
// controller interface. The SPI is active as soon as CSn is pulled low. The AS5048A then reads the digital value on the
// MOSI(master out slave in) input with every falling edge of CLK and writes on its MISO(master in slave out) output
// with the rising edge. After 16 clock cycles CSn has to be set back to a high status in order to reset some parts of
// the interface core.
//

// Pure read mode: write 0xFF 0xFF and meanwhile read the result.
//

// register access mode:
//

// STM32 Demo
/*
uint8_t enc_req[2] = {0xFF, 0xFF};
uint16_t enc_res = 0;
HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
HAL_SPI_TransmitReceive(&hspi2, enc_req, (uint8_t *)&enc_res, 2, 10);
HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
ctl_step_pos_encoder(&pos_enc, 0x3FFF - (gmp_l2b16(enc_res) & 0x3FFF));
*/

#include <gmp_core.h>

#include <ctl/component/motor_control/basic/encoder.h>

#ifndef _FILE_AS5048A_H_
#define _FILE_AS5048A_H_

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

#ifndef AS5048A_CFG_TIMEOUT
#define AS5048A_CFG_TIMEOUT (5U) /**< @brief Timeout for SPI transfers in ms */
#endif

/* AS5048A Register Addresses (14-bit) */
#define AS5048A_REG_NOP           0x0000 /**< @brief No Operation / Dummy Read */
#define AS5048A_REG_CLEAR_ERROR   0x0001 /**< @brief Clear Error Flag Register */
#define AS5048A_REG_PROGRAM_CTRL  0x0003 /**< @brief Programming Control Register */
#define AS5048A_REG_OTP_ZERO_HIGH 0x0015 /**< @brief OTP Zero Position MSB */
#define AS5048A_REG_OTP_ZERO_LOW  0x0016 /**< @brief OTP Zero Position LSB */
#define AS5048A_REG_DIAGNOSTICS   0x3FFD /**< @brief Diagnostics and AGC Register */
#define AS5048A_REG_MAGNITUDE     0x3FFE /**< @brief Magnitude Register */
#define AS5048A_REG_ANGLE         0x3FFF /**< @brief Angle Register (14-bit) */

#define AS5048A_CMD_READ  (1 << 14) /**< @brief Read Command Bit */
#define AS5048A_CMD_WRITE (0 << 14) /**< @brief Write Command Bit */

/* ========================================================================= */
/* ==================== DEVICE STRUCTURE =================================== */
/* ========================================================================= */

/* ========================================================================= */
/* ==================== DEVICE STRUCTURE =================================== */
/* ========================================================================= */

/**
 * @brief Data structure for the AS5048A magnetic position encoder.
 * @note  The encif member must remain at the beginning of the struct to allow
 * safe pointer casting to rotation_ift*.
 */
typedef struct _tag_as5048a_dev_t
{
    /* --- Standard Control Interface (Must be first!) --- */
    rotation_ift encif; /**< @brief Standard rotation interface for output. */

    /* --- Internal Tracking Variables --- */
    uint32_t raw;           /**< @brief Raw data from the AS5048A (14-bit, 0-16383). */
    ctrl_gt offset;         /**< @brief Position offset in per-unit (p.u.). */
    uint16_t pole_pairs;    /**< @brief Number of motor pole pairs. */
    ctrl_gt last_pos;       /**< @brief Last recorded mechanical position (p.u.) for turn counting. */
    uint32_t position_base; /**< @brief Base value for a full mechanical revolution (16384 for AS5048A). */

    /* --- Hardware Interface & Diagnostics --- */
    spi_device_halt spi_node; /**< @brief Layer 2 Logical SPI Device Handle. */
    uint16_t diag_flags;      /**< @brief Cached diagnostics flags (OCF, COF, COMP). */
    fast_gt err_flag;            /**< @brief SPI communication parity/framing error flag. */

} as5048a_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief Initializes the AS5048A encoder structure and checks hardware connection.
 * @param[out] dev       Pointer to the AS5048A encoder structure.
 * @param[in]  spi_node  Layer 2 SPI Logical Device handle.
 * @param[in]  poles     Number of motor pole pairs.
 * @return ec_gt         Error code (GMP_EC_OK on success).
 */
ec_gt as5048a_init(as5048a_dev_t* dev, spi_device_halt spi_node, uint16_t poles);

/**
 * @brief  Updates the diagnostics status from the AS5048A hardware.
 * @param[in,out] dev    Pointer to the AS5048A encoder structure.
 * @return ec_gt         Error code (GMP_EC_OK on success).
 */
ec_gt as5048a_update_diagnostics(as5048a_dev_t* dev);

/**
 * @brief  Clears the internal error flag register of the AS5048A.
 * @param[in,out] dev    Pointer to the AS5048A encoder structure.
 * @return ec_gt         Error code (GMP_EC_OK on success).
 */
ec_gt as5048a_clear_error_flag(as5048a_dev_t* dev);

/* ========================================================================= */
/* ==================== INLINE CRITICAL CONTROL FUNCTIONS ================== */
/* ========================================================================= */

/**
 * @brief Sets the mechanical offset for the encoder from a raw hardware value.
 * @param[in,out] dev    Pointer to the AS5048A encoder structure.
 * @param[in]     raw    The raw encoder value (14-bit) that corresponds to electrical zero.
 */
GMP_STATIC_INLINE void as5048a_set_offset_raw(as5048a_dev_t* dev, uint32_t raw)
{
    dev->offset = float2ctrl((ctrl_gt)raw / dev->position_base);
}

/**
 * @brief Sets the mechanical offset for the encoder directly in per-unit (p.u.).
 * @param[in,out] dev    Pointer to the AS5048A encoder structure.
 * @param[in]    _offset The offset value in per-unit (0.0 to 1.0).
 */
GMP_STATIC_INLINE void as5048a_set_mech_offset(as5048a_dev_t* dev, ctrl_gt _offset)
{
    dev->offset = _offset;
}

/**
 * @brief Calculates the even parity bit for a 16-bit word.
 * @note  Required by AS5048A hardware communication protocol.
 * @param[in] val        The 16-bit value to check.
 * @return uint16_t      1 if parity is odd (requires padding), 0 if even.
 */
GMP_STATIC_INLINE uint16_t as5048a_calc_parity(uint16_t val)
{
    val ^= val >> 8;
    val ^= val >> 4;
    val ^= val >> 2;
    val ^= val >> 1;
    return val & 1;
}

/**
 * @brief Processes a new reading from AS5048A and updates the auto-turn revolution count.
 * @note  This function performs the SPI transfer, checks parity, calculates p.u.
 * position, tracks revolution rollovers, and computes the electrical angle.
 * Designed for high-frequency execution in the FOC control loop.
 * * @param[in,out] dev    Pointer to the AS5048A encoder structure.
 * @return ctrl_gt       The calculated electrical position in per-unit (0.0 to 1.0).
 */
GMP_STATIC_INLINE ctrl_gt as5048a_step(as5048a_dev_t* dev)
{
    /* 1. Assemble the Read Angle Command: (0x3FFF | Read(1<<14)) = 0x7FFF */
    uint16_t cmd = 0x7FFF;
    if (as5048a_calc_parity(cmd))
    {
        cmd |= 0x8000; /* Pad MSB to ensure Even Parity */
    }

    /* 2. Execute SPI Transfer (Layer 2 API handles CS automatically) */
    data_gt tx_buf[2], rx_buf[2];
    tx_buf[0] = (data_gt)(cmd >> 8);
    tx_buf[1] = (data_gt)(cmd & 0xFF);

    gmp_hal_spi_dev_transfer(dev->spi_node, tx_buf, rx_buf, 2, AS5048A_CFG_TIMEOUT);

    uint16_t raw_res = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    /* 3. Hardware Error & Parity Verification */
    dev->err_flag = 0;
    if (as5048a_calc_parity(raw_res & 0x7FFF) != (raw_res >> 15))
    {
        dev->err_flag = 1; /* Parity Mismatch */
    }
    if (raw_res & 0x4000)
    {
        dev->err_flag = 1; /* AS5048A Internal Error Flag (Bit 14) */
    }

    /* 4. Extract Raw Data (Lower 14 bits) */
    dev->raw = (uint32_t)(raw_res & 0x3FFF);

    /* 5. Calculate Mechanical Position (p.u.) */
    dev->encif.position = ctl_div(dev->raw, dev->position_base);

    /* 6. Calculate Electrical Position (p.u.) */
    ctrl_gt elec_pos = (dev->encif.position + CTL_CTRL_CONST_1 - dev->offset) * dev->pole_pairs;
    dev->encif.elec_position = ctrl_mod_1(elec_pos);

    /* 7. Auto-Turn Counting Logic (Detect Rollovers) */
    if (dev->encif.position - dev->last_pos > CTL_CTRL_CONST_1_OVER_2)
    {
        dev->encif.revolutions -= 1; /* Negative direction rollover */
    }
    if (dev->last_pos - dev->encif.position > CTL_CTRL_CONST_1_OVER_2)
    {
        dev->encif.revolutions += 1; /* Positive direction rollover */
    }

    /* 8. Record position for next iteration */
    dev->last_pos = dev->encif.position;

    return dev->encif.elec_position;
}
#endif // _FILE_AS5048A_H_
