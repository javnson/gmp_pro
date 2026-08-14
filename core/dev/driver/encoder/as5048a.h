
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

#include <core/dev/driver/driver.h>

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
 * @note  Control-domain position, pole-pair and revolution tracking state is
 * intentionally kept outside this hardware driver.
 */
typedef struct _tag_as5048a_dev_t
{
    uint16_t raw;             /**< @brief Latest raw angle (14-bit, 0-16383). */
    spi_device_halt spi_node; /**< @brief Layer 2 Logical SPI Device Handle. */
    uint16_t diag_flags;      /**< @brief Cached diagnostics flags. */
    fast_gt err_flag;         /**< @brief SPI parity/framing error flag. */

} as5048a_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

/**
 * @brief Initializes the AS5048A encoder structure and checks hardware connection.
 * @param[out] dev       Pointer to the AS5048A encoder structure.
 * @param[in]  spi_node  Layer 2 SPI Logical Device handle.
 * @return ec_gt         Error code (GMP_EC_OK on success).
 */
ec_gt as5048a_init(as5048a_dev_t* dev, spi_device_halt spi_node);

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
 * @brief Acquires one raw angle sample from AS5048A.
 * @note  This direct-call fast path performs no CTL-domain conversion. A
 * control module may consume dev->raw and apply offsets, pole pairs or turn
 * tracking without making the hardware driver depend on CTL.
 * @param[in,out] dev Pointer to the AS5048A encoder structure.
 * @return ec_gt Transfer or device status.
 */
GMP_STATIC_INLINE ec_gt as5048a_step(as5048a_dev_t* dev)
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

    ec_gt result = gmp_hal_spi_dev_transfer(dev->spi_node, tx_buf, rx_buf, 2, AS5048A_CFG_TIMEOUT);
    if (result != GMP_EC_OK)
    {
        dev->err_flag = 1;
        return result;
    }

    uint16_t raw_res = ((uint16_t)rx_buf[0] << 8) | ((uint16_t)rx_buf[1] & 0x00FFU);

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
    dev->raw = (uint16_t)(raw_res & 0x3FFF);
    return dev->err_flag ? GMP_EC_GENERAL_ERROR : GMP_EC_OK;
}
#endif // _FILE_AS5048A_H_
