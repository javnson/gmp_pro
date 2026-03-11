/**
 * @file    ht16k33.h
 * @brief   Driver layer for HT16K33 LED matrix / Key scan controller.
 */

 #include <gmp_core.h>

#ifndef _GMP_DEV_HT16K33_H
#define _GMP_DEV_HT16K33_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/* Registers Definitions (Ref: Linux ht16k33.c) */
#define HT16K33_REG_SYSTEM_SETUP  0x20
#define HT16K33_REG_DISPLAY_SETUP 0x80
#define HT16K33_REG_ROWINT_SET    0xA0
#define HT16K33_REG_BRIGHTNESS    0xE0
#define HT16K33_REG_KEY_DATA_ADDR 0x40

#define HT16K33_CMD_OSC_ON     (HT16K33_REG_SYSTEM_SETUP | 0x01)
#define HT16K33_CMD_DISPLAY_ON (HT16K33_REG_DISPLAY_SETUP | 0x01)

/**
 * @brief Initialization parameters for HT16K33.
 * Used only during the initialization phase.
 */
typedef struct
{
    uint8_t brightness; /**< Brightness level (0 to 15) */
    uint8_t blink_rate; /**< Blink frequency (0=Off, 1=2Hz, 2=1Hz, 3=0.5Hz) */
    bool int_enable;    /**< Enable INT pin output */
    bool int_act_high;  /**< True for Active-High, False for Active-Low */
} ht16k33_init_t;

/**
 * @brief Runtime Device Object for HT16K33.
 */
typedef struct
{
    iic_halt bus;            /**< I2C hardware handle */
    addr16_gt dev_addr;      /**< 7-bit device address */
    data_gt display_ram[16]; /**< Local cache for display memory (16x8 matrix) */
    bool is_dirty;           /**< Flag indicating if display_ram was modified */
} ht16k33_dev_t;

/**
 * @brief   Initialize the HT16K33 device.
 * Probes the device, starts the oscillator, and applies init settings.
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     bus       I2C hardware handle to attach.
 * @param[in]     dev_addr  7-bit device address.
 * @param[in]     init_cfg  Pointer to the initialization parameters.
 * @param[in]     timeout   Timeout for I2C operations.
 * * @return  ec_gt           Error code.
 */
ec_gt ht16k33_init(ht16k33_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const ht16k33_init_t* init_cfg,
                   time_gt timeout);

/**
 * @brief   Flush the local display RAM to the HT16K33 if is_dirty flag is set.
 * Automatically clears the is_dirty flag upon success.
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     timeout   Timeout for I2C operations.
 * * @return  ec_gt           Error code.
 */
ec_gt ht16k33_update_display(ht16k33_dev_t* dev, time_gt timeout);

/**
 * @brief   Scan and return the current pressed key ID.
 * * @param[in]  dev          Pointer to the device object.
 * @param[out] key_id_ret   Pointer to store the pressed key ID (0 if no key).
 * @param[in]  timeout      Timeout for I2C operations.
 * * @return  ec_gt           Error code.
 */
ec_gt ht16k33_read_keys(ht16k33_dev_t* dev, uint8_t* key_id_ret, time_gt timeout);

/**
 * @brief   Perform a full-screen display test (Turns on all LEDs).
 * * @param[in,out] dev       Pointer to the device object.
 * @param[in]     timeout   Timeout for I2C operations.
 * * @return  ec_gt           Error code.
 */
ec_gt ht16k33_test_all_leds_on(ht16k33_dev_t* dev, time_gt timeout);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* _GMP_DEV_HT16K33_H */
