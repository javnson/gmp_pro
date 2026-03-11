/**
 * @file    ht16k33.h
 * @brief   High-performance, Hardware-agnostic driver for HT16K33.
 */

#ifndef HT16K33_H
#define HT16K33_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/* ==================== CONFIGURATION MACROS =============================== */
/* ========================================================================= */

/** * @brief Default I2C timeout in milliseconds. 
 * Note: 16 bytes at 100kHz takes ~1.8ms. 5ms is a safe default.
 */
#ifndef HT16K33_CFG_TIMEOUT
#define HT16K33_CFG_TIMEOUT (5U)
#endif

/** * @brief Number of bytes used for display RAM.
 * Reduce this value if your hardware uses fewer digits to save I2C bandwidth.
 * Maximum: 16 (for 16x8 matrix).
 */
#ifndef HT16K33_CFG_DISP_RAM_SIZE
#define HT16K33_CFG_DISP_RAM_SIZE (16U)
#endif

/** * @brief Number of bytes used for key scanning RAM.
 * Reduce this value if your hardware uses fewer key rows.
 * Maximum: 6 (for 13x3 matrix, KS0~KS2).
 */
#ifndef HT16K33_CFG_KEY_RAM_SIZE
#define HT16K33_CFG_KEY_RAM_SIZE (6U)
#endif

/* Compile-time bounds checking for configurations */
#if (HT16K33_CFG_DISP_RAM_SIZE == 0) || (HT16K33_CFG_DISP_RAM_SIZE > 16)
#error "HT16K33_CFG_DISP_RAM_SIZE must be between 1 and 16."
#endif

#if (HT16K33_CFG_KEY_RAM_SIZE == 0) || (HT16K33_CFG_KEY_RAM_SIZE > 6)
#error "HT16K33_CFG_KEY_RAM_SIZE must be between 1 and 6."
#endif

/* ========================================================================= */
/* ==================== REGISTERS & COMMANDS =============================== */
/* ========================================================================= */

#define HT16K33_REG_SYSTEM_SETUP  0x20
#define HT16K33_REG_DISPLAY_SETUP 0x80
#define HT16K33_REG_ROWINT_SET    0xA0
#define HT16K33_REG_BRIGHTNESS    0xE0
#define HT16K33_REG_KEY_DATA_ADDR 0x40

#define HT16K33_CMD_OSC_ON     (HT16K33_REG_SYSTEM_SETUP | 0x01)
#define HT16K33_CMD_DISPLAY_ON (HT16K33_REG_DISPLAY_SETUP | 0x01)

/* ========================================================================= */
/* ==================== DATA STRUCTURES ==================================== */
/* ========================================================================= */

/**
 * @brief Initialization parameters for HT16K33.
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
 * Automatically scaled down by the CFG_DISP_RAM_SIZE macro to save memory.
 */
typedef struct
{
    iic_halt bus;
    addr16_gt dev_addr;
    data_gt display_ram[HT16K33_CFG_DISP_RAM_SIZE];
    bool is_dirty;
} ht16k33_dev_t;

/* ========================================================================= */
/* ==================== API FUNCTIONS ====================================== */
/* ========================================================================= */

ec_gt ht16k33_init(ht16k33_dev_t* dev, iic_halt bus, addr16_gt dev_addr, const ht16k33_init_t* init_cfg);
ec_gt ht16k33_update_display(ht16k33_dev_t* dev);
ec_gt ht16k33_read_keys(ht16k33_dev_t* dev, uint8_t* key_id_ret);
ec_gt ht16k33_test_all_leds_on(ht16k33_dev_t* dev);

#ifdef __cplusplus
}
#endif

#endif /* HT16K33_H */
