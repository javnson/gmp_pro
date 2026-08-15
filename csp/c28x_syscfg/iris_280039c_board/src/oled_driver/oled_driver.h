

#ifndef _FILE_OLED_DRIVER_H_
#define _FILE_OLED_DRIVER_H_

/* Your macro for the 7-bit slave address (e.g., 0x3C) */
#define OLED_IIC_7BIT_ADDR (0x78>>1)

/* Global IIC bus handle declared in your repository */
extern iic_halt iic_bus;

/**
 * @brief  Optimized, low-overhead positional command function.
 * @note   Combines page and column positioning into a single I2C burst transaction.
 */
void oled_set_position(uint8_t x, uint8_t y_page);

/**
 * @brief  Turns ON the OLED panel and internal charge pump in a single I2C transaction.
 */
void oled_display_on(void);

/**
 * @brief  Turns OFF the OLED panel and internal charge pump cleanly.
 */
void oled_display_off(void);

/**
 * @brief  Optimized ultra-fast, non-blocking-friendly OLED clear function.
 * @note   Replaces 1024 separate I2C bursts with 8 continuous page bursts.
 */
void oled_clear(void);

/**
 * @brief  Ultra-efficient character rendering block without inner loop I2C overhead.
 * @param  x: Horizontal start column index (0 to 127).
 * @param  y_page: Vertical page index (0 to 7).
 * @param  chr: ASCII character to display.
 */
void oled_show_char(uint8_t x, uint8_t y_page, uint8_t chr);

/**
 * @brief  Displays a null-terminated string on the character grid.
 * @note   Leverages the highly optimized non-blocking oled_show_char underneath.
 */
void oled_show_str(uint8_t x, uint8_t y_page, const char *str);

/**
 * @brief  Draws a BMP image using highly efficient page-burst transfers instead of byte-by-byte streaming.
 * @note   Optimized for C2000 FIFO architecture. Suitable for block initialization displays.
 * @param  x0: Starting column coordinate (0 to 127).
 * @param  y0: Starting page coordinate (0 to 7).
 * @param  x1: Ending column coordinate (1 to 128).
 * @param  y1: Ending page coordinate (1 to 8).
 * @param  BMP: Array containing the raw monochrome picture dot matrix data.
 */
void oled_show_bmp(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[]);

/**
 * @brief  Initializes the SSD1306/SH1106 OLED module controller registers.
 * @details This function transmits a pre-defined hardware configuration macro sequence
 *          to setup internal clocks, multiplex ratio, display flip directions, and
 *          crucially activates the internal charge pump required to drive the panel VCC.
 *          It forces Page Addressing Mode and performs a clear screen layout at the end.
 *
 * @param  None
 * @return None
 *
 * @note   Ensure the physical hardware supply is connected to 5V prior to running this
 *         sequence to avoid sub-optimal low voltage NACK lockouts from the display controller.
 * @see    OLED_WR_Byte
 * @see    oled_clear
 * @see    oled_set_position
 */
void oled_init(void);

#define OLED_CMD  0 //Ð´ÃüÁî
#define OLED_DATA 1 //Ð´Êý¾Ý
#define OLED_MODE 0

#define FONT_SIZE 8

#endif // _FILE_OLED_DRIVER_H_

