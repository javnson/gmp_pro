
#include <gmp_core.h>

#include <oled_driver.h>
#include <oledfont.h>

#include <core/pm/function_scheduler.h>


/**
 * @brief  Re-implementation of official OLED_WR_Byte using your custom HAL.
 * @param  dat: The actual byte of command or pixel data to be transmitted.
 * @param  cmd: 0 for Command (OLED_CMD), 1 for Pixel Data (OLED_DATA).
 */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    /* Use an explicit tick timeout value for C2000 blocking execution */
    const time_gt timeout_ticks = 200;
    uint32_t control_byte = (cmd != 0) ? 0x40 : 0x00;

    /*
     * dev_addr:  OLED 7-bit address
     * reg_addr:  Control Byte (0x00 or 0x40), length = 1 byte
     * reg_data:  The actual instruction or graphic payload, length = 1 byte
     */
    gmp_hal_iic_write_reg(iic_bus, OLED_IIC_7BIT_ADDR, control_byte, 1, (uint32_t)dat, 1, timeout_ticks);
}

/**
 * @brief  Optimized, low-overhead positional command function.
 * @note   Combines page and column positioning into a single I2C burst transaction.
 */
void oled_set_position(uint8_t x, uint8_t y_page)
{
    const time_gt timeout_ticks = 2000;
    static data_gt pos_cmds[4];

    pos_cmds[0] = 0x00;                           /* Control Byte: Following are commands */
    pos_cmds[1] = (data_gt)(0xB0 + y_page);       /* Set Target Page Address */
    pos_cmds[2] = (data_gt)((((x + 2) & 0xF0) >> 4) | 0x10); /* Higher column nibble with +2 offset */
    pos_cmds[3] = (data_gt)((x + 2) & 0x0F);       /* Lower column nibble with +2 offset */

    /* Dispatch all 4 bytes consecutively to save bus time slices */
    gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, pos_cmds, 4, timeout_ticks);
}


/**
 * @brief  Turns ON the OLED panel and internal charge pump in a single I2C transaction.
 */
void oled_display_on(void)
{
    const time_gt timeout_ticks = 2000;
    static data_gt on_cmds[4];

    on_cmds[0] = 0x00; /* Control Byte: Following are commands */
    on_cmds[1] = 0x8D; /* Charge Pump Command Specifier */
    on_cmds[2] = 0x14; /* Enable Charge Pump */
    on_cmds[3] = 0xAF; /* Display ON */

    gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, on_cmds, 4, timeout_ticks);
}

/**
 * @brief  Turns OFF the OLED panel and internal charge pump cleanly.
 */
void oled_display_off(void)
{
    const time_gt timeout_ticks = 2000;
    static data_gt off_cmds[4];

    off_cmds[0] = 0x00; /* Control Byte: Following are commands */
    off_cmds[1] = 0x8D; /* Charge Pump Command Specifier */
    off_cmds[2] = 0x10; /* Disable Charge Pump */
    off_cmds[3] = 0xAE; /* Display OFF */

    gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, off_cmds, 4, timeout_ticks);
}

/**
 * @brief  Optimized ultra-fast, non-blocking-friendly OLED clear function.
 * @note   Replaces 1024 separate I2C bursts with 8 continuous page bursts.
 */
void oled_clear(void)
{
    const time_gt timeout_ticks = 2000;

    /* 1. Pack commands into a single packet to target Page and Column addresses */
    static data_gt page_cmd[4];
    page_cmd[0] = 0x00; /* Control Byte: Following are commands */
    page_cmd[2] = 0x02; /* Lower Column Start Address (Matching official 0x02) */
    page_cmd[3] = 0x10; /* Higher Column Start Address (Matching official 0x10) */

    /* 2. Allocate a page buffer on DSP containing 1 Control Byte + 128 Data Bytes (all zeros) */
    static data_gt blank_page[129];
    blank_page[0] = 0x40; /* Control Byte: Following is graphic streaming data */

    uint16_t idx;

    for (idx = 1; idx <= 128; idx++)
    {
        blank_page[idx] = 0; /* Turn off all pixels */
    }

    /* 3. Loop through 8 independent pages */
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        page_cmd[1] = (data_gt)(0xb0 + i); /* Update targeting Page index */

        /* Send 4 bytes of positioning commands */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, page_cmd, 4, timeout_ticks);

        /* Stream 129 bytes of zero data into the whole targeted GDDRAM page at once */
        /* C2000 Hardware FIFO will automatically stream this with zero CPU delays */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, blank_page, 129, timeout_ticks);
    }
}

/**
 * @brief  Ultra-efficient character rendering block without inner loop I2C overhead.
 * @param  x: Horizontal start column index (0 to 127).
 * @param  y_page: Vertical page index (0 to 7).
 * @param  chr: ASCII character to display.
 */
void oled_show_char(uint8_t x, uint8_t y_page, uint8_t chr)
{
    uint16_t i;

    const time_gt timeout_ticks = 2000;
    uint8_t c_offset = chr - ' '; /* Calculate ASCII matrix offset index */

    /*
     * Temporary transmit buffer for the data stream.
     * Element [0] is reserved for the 0x40 Data Control Byte.
     * Max elements needed: 1 (Control) + 8 (Pixels) = 9.
     */
    static data_gt tx_payload[9];
    tx_payload[0] = 0x40; /* Control Byte: Following stream is graphic display RAM data */

    /* Auto wrap text boundaries just like your official code */
    if (x > 128 - 1)
    {
        x = 0;
        y_page = y_page + 2;
    }

    if (FONT_SIZE == 16)
    {
        uint16_t font_index = (uint16_t)c_offset * 16U;

        /* 1. Render Upper Half (8 pixels high, 8 pixels wide) */
        oled_set_position(x, y_page);
        for (i = 0; i < 8; i++)
        {
            tx_payload[i + 1] = (data_gt)F8X16[font_index + i];
        }
        /* Continuous push of 1 control byte + 8 data bytes via C2000 FIFO */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, tx_payload, 9, timeout_ticks);

        /* 2. Render Lower Half (8 pixels high, 8 pixels wide) at the next vertical page slot */
        oled_set_position(x, y_page + 1);
        for (i = 0; i < 8; i++)
        {
            tx_payload[i + 1] = (data_gt)F8X16[font_index + i + 8];
        }
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, tx_payload, 9, timeout_ticks);
    }
    else
    {
        /* Render Small Font (8 pixels high, 6 pixels wide) */
        oled_set_position(x, y_page);
        for (i = 0; i < 6; i++)
        {
            tx_payload[i + 1] = (data_gt)F6x8[c_offset][i];
        }
        /* Continuous push of 1 control byte + 6 data bytes */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, tx_payload, 7, timeout_ticks);
    }
}

/**
 * @brief  Displays a null-terminated string on the character grid.
 * @note   Leverages the highly optimized non-blocking oled_show_char underneath.
 */
void oled_show_str(uint8_t x, uint8_t y_page, const char *str)
{
    uint16_t j = 0;

    while (str[j] != '\0')
    {
        oled_show_char(x, y_page, (uint8_t)str[j]);

        /* Advance x by 8 pixels (width of F8X16 or font boundary spacing) */
        x += 8;
        if (x > 120)
        {
            x = 0;
            y_page += 2; /* Move down 2 pages for the 16-pixel high font height wrap */
        }
        j++;
    }
}

/**
 * @brief  Draws a BMP image using highly efficient page-burst transfers instead of byte-by-byte streaming.
 * @note   Optimized for C2000 FIFO architecture. Suitable for block initialization displays.
 * @param  x0: Starting column coordinate (0 to 127).
 * @param  y0: Starting page coordinate (0 to 7).
 * @param  x1: Ending column coordinate (1 to 128).
 * @param  y1: Ending page coordinate (1 to 8).
 * @param  BMP: Array containing the raw monochrome picture dot matrix data.
 */
void oled_show_bmp(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    const time_gt timeout_ticks = 2000;
    unsigned int bmp_idx = 0;
    unsigned char y_page;
    uint16_t x;

    /*
     * Temporary continuous transfer buffer.
     * Element [0] is reserved for the 0x40 Data Control Byte.
     * Maximum payload size: 1 (Control) + 128 (Max columns) = 129 words.
     */
    static data_gt page_payload[129];
    page_payload[0] = 0x40; /* Control Byte: Following stream is graphic display RAM data */

    /* Calculate horizontal segment width per burst */
    uint16_t chunk_width = (uint16_t)(x1 - x0);
    if (chunk_width > 128) chunk_width = 128;

    /* Loop through each targeted vertical page row sequentially */
    for (y_page = y0; y_page < y1; y_page++)
    {
        /* 1. Set the physical column and page address pointers on the OLED controller */
        oled_set_position(x0, y_page);

        /* 2. Assemble the current page row's pixel dataset into our linear buffer */
        for (x = 0; x < chunk_width; x++)
        {
            page_payload[x + 1] = (data_gt)BMP[bmp_idx++];
        }

        /*
         * 3. Send the entire line block in a single fast continuous I2C burst.
         * Total length = 1 control byte + chunk_width data bytes.
         * Your gmp_hal_iic_write_mem will manage the 16-byte hardware FIFO internally without stalls.
         */
        gmp_hal_iic_write_mem(iic_bus, OLED_IIC_7BIT_ADDR, 0, 0, page_payload, chunk_width + 1, timeout_ticks);
    }
}

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
void oled_init(void)
{
    OLED_WR_Byte(0xAE, OLED_CMD); //--turn  oled panel
    OLED_WR_Byte(0x02, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8, OLED_CMD); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset   Shift Mapping RAM Counter (0x00~0x3F)

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0x00, OLED_CMD); //-not offset
    OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0x40, OLED_CMD); //Set VCOM Deselect Level
    OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD); //
    OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
    OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
    OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel

    DEVICE_DELAY_US(200U);

    OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
    oled_clear();
    oled_set_position(0, 0);

    oled_show_str(0,0,"OLED TEST");
    oled_show_str(0,4,"2026/07/01");
}

