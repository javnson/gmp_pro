/**
 * @file main.c
 * @brief GMP integration example for Raspberry Pi Pico and Pico 2 boards.
 *
 * The example blinks the board LED through the GMP GPIO API and echoes bytes
 * received on UART0. It also initializes I2C0 and SPI0 so GMP device drivers
 * can use those bus handles directly.
 */

#include <gmp_core.h>

#define DEMO_UART_TX_PIN 0U
#define DEMO_UART_RX_PIN 1U
#define DEMO_I2C_SDA_PIN 4U
#define DEMO_I2C_SCL_PIN 5U
#define DEMO_SPI_RX_PIN  16U
#define DEMO_SPI_CS_PIN  17U
#define DEMO_SPI_SCK_PIN 18U
#define DEMO_SPI_TX_PIN  19U

#define DEMO_UART_BAUD_RATE 115200U
#define DEMO_I2C_BAUD_RATE  400000U
#define DEMO_SPI_BAUD_RATE  1000000U

static time_gt g_last_blink_tick;
static fast_gt g_led_level;

/**
 * @brief Initialize the board peripherals used by the example.
 */
void setup_peripheral(void)
{
    uart_init(uart0, DEMO_UART_BAUD_RATE);
    gpio_set_function(DEMO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DEMO_UART_RX_PIN, GPIO_FUNC_UART);
    debug_uart = uart0;

    i2c_init(i2c0, DEMO_I2C_BAUD_RATE);
    gpio_set_function(DEMO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(DEMO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(DEMO_I2C_SDA_PIN);
    gpio_pull_up(DEMO_I2C_SCL_PIN);

    spi_init(spi0, DEMO_SPI_BAUD_RATE);
    gpio_set_function(DEMO_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DEMO_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DEMO_SPI_TX_PIN, GPIO_FUNC_SPI);

    (void)gmp_hal_gpio_set_dir(GMP_RPI_PICO_GPIO(DEMO_SPI_CS_PIN), GMP_HAL_GPIO_DIR_OUT);
    (void)gmp_hal_gpio_write(GMP_RPI_PICO_GPIO(DEMO_SPI_CS_PIN), GMP_HAL_GPIO_HIGH);

#ifdef PICO_DEFAULT_LED_PIN
    (void)gmp_hal_gpio_set_dir(GMP_RPI_PICO_GPIO(PICO_DEFAULT_LED_PIN), GMP_HAL_GPIO_DIR_OUT);
#endif
}

/**
 * @brief Initialize the application state after peripheral setup.
 */
void init(void)
{
    static const data_gt banner[] = "GMP Raspberry Pi Pico example ready\r\n";
    g_last_blink_tick = gmp_base_get_system_tick();
    g_led_level = GMP_HAL_GPIO_LOW;
    (void)gmp_hal_uart_write(debug_uart, banner, sizeof(banner) - 1U, 100U);
}

/**
 * @brief Run one non-blocking iteration of the example application.
 */
void mainloop(void)
{
    time_gt now = gmp_base_get_system_tick();
    if ((now - g_last_blink_tick) >= 500U)
    {
        g_last_blink_tick = now;
        g_led_level = g_led_level == GMP_HAL_GPIO_LOW ? GMP_HAL_GPIO_HIGH : GMP_HAL_GPIO_LOW;
#ifdef PICO_DEFAULT_LED_PIN
        (void)gmp_hal_gpio_write(GMP_RPI_PICO_GPIO(PICO_DEFAULT_LED_PIN), g_led_level);
#endif
    }

    if (gmp_hal_uart_get_rx_available(debug_uart) != 0U)
    {
        data_gt byte;
        size_gt bytes_read;
        if (gmp_hal_uart_read(debug_uart, &byte, 1U, 1U, &bytes_read) == GMP_EC_OK && bytes_read == 1U)
        {
            (void)gmp_hal_uart_write(debug_uart, &byte, 1U, 1U);
        }
    }
}

/**
 * @brief Enter the GMP application lifecycle.
 * @return This function does not return during normal operation.
 */
int main(void)
{
    gmp_base_entry();
    return 0;
}
