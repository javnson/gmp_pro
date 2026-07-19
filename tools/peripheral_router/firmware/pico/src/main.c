/**
 * @file main.c
 * @brief Raspberry Pi Pico firmware for the GMP peripheral router.
 */

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "gmp_router_protocol.h"

static uint8_t packet_buffer[GMP_ROUTER_MAX_PACKET];
static uint8_t wire_buffer[GMP_ROUTER_MAX_WIRE_PACKET];
static uint8_t response_payload[GMP_ROUTER_MAX_PAYLOAD];

static uint16_t get_u16(const uint8_t* input)
{
    return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

static uint32_t get_u32(const uint8_t* input)
{
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

static int valid_channel(uint16_t channel, uint16_t count)
{
    return channel < count;
}

static int valid_pin(uint16_t pin)
{
    return pin < NUM_BANK0_GPIOS;
}

static int dispatch_system(const gmp_router_header_t* request, const uint8_t* payload,
                           uint16_t* response_length)
{
    char board_id[2U * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1U];
    size_t id_length;
    (void)payload;
    if (request->operation == GMP_ROUTER_SYSTEM_HELLO) {
        pico_get_unique_board_id_string(board_id, sizeof(board_id));
        id_length = strlen(board_id);
        response_payload[0] = GMP_ROUTER_VERSION;
        response_payload[1] = (uint8_t)NUM_BANK0_GPIOS;
        response_payload[2] = 2U;
        response_payload[3] = 2U;
        response_payload[4] = 2U;
        response_payload[5] = 0U; /* RP2040/RP2350 has no native CAN controller. */
        response_payload[6] = (uint8_t)id_length;
        memcpy(response_payload + 7U, board_id, id_length);
        *response_length = (uint16_t)(7U + id_length);
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_SYSTEM_PING) {
        memcpy(response_payload, payload, request->payload_length);
        *response_length = request->payload_length;
        return GMP_ROUTER_STATUS_OK;
    }
    return GMP_ROUTER_STATUS_UNSUPPORTED;
}

static int dispatch_gpio(const gmp_router_header_t* request, const uint8_t* payload,
                         uint16_t* response_length)
{
    if (!valid_pin(request->channel))
        return GMP_ROUTER_STATUS_INVALID;
    gpio_init(request->channel);
    if (request->operation == GMP_ROUTER_GPIO_CONFIGURE && request->payload_length == 2U) {
        gpio_set_dir(request->channel, payload[0] ? GPIO_OUT : GPIO_IN);
        gpio_disable_pulls(request->channel);
        if (payload[1] == 1U) gpio_pull_up(request->channel);
        else if (payload[1] == 2U) gpio_pull_down(request->channel);
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_GPIO_WRITE && request->payload_length == 1U) {
        gpio_put(request->channel, payload[0] != 0U);
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_GPIO_READ && request->payload_length == 0U) {
        response_payload[0] = gpio_get(request->channel) ? 1U : 0U;
        *response_length = 1U;
        return GMP_ROUTER_STATUS_OK;
    }
    return GMP_ROUTER_STATUS_INVALID;
}

static uart_inst_t* uart_instance(uint16_t channel)
{
    return channel == 0U ? uart0 : uart1;
}

static int dispatch_uart(const gmp_router_header_t* request, const uint8_t* payload,
                         uint16_t* response_length)
{
    uart_inst_t* uart;
    uint16_t requested;
    uint16_t count = 0U;
    if (!valid_channel(request->channel, 2U)) return GMP_ROUTER_STATUS_INVALID;
    uart = uart_instance(request->channel);
    if (request->operation == GMP_ROUTER_BUS_CONFIGURE && request->payload_length == 8U) {
        if (!valid_pin(get_u16(payload + 4U)) || !valid_pin(get_u16(payload + 6U)))
            return GMP_ROUTER_STATUS_INVALID;
        uart_init(uart, get_u32(payload));
        gpio_set_function(get_u16(payload + 4U), GPIO_FUNC_UART);
        gpio_set_function(get_u16(payload + 6U), GPIO_FUNC_UART);
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_BUS_WRITE) {
        uart_write_blocking(uart, payload, request->payload_length);
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_BUS_READ && request->payload_length == 2U) {
        requested = get_u16(payload);
        if (requested > GMP_ROUTER_MAX_PAYLOAD) return GMP_ROUTER_STATUS_INVALID;
        while (count < requested && uart_is_readable(uart))
            response_payload[count++] = (uint8_t)uart_getc(uart);
        *response_length = count;
        return GMP_ROUTER_STATUS_OK;
    }
    return GMP_ROUTER_STATUS_INVALID;
}

static i2c_inst_t* i2c_instance(uint16_t channel)
{
    return channel == 0U ? i2c0 : i2c1;
}

static int dispatch_i2c(const gmp_router_header_t* request, const uint8_t* payload,
                        uint16_t* response_length)
{
    i2c_inst_t* i2c;
    int result;
    uint16_t length;
    if (!valid_channel(request->channel, 2U)) return GMP_ROUTER_STATUS_INVALID;
    i2c = i2c_instance(request->channel);
    if (request->operation == GMP_ROUTER_BUS_CONFIGURE && request->payload_length == 8U) {
        if (!valid_pin(get_u16(payload + 4U)) || !valid_pin(get_u16(payload + 6U)))
            return GMP_ROUTER_STATUS_INVALID;
        i2c_init(i2c, get_u32(payload));
        gpio_set_function(get_u16(payload + 4U), GPIO_FUNC_I2C);
        gpio_set_function(get_u16(payload + 6U), GPIO_FUNC_I2C);
        gpio_pull_up(get_u16(payload + 4U));
        gpio_pull_up(get_u16(payload + 6U));
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_BUS_WRITE && request->payload_length >= 2U) {
        if (get_u16(payload) > 0x7FU) return GMP_ROUTER_STATUS_INVALID;
        result = i2c_write_blocking(i2c, get_u16(payload), payload + 2U,
                                    request->payload_length - 2U, false);
        return result < 0 ? GMP_ROUTER_STATUS_IO : GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_BUS_READ && request->payload_length == 4U) {
        if (get_u16(payload) > 0x7FU) return GMP_ROUTER_STATUS_INVALID;
        length = get_u16(payload + 2U);
        if (length > GMP_ROUTER_MAX_PAYLOAD) return GMP_ROUTER_STATUS_INVALID;
        result = i2c_read_blocking(i2c, get_u16(payload), response_payload, length, false);
        if (result < 0) return GMP_ROUTER_STATUS_IO;
        *response_length = (uint16_t)result;
        return GMP_ROUTER_STATUS_OK;
    }
    return GMP_ROUTER_STATUS_INVALID;
}

static spi_inst_t* spi_instance(uint16_t channel)
{
    return channel == 0U ? spi0 : spi1;
}

static int dispatch_spi(const gmp_router_header_t* request, const uint8_t* payload,
                        uint16_t* response_length)
{
    spi_inst_t* spi;
    uint16_t mode;
    if (!valid_channel(request->channel, 2U)) return GMP_ROUTER_STATUS_INVALID;
    spi = spi_instance(request->channel);
    if (request->operation == GMP_ROUTER_BUS_CONFIGURE && request->payload_length == 11U) {
        if (!valid_pin(get_u16(payload + 4U)) || !valid_pin(get_u16(payload + 6U)) ||
            !valid_pin(get_u16(payload + 8U)))
            return GMP_ROUTER_STATUS_INVALID;
        spi_init(spi, get_u32(payload));
        gpio_set_function(get_u16(payload + 4U), GPIO_FUNC_SPI);
        gpio_set_function(get_u16(payload + 6U), GPIO_FUNC_SPI);
        gpio_set_function(get_u16(payload + 8U), GPIO_FUNC_SPI);
        mode = payload[10U] & 3U;
        spi_set_format(spi, 8U, (mode & 2U) ? SPI_CPOL_1 : SPI_CPOL_0,
                       (mode & 1U) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
        return GMP_ROUTER_STATUS_OK;
    }
    if (request->operation == GMP_ROUTER_BUS_TRANSFER) {
        spi_write_read_blocking(spi, payload, response_payload, request->payload_length);
        *response_length = request->payload_length;
        return GMP_ROUTER_STATUS_OK;
    }
    return GMP_ROUTER_STATUS_INVALID;
}

static int dispatch(const gmp_router_header_t* request, const uint8_t* payload,
                    uint16_t* response_length)
{
    *response_length = 0U;
    switch (request->peripheral) {
    case GMP_ROUTER_PERIPHERAL_SYSTEM: return dispatch_system(request, payload, response_length);
    case GMP_ROUTER_PERIPHERAL_GPIO: return dispatch_gpio(request, payload, response_length);
    case GMP_ROUTER_PERIPHERAL_UART: return dispatch_uart(request, payload, response_length);
    case GMP_ROUTER_PERIPHERAL_I2C: return dispatch_i2c(request, payload, response_length);
    case GMP_ROUTER_PERIPHERAL_SPI: return dispatch_spi(request, payload, response_length);
    case GMP_ROUTER_PERIPHERAL_CAN: return GMP_ROUTER_STATUS_UNSUPPORTED;
    default: return GMP_ROUTER_STATUS_UNSUPPORTED;
    }
}

static void respond(const gmp_router_header_t* request, int status, uint16_t payload_length)
{
    gmp_router_header_t response = *request;
    size_t packet_length = 0U;
    size_t wire_length = 0U;
    response.message_type = GMP_ROUTER_MESSAGE_RESPONSE;
    response.status = (int16_t)status;
    response.payload_length = payload_length;
    if (gmp_router_packet_encode(&response, response_payload, packet_buffer,
                                 sizeof(packet_buffer), &packet_length) != GMP_ROUTER_STATUS_OK)
        return;
    if (gmp_router_cobs_encode(packet_buffer, packet_length, wire_buffer,
                               sizeof(wire_buffer) - 1U, &wire_length) != GMP_ROUTER_STATUS_OK)
        return;
    wire_buffer[wire_length++] = 0U;
    fwrite(wire_buffer, 1U, wire_length, stdout);
    fflush(stdout);
}

int main(void)
{
    size_t wire_length = 0U;
    stdio_init_all();
    while (true) {
        int value = getchar_timeout_us(1000U);
        if (value == PICO_ERROR_TIMEOUT) continue;
        if (value != 0) {
            if (wire_length < sizeof(wire_buffer)) wire_buffer[wire_length++] = (uint8_t)value;
            else wire_length = 0U;
            continue;
        }
        if (wire_length != 0U) {
            size_t packet_length;
            gmp_router_header_t request;
            const uint8_t* payload;
            if (gmp_router_cobs_decode(wire_buffer, wire_length, packet_buffer,
                                       sizeof(packet_buffer), &packet_length) == GMP_ROUTER_STATUS_OK &&
                gmp_router_packet_decode(packet_buffer, packet_length, &request, &payload) == GMP_ROUTER_STATUS_OK &&
                request.message_type == GMP_ROUTER_MESSAGE_REQUEST) {
                uint16_t response_length;
                int status = dispatch(&request, payload, &response_length);
                respond(&request, status, response_length);
            }
        }
        wire_length = 0U;
    }
}
