/**
 * @file xplt.peripheral.c
 * @brief STM32C092 platform binding for UART DMA, TIM3, and the user LED.
 */

#include <gmp_core.h>

#include "main.h"
#include "user_main.h"
#include <xplt.peripheral.h>

#define XPLT_DL_UART_RX_DMA_SIZE 256U

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

static gmp_datalink_t* bound_datalink;
static byte_gt uart_rx_dma_buffer[XPLT_DL_UART_RX_DMA_SIZE];
static volatile uint16_t uart_rx_dma_position;
static gpio_model_stm32_t user_led_entity;
static gpio_halt user_led;

/** @brief DMA transmit phases for one split Data Link frame. */
typedef enum
{
    XPLT_DL_UART_TX_IDLE = 0,
    XPLT_DL_UART_TX_HEADER,
    XPLT_DL_UART_TX_PAYLOAD
} xplt_dl_uart_tx_phase_t;

static volatile xplt_dl_uart_tx_phase_t uart_tx_phase;

/** @brief Arm circular DMA with half, full, and UART-idle receive events. */
static void xplt_dl_arm_rx(void)
{
    uart_rx_dma_position = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_rx_dma_buffer,
                                      XPLT_DL_UART_RX_DMA_SIZE);
}

void setup_peripheral(void)
{
    debug_uart = &huart2;
    user_led_entity.gpio_port = LED1_GPIO_Port;
    user_led_entity.gpio_pin = LED1_Pin;
    user_led = &user_led_entity;
    bound_datalink = NULL;
    uart_tx_phase = XPLT_DL_UART_TX_IDLE;
    xplt_dl_arm_rx();
}

void xplt_dl_bind(gmp_datalink_t* datalink)
{
    bound_datalink = datalink;
}

void xplt_dl_start_tx(gmp_datalink_t* datalink)
{
    bound_datalink = datalink;
    uart_tx_phase = XPLT_DL_UART_TX_HEADER;
    if (HAL_UART_Transmit_DMA(
            &huart2, (const uint8_t*)gmp_dev_dl_get_tx_hw_hdr_ptr(datalink),
            (uint16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink)) != HAL_OK)
    {
        uart_tx_phase = XPLT_DL_UART_TX_IDLE;
        gmp_dev_dl_tx_state_done(datalink);
    }
}

void xplt_start_sample_timer(void)
{
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
        Error_Handler();
}

/**
 * @brief Toggle the NUCLEO user LED through the GMP STM32 GPIO CSP.
 * @return GMP task completion status.
 */
void xplt_toggle_user_led(void)
{
    static fast_gt state;
    state = !state;
    (void)gmp_hal_gpio_write(user_led, state);
}

/**
 * @brief Queue circular-DMA receive spans into the bound Data Link context.
 * @param uart UART handle reporting the receive event.
 * @param size Current DMA write position in protocol octets.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size)
{
    if (uart == &huart2 && bound_datalink != NULL)
    {
        uint16_t previous = uart_rx_dma_position;
        if (size > previous)
        {
            gmp_dev_dl_push_str(bound_datalink, &uart_rx_dma_buffer[previous],
                                (size_gt)(size - previous));
        }
        else if (size < previous)
        {
            if (previous < XPLT_DL_UART_RX_DMA_SIZE)
                gmp_dev_dl_push_str(bound_datalink, &uart_rx_dma_buffer[previous],
                                    (size_gt)(XPLT_DL_UART_RX_DMA_SIZE - previous));
            if (size > 0U)
                gmp_dev_dl_push_str(bound_datalink, uart_rx_dma_buffer, size);
        }
        uart_rx_dma_position = size;
    }
}

/** @brief Continue or complete the two-phase DMA frame transmission. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
    if (uart != &huart2 || bound_datalink == NULL)
        return;

    if (uart_tx_phase == XPLT_DL_UART_TX_HEADER &&
        gmp_dev_dl_get_tx_hw_pld_size(bound_datalink) > 0U)
    {
        uart_tx_phase = XPLT_DL_UART_TX_PAYLOAD;
        if (HAL_UART_Transmit_DMA(
                &huart2,
                (const uint8_t*)gmp_dev_dl_get_tx_hw_pld_ptr(bound_datalink),
                (uint16_t)gmp_dev_dl_get_tx_hw_pld_size(bound_datalink)) == HAL_OK)
            return;
    }

    uart_tx_phase = XPLT_DL_UART_TX_IDLE;
    gmp_dev_dl_tx_state_done(bound_datalink);
}

/** @brief Recover circular DMA reception after a UART overrun or line error. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
    if (uart == &huart2)
    {
        if (bound_datalink != NULL && uart_tx_phase != XPLT_DL_UART_TX_IDLE)
        {
            uart_tx_phase = XPLT_DL_UART_TX_IDLE;
            gmp_dev_dl_tx_state_done(bound_datalink);
        }
        (void)HAL_UART_AbortReceive(uart);
        xplt_dl_arm_rx();
    }
}

/** @brief Advance the user DSA application from the TIM3 update interrupt. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer)
{
    if (timer == &htim3)
        user_dsa_timer_step();
}
