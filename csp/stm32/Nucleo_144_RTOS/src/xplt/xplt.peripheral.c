/**
 * @file xplt.peripheral.c
 * @brief RTOS-safe H753 peripheral preparation and post-init activation.
 */

#include <gmp_core.h>

#include "main.h"
#include "user_dl.h"
#include <xplt.peripheral.h>

#if GMP_NUCLEO_ENABLE_CONTROL
extern ADC_HandleTypeDef GMP_NUCLEO_ADC_PRIMARY_SYMBOL;
#if !GMP_NUCLEO_ADC_REGULAR_DMA
extern ADC_HandleTypeDef GMP_NUCLEO_ADC_SECONDARY_SYMBOL;
#endif
extern TIM_HandleTypeDef GMP_NUCLEO_PWM_TIMER_SYMBOL;
extern TIM_HandleTypeDef GMP_NUCLEO_QEP_TIMER_SYMBOL;
#if GMP_NUCLEO_ADC_TRIGGER_BRIDGE
extern TIM_HandleTypeDef GMP_NUCLEO_ADC_TRIGGER_TIMER_SYMBOL;
#endif
#endif
#if GMP_NUCLEO_ENABLE_UART_DL
extern UART_HandleTypeDef GMP_NUCLEO_DL_UART_SYMBOL;
#endif
#if GMP_NUCLEO_HAS_DAC
extern DAC_HandleTypeDef GMP_NUCLEO_DAC_SYMBOL;
#endif

#define XPLT_PWM_CCER_MASK                                                   \
    (TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |     \
     TIM_CCER_CC3E | TIM_CCER_CC3NE)

#if GMP_NUCLEO_ENABLE_UART_DL
static gmp_datalink_t* uart_datalink;
static byte_gt uart_rx_dma_buffer[GMP_NUCLEO_DL_RX_BUFFER_SIZE];
static volatile uint16_t uart_rx_dma_position;
#endif
#if GMP_NUCLEO_ENABLE_STATUS_LED
static volatile fast_gt status_led_on;
#endif
#if GMP_NUCLEO_ENABLE_CONTROL && GMP_NUCLEO_ADC_REGULAR_DMA
static uint16_t adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB_COUNT];
static fast_gt control_started;
#endif
volatile uint32_t gmp_nucleo_platform_diag[18];

#if GMP_NUCLEO_ENABLE_UART_DL
typedef enum
{
    XPLT_DL_UART_TX_IDLE = 0,
    XPLT_DL_UART_TX_HEADER,
    XPLT_DL_UART_TX_PAYLOAD
} xplt_dl_uart_tx_phase_t;

static volatile xplt_dl_uart_tx_phase_t uart_tx_phase;

static void xplt_dl_arm_rx(void)
{
    uart_rx_dma_position = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(
            GMP_NUCLEO_DL_UART_HANDLE, uart_rx_dma_buffer,
            (uint16_t)GMP_NUCLEO_DL_RX_BUFFER_SIZE) != HAL_OK)
        Error_Handler();
}
#endif

#if GMP_NUCLEO_ENABLE_CONTROL && !GMP_NUCLEO_ADC_REGULAR_DMA
static void xplt_select_adc_trigger(ADC_HandleTypeDef* adc)
{
    MODIFY_REG(adc->Instance->JSQR, GMP_NUCLEO_ADC_TRIGGER_MASK,
               GMP_NUCLEO_PWM_ADC_TRIGGER | GMP_NUCLEO_ADC_TRIGGER_EDGE);
}
#endif

void setup_peripheral(void)
{
    size_gt index;

    for (index = 0U;
         index < sizeof(gmp_nucleo_platform_diag) /
                     sizeof(gmp_nucleo_platform_diag[0]);
         ++index)
        gmp_nucleo_platform_diag[index] = 0U;
#if GMP_NUCLEO_ENABLE_UART_DL
    debug_uart = GMP_NUCLEO_DL_UART_HANDLE;
    debug_uart->Init.BaudRate = GMP_NUCLEO_DL_BAUD_RATE;
    if (HAL_UART_Init(debug_uart) != HAL_OK)
        Error_Handler();

    uart_datalink = NULL;
    uart_tx_phase = XPLT_DL_UART_TX_IDLE;
    xplt_dl_arm_rx();
#endif

#if GMP_NUCLEO_ENABLE_STATUS_LED
    GPIO_InitTypeDef led_gpio = {0};

    /* Nucleo-144 user LEDs are on GPIOB and, on newer boards, GPIOE.  Keep
       this initialization here as well as in generated code so each H755
       core can own and start its status LED independently. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    led_gpio.Pin = GMP_NUCLEO_STATUS_LED_PIN;
    led_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    led_gpio.Pull = GPIO_NOPULL;
    led_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GMP_NUCLEO_STATUS_LED_PORT, &led_gpio);
    HAL_GPIO_WritePin(GMP_NUCLEO_STATUS_LED_PORT, GMP_NUCLEO_STATUS_LED_PIN,
                      GMP_NUCLEO_STATUS_LED_OFF);
#if GMP_NUCLEO_DUAL_CORE && defined(CORE_CM7)
    led_gpio.Pin = GMP_NUCLEO_ENTITY(_STATUS_LED3_PIN);
    HAL_GPIO_Init(GMP_NUCLEO_ENTITY(_STATUS_LED3_PORT), &led_gpio);
    HAL_GPIO_WritePin(GMP_NUCLEO_ENTITY(_STATUS_LED3_PORT),
                      GMP_NUCLEO_ENTITY(_STATUS_LED3_PIN),
                      GMP_NUCLEO_ENTITY(_STATUS_LED3_OFF));
#endif
    status_led_on = 0;
#endif

#if GMP_NUCLEO_ENABLE_CONTROL
    control_started = 0;
    xplt_pwm_disable();
    __HAL_TIM_SET_AUTORELOAD(GMP_NUCLEO_PWM_TIMER_HANDLE,
                             GMP_NUCLEO_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(GMP_NUCLEO_PWM_TIMER_HANDLE, TIM_CHANNEL_4,
                          GMP_NUCLEO_PWM_PERIOD / 2U);
    SET_BIT(GMP_NUCLEO_PWM_TIMER_HANDLE->Instance->CCER, TIM_CCER_CC4E);
    if (HAL_TIM_Encoder_Start(GMP_NUCLEO_QEP_TIMER_HANDLE,
                              TIM_CHANNEL_ALL) != HAL_OK)
        Error_Handler();

#if GMP_NUCLEO_ADC_REGULAR_DMA
#if GMP_NUCLEO_ADC_HAS_CALIBRATION
#if GMP_NUCLEO_ADC_CALIBRATION_STYLE == 1
    if (HAL_ADCEx_Calibration_Start(GMP_NUCLEO_ADC_PRIMARY_HANDLE) != HAL_OK)
#elif GMP_NUCLEO_ADC_CALIBRATION_STYLE == 2
    if (HAL_ADCEx_Calibration_Start(GMP_NUCLEO_ADC_PRIMARY_HANDLE,
                                    ADC_SINGLE_ENDED) != HAL_OK)
#elif GMP_NUCLEO_ADC_CALIBRATION_STYLE == 3
    if (HAL_ADCEx_Calibration_Start(GMP_NUCLEO_ADC_PRIMARY_HANDLE,
                                    ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK)
#else
#error "Unsupported GMP_NUCLEO_ADC_CALIBRATION_STYLE"
#endif
        Error_Handler();
#endif
#else
    if (HAL_ADCEx_Calibration_Start(GMP_NUCLEO_ADC_SECONDARY_HANDLE,
                                    ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(GMP_NUCLEO_ADC_PRIMARY_HANDLE,
                                    ADC_SINGLE_ENDED) != HAL_OK)
        Error_Handler();
    xplt_select_adc_trigger(GMP_NUCLEO_ADC_SECONDARY_HANDLE);
    xplt_select_adc_trigger(GMP_NUCLEO_ADC_PRIMARY_HANDLE);
    if (HAL_ADCEx_InjectedStart(GMP_NUCLEO_ADC_SECONDARY_HANDLE) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(GMP_NUCLEO_ADC_PRIMARY_HANDLE) != HAL_OK)
        Error_Handler();
#endif

#if GMP_NUCLEO_ADC_TRIGGER_BRIDGE
    if (HAL_TIM_Base_Start(GMP_NUCLEO_ADC_TRIGGER_TIMER_HANDLE) != HAL_OK)
        Error_Handler();
#endif
#if GMP_NUCLEO_CAN_HAS_STBY
    HAL_GPIO_WritePin(GMP_NUCLEO_CAN_STBY_PORT, GMP_NUCLEO_CAN_STBY_PIN,
                      GMP_NUCLEO_CAN_STBY_OFF);
#endif

#if GMP_NUCLEO_HAS_DAC
    if (HAL_DAC_Start(GMP_NUCLEO_DAC_HANDLE, GMP_NUCLEO_DAC_CHANNEL) != HAL_OK)
        Error_Handler();
#endif
#endif
}

void gmp_csp_post_process(void)
{
#if GMP_NUCLEO_ENABLE_CONTROL && GMP_NUCLEO_ADC_REGULAR_DMA
    if (control_started)
        return;

    /* gmp_base_prepare() has completed ctl_init() and init() before this hook.
       Only now may the ADC DMA and its 20 kHz control interrupt begin. */
    if (HAL_ADC_Start_DMA(GMP_NUCLEO_ADC_PRIMARY_HANDLE,
                          (uint32_t*)adc_regular_dma_buffer,
                          GMP_NUCLEO_ADC_FB_COUNT) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_Base_Start(GMP_NUCLEO_PWM_TIMER_HANDLE) != HAL_OK)
        Error_Handler();
    control_started = 1;
#endif
}

#if GMP_NUCLEO_ENABLE_CONTROL
void xplt_pwm_enable(void)
{
    TIM_HandleTypeDef* timer = GMP_NUCLEO_PWM_TIMER_HANDLE;

    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, gmp_nucleo_pwm_compare[0]);
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, gmp_nucleo_pwm_compare[1]);
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, gmp_nucleo_pwm_compare[2]);
    gmp_base_enter_critical();
    SET_BIT(timer->Instance->CCER, XPLT_PWM_CCER_MASK);
    SET_BIT(timer->Instance->BDTR, TIM_BDTR_MOE);
    gmp_nucleo_platform_diag[1] =
        timer->Instance->CCER & XPLT_PWM_CCER_MASK;
    if ((timer->Instance->BDTR & TIM_BDTR_MOE) != 0U)
        gmp_nucleo_platform_diag[1] |= 0x80000000UL;
    gmp_base_leave_critical();
}

void xplt_pwm_disable(void)
{
    TIM_HandleTypeDef* timer = GMP_NUCLEO_PWM_TIMER_HANDLE;

    gmp_base_enter_critical();
    CLEAR_BIT(timer->Instance->BDTR, TIM_BDTR_MOE);
    CLEAR_BIT(timer->Instance->CCER, XPLT_PWM_CCER_MASK);
    gmp_nucleo_platform_diag[1] = 0U;
    gmp_base_leave_critical();
}
#endif

#if GMP_NUCLEO_ENABLE_STATUS_LED
void xplt_toggle_status_led(void)
{
    status_led_on = !status_led_on;
    gmp_nucleo_platform_diag[5] = (uint32_t)status_led_on;
    gmp_nucleo_platform_diag[6]++;
    HAL_GPIO_WritePin(GMP_NUCLEO_STATUS_LED_PORT, GMP_NUCLEO_STATUS_LED_PIN,
                      status_led_on ? GMP_NUCLEO_STATUS_LED_ON
                                    : GMP_NUCLEO_STATUS_LED_OFF);
}
#endif

#if GMP_NUCLEO_ENABLE_CONTROL
static uint32_t xplt_limit_pwm_compare(uint32_t compare)
{
    return compare <= (uint32_t)GMP_NUCLEO_PWM_PERIOD
               ? compare
               : (uint32_t)GMP_NUCLEO_PWM_PERIOD;
}

void xplt_ctl_input(void)
{
#if GMP_NUCLEO_ADC_REGULAR_DMA
    gmp_nucleo_adc_raw[0] = adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB0_RANK];
    gmp_nucleo_adc_raw[1] = adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB1_RANK];
    gmp_nucleo_adc_raw[2] = adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB2_RANK];
    gmp_nucleo_adc_raw[3] = adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB3_RANK];
    gmp_nucleo_adc_raw[4] = adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB4_RANK];
    gmp_nucleo_adc_raw[5] = adc_regular_dma_buffer[GMP_NUCLEO_ADC_FB5_RANK];
#else
    gmp_nucleo_adc_raw[0] = HAL_ADCEx_InjectedGetValue(
        GMP_NUCLEO_ADC_FB0_HANDLE, GMP_NUCLEO_ADC_FB0_RANK);
    gmp_nucleo_adc_raw[1] = HAL_ADCEx_InjectedGetValue(
        GMP_NUCLEO_ADC_FB1_HANDLE, GMP_NUCLEO_ADC_FB1_RANK);
    gmp_nucleo_adc_raw[2] = HAL_ADCEx_InjectedGetValue(
        GMP_NUCLEO_ADC_FB2_HANDLE, GMP_NUCLEO_ADC_FB2_RANK);
    gmp_nucleo_adc_raw[3] = HAL_ADCEx_InjectedGetValue(
        GMP_NUCLEO_ADC_FB3_HANDLE, GMP_NUCLEO_ADC_FB3_RANK);
    gmp_nucleo_adc_raw[4] = HAL_ADCEx_InjectedGetValue(
        GMP_NUCLEO_ADC_FB4_HANDLE, GMP_NUCLEO_ADC_FB4_RANK);
    gmp_nucleo_adc_raw[5] = HAL_ADCEx_InjectedGetValue(
        GMP_NUCLEO_ADC_FB5_HANDLE, GMP_NUCLEO_ADC_FB5_RANK);
#endif
    gmp_nucleo_qep_count =
        (int32_t)__HAL_TIM_GET_COUNTER(GMP_NUCLEO_QEP_TIMER_HANDLE);
}

void xplt_ctl_output(void)
{
    __HAL_TIM_SET_COMPARE(GMP_NUCLEO_PWM_TIMER_HANDLE, TIM_CHANNEL_1,
                          xplt_limit_pwm_compare(gmp_nucleo_pwm_compare[0]));
    __HAL_TIM_SET_COMPARE(GMP_NUCLEO_PWM_TIMER_HANDLE, TIM_CHANNEL_2,
                          xplt_limit_pwm_compare(gmp_nucleo_pwm_compare[1]));
    __HAL_TIM_SET_COMPARE(GMP_NUCLEO_PWM_TIMER_HANDLE, TIM_CHANNEL_3,
                          xplt_limit_pwm_compare(gmp_nucleo_pwm_compare[2]));
}
#endif

#if GMP_NUCLEO_ENABLE_UART_DL
void xplt_uart_dl_bind(gmp_datalink_t* datalink)
{
    uart_datalink = datalink;
}

void xplt_uart_dl_start_tx(gmp_datalink_t* datalink)
{
    uart_datalink = datalink;
    uart_tx_phase = XPLT_DL_UART_TX_HEADER;
    if (HAL_UART_Transmit_DMA(
            GMP_NUCLEO_DL_UART_HANDLE,
            (const uint8_t*)gmp_dev_dl_get_tx_hw_hdr_ptr(datalink),
            (uint16_t)gmp_dev_dl_get_tx_hw_hdr_size(datalink)) != HAL_OK)
    {
        uart_tx_phase = XPLT_DL_UART_TX_IDLE;
        gmp_dev_dl_tx_state_done(datalink);
    }
}
#endif

#if GMP_NUCLEO_ENABLE_CONTROL && GMP_NUCLEO_HAS_DAC
void xplt_dac_write(uint32_t value)
{
    if (value > 4095U)
        value = 4095U;
    (void)HAL_DAC_SetValue(GMP_NUCLEO_DAC_HANDLE, GMP_NUCLEO_DAC_CHANNEL,
                           DAC_ALIGN_12B_R, value);
}
#endif

#if GMP_NUCLEO_ENABLE_CONTROL && GMP_NUCLEO_ADC_REGULAR_DMA
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adc)
#elif GMP_NUCLEO_ENABLE_CONTROL
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* adc)
#endif
#if GMP_NUCLEO_ENABLE_CONTROL
{
    if (adc == GMP_NUCLEO_ADC_PRIMARY_HANDLE)
    {
        gmp_nucleo_platform_diag[0]++;
        gmp_base_ctl_step();
        user_dl_control_step();
    }
}
#endif

#if GMP_NUCLEO_ENABLE_CONTROL && GMP_NUCLEO_QEP_SOFTWARE_INDEX
#if GMP_NUCLEO_QEP_LEGACY_EXTI_CALLBACK
void HAL_GPIO_EXTI_Callback(uint16_t pin)
#else
void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin)
#endif
{
    if (pin == GMP_NUCLEO_QEP_Z_PIN)
        __HAL_TIM_SET_COUNTER(GMP_NUCLEO_QEP_TIMER_HANDLE, 0U);
}
#endif

#if GMP_NUCLEO_ENABLE_UART_DL
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size)
{
    if (uart == GMP_NUCLEO_DL_UART_HANDLE && uart_datalink != NULL)
    {
        gmp_nucleo_platform_diag[2]++;
        uint16_t previous = uart_rx_dma_position;
        if (size > previous)
        {
            gmp_dev_dl_push_str(uart_datalink, &uart_rx_dma_buffer[previous],
                                (size_gt)(size - previous));
        }
        else if (size < previous)
        {
            if (previous < GMP_NUCLEO_DL_RX_BUFFER_SIZE)
                gmp_dev_dl_push_str(
                    uart_datalink, &uart_rx_dma_buffer[previous],
                    (size_gt)(GMP_NUCLEO_DL_RX_BUFFER_SIZE - previous));
            if (size > 0U)
                gmp_dev_dl_push_str(uart_datalink, uart_rx_dma_buffer, size);
        }
        uart_rx_dma_position = size;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
    if (uart != GMP_NUCLEO_DL_UART_HANDLE || uart_datalink == NULL)
        return;

    gmp_nucleo_platform_diag[3]++;
    if (uart_tx_phase == XPLT_DL_UART_TX_HEADER &&
        gmp_dev_dl_get_tx_hw_pld_size(uart_datalink) > 0U)
    {
        uart_tx_phase = XPLT_DL_UART_TX_PAYLOAD;
        if (HAL_UART_Transmit_DMA(
                GMP_NUCLEO_DL_UART_HANDLE,
                (const uint8_t*)gmp_dev_dl_get_tx_hw_pld_ptr(uart_datalink),
                (uint16_t)gmp_dev_dl_get_tx_hw_pld_size(uart_datalink)) ==
            HAL_OK)
            return;
    }

    uart_tx_phase = XPLT_DL_UART_TX_IDLE;
    gmp_dev_dl_tx_state_done(uart_datalink);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
    if (uart == GMP_NUCLEO_DL_UART_HANDLE)
    {
        gmp_nucleo_platform_diag[4]++;
        if (uart_datalink != NULL && uart_tx_phase != XPLT_DL_UART_TX_IDLE)
        {
            uart_tx_phase = XPLT_DL_UART_TX_IDLE;
            gmp_dev_dl_tx_state_done(uart_datalink);
        }
        (void)HAL_UART_AbortReceive(uart);
        xplt_dl_arm_rx();
    }
}
#endif
