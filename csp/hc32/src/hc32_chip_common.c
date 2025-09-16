/**
 * @file hc32_chip_common.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

/* debug USART channel definition */
#define GMP_CSP_DBG_USART_CH (M4_USART2)

/* debug USART baudrate definition */
#define GMP_CSP_DBG_USART_BAUDRATE (115200ul)

/* debug USART RX Port/Pin definition */
#define GMP_CSP_DBG_USART_RX_PORT (PortA)
#define GMP_CSP_DBG_USART_RX_PIN  (Pin10)
#define GMP_CSP_DBG_USART_RX_FUNC (Func_Usart2_Rx)

/* debug USART TX Port/Pin definition */
#define GMP_CSP_DBG_USART_TX_PORT (PortA)
#define GMP_CSP_DBG_USART_TX_PIN  (Pin15)
#define GMP_CSP_DBG_USART_TX_FUNC (Func_Usart2_Tx)

// This varaible should increase per micro second.
time_gt gmp_internal_system_tick = 0;

// HC32 doesn't have a System Tick, so use a variable instead.
time_gt gmp_base_get_system_tick(void)
{
    return gmp_internal_system_tick;
}

// This function should be called by user every [ms].
void gmp_base_upgrade_system_tick(void)
{
    gmp_internal_system_tick += 1;
}

/**
 * @brief This function may fresh IWDG counter.
 * This function should be implemented by CSP,
 * Every Loop routine, this function would be called.
 * CSP implementation should ensure that the function has only one thing is to feed the watchdog
 */
void gmp_hal_wd_feed(void)
{
#if defined SPECIFY_ENABLE_FEED_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg);
#endif // SPECIFY_ENABLE_FEED_WATCHDOG
}

// This function may be called and used to initilize all the peripheral.
void gmp_csp_startup(void)
{
    en_result_t enRet = Ok;

    /* Initialize Clock */
    BSP_CLK_Init();

    // Disable TDI TDO
    // If this two function is disbaled, UART output may invisable.
    PORT_DebugPortSetting(TDI, Disable);
    PORT_DebugPortSetting(TDO_SWO, Disable);

    // Enable peripheral clock
    uint32_t u32Fcg1Periph =
        PWC_FCG1_PERIPH_USART1 | PWC_FCG1_PERIPH_USART2 | PWC_FCG1_PERIPH_USART3 | PWC_FCG1_PERIPH_USART4;

    PWC_Fcg1PeriphClockCmd(u32Fcg1Periph, Enable);

    // Initialize GMP CSP USART IO
    PORT_SetFunc(GMP_CSP_DBG_USART_RX_PORT, GMP_CSP_DBG_USART_RX_PIN, USART_RX_FUNC, Disable);
    PORT_SetFunc(GMP_CSP_DBG_USART_TX_PORT, GMP_CSP_DBG_USART_TX_PIN, USART_TX_FUNC, Disable);

    // Initialize UART
    const stc_usart_uart_init_t stcInitCfg = {
        UsartIntClkCkNoOutput, UsartClkDiv_1,   UsartDataBits8,        UsartDataLsbFirst, UsartOneStopBit,
        UsartParityNone,       UsartSampleBit8, UsartStartBitFallEdge, UsartRtsEnable,
    };

    enRet = USART_UART_Init(GMP_CSP_DBG_USART_CH, &stcInitCfg);
    if (enRet != Ok)
    {
        gmp_base_system_stuck();
    }

    // Set baudrate
    enRet = USART_SetBaudrate(GMP_CSP_DBG_USART_CH, GMP_CSP_DBG_USART_BAUDRATE);
    if (enRet != Ok)
    {
        gmp_base_system_stuck();
    }

    /*Enable RX && TX function*/
    USART_FuncCmd(GMP_CSP_DBG_USART_CH, UsartRx, Enable);
    USART_FuncCmd(GMP_CSP_DBG_USART_CH, UsartTx, Enable);
}

// This function would be called when fatal error occorred.
void gmp_port_system_stuck(void)
{
}

// This function would be called when all the initilization process happened.
void gmp_csp_post_process(void)
{
}

// This function is unreachable.
void gmp_exit_routine(void)
{
}

// This function may invoke when main loop occurred.
void gmp_csp_loop(void)
{
}

// redirect output USART to printf
int fputc(int ch, FILE* f)
{
    USART_SendData(GMP_CSP_DBG_USART_CH, (uint8_t)ch);
    while (Reset == USART_GetStatus(GMP_CSP_DBG_USART_CH, UsartTxEmpty))
        ;

    return ch;
}

// Clear receive fault flags
void usart_clear_rx_error(void)
{
    if (Set == USART_GetStatus(GMP_CSP_DBG_USART_CH, UsartFrameErr))
    {
        USART_ClearStatus(GMP_CSP_DBG_USART_CH, UsartFrameErr);
    }

    if (Set == USART_GetStatus(GMP_CSP_DBG_USART_CH, UsartParityErr))
    {
        USART_ClearStatus(GMP_CSP_DBG_USART_CH, UsartParityErr);
    }

    if (Set == USART_GetStatus(GMP_CSP_DBG_USART_CH, UsartOverrunErr))
    {
        USART_ClearStatus(GMP_CSP_DBG_USART_CH, UsartOverrunErr);
    }
}
