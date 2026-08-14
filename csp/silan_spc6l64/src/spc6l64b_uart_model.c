#include <driver/inc/uart1.h>

// uart handle
extern UART_PARAM_s Uart0_S, Uart1_S, Uart2_S;

void gmp_hal_uart_write(uart_halt uart, const byte_gt *data, size_gt length)
{
    size_gt i;

    for (i = 0; i < length; ++i)
        UartSendByte(uart, data[i]);
}

