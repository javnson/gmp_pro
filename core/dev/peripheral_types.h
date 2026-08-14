

#ifndef _FILE_PERIPHERAL_CFG_H_
#define _FILE_PERIPHERAL_CFG_H_

#include <gmp_type.h>

//////////////////////////////////////////////////////////////////////////
// Step I peripheral handle type
//

// ....................................................................//
// handle of UART (Universal Asynchronous Receiver/Transmitter)
//
#ifndef GMP_PORT_UART_T
#define GMP_PORT_UART_T void *
#endif // GMP_PORT_UART_T

typedef GMP_PORT_UART_T uart_halt;

// ....................................................................//
// handle of USART (Universal Synchronous/Asynchronous Receiver/Transmitter)
//
// #ifndef GMP_PORT_USART_T
// #define GMP_PORT_USART_T void *
// #endif // GMP_PORT_USART_T
//
// typedef GMP_PORT_USART_T usart_halt;

// ....................................................................//
// handle of IIC (Inter-Integrated Circuit)
//
#ifndef GMP_PORT_I2C_T
#define GMP_PORT_I2C_T void *
#endif // GMP_PORT_I2C_T

typedef GMP_PORT_I2C_T iic_halt;

// ....................................................................//
// handle of IIS (Inter-IC Sound)
//
#ifndef GMP_PORT_IIS_T
#define GMP_PORT_IIS_T void *
#endif // GMP_PORT_IIS_T

typedef GMP_PORT_IIS_T iis_halt;

// ....................................................................//
// handle of SPI (Serial Peripheral Interface Bus)
//
#ifndef GMP_PORT_SPI_T
#define GMP_PORT_SPI_T void *
#endif // GMP_PORT_SPI_T

typedef GMP_PORT_SPI_T spi_halt;

// ....................................................................//
// handle of CAN (Controller Area Network)
//
#ifndef GMP_PORT_CAN_T
#define GMP_PORT_CAN_T void *
#endif // GMP_PORT_CAN_T

typedef GMP_PORT_CAN_T can_halt;

// ....................................................................//
// handle of GPIO (General-purpose input/output)
//
#ifndef GMP_PORT_GPIO_T
// WARN: if this default GPIO handle is used the GPIO function is out-of-service,
// so any other code which will use GPIO should judge the following macro,
// in order to ensure the GPIO handle is implemented.
#define GMP_GPIO_NOT_IMPLEMENTED

#define GMP_PORT_GPIO_T void *
#endif // GMP_PORT_GPIO_T

typedef GMP_PORT_GPIO_T gpio_halt;

//////////////////////////////////////////////////////////////////////////
// Step II peripheral functions
//

#endif //_FILE_PERIPHERAL_CFG_H_
