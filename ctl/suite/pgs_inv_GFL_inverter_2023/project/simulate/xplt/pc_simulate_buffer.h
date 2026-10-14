
// User should implement RX & TX buffer prototype in this header.

#ifndef _FILE_PC_SIMULATE_BUFFER_H_
#define _FILE_PC_SIMULATE_BUFFER_H_

#pragma pack(1)
// Specify Simulink communication TX structure type
// Default type name is tx_buf_t
typedef struct _tag_tx_buf_t
{
    // enable

    // Compare register
    uint32_t tabc[3];

    // monitor port
    double monitor_port[4];
} tx_buf_t;
#pragma pack()

#pragma pack(1)
// Specify Simulink communication RX structure type
// Default type name is rx_buf_t
typedef struct _tag_rx_buf_t
{
    // time

    // current feedback
    uint32_t iabc[3];

    // voltage feedback
    uint32_t uabc[3];

    // DC bus voltage
    uint32_t udc;

    // DC bus current
    uint32_t idc;

    // encoder feedback
    uint32_t encoder;

    int32_t revolution;

} rx_buf_t;
#pragma pack()

#define GMP_PC_SIMULINK_TX_STRUCT tx_buf_t
#define GMP_PC_SIMULINK_RX_STRUCT rx_buf_t

#endif // _FILE_PC_SIMULATE_BUFFER_H_
