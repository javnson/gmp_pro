/**
 * @file gmp_dev_print.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_type.h>
#include <core/dev/peripheral_types.h>
#include <core/dev/peripheral_port.h>

#ifndef SPECIFY_DISABLE_CSP
#include <csp.general.h>
#endif // SPECIFY_DISABLE_CSP

//////////////////////////////////////////////////////////////////////////
// Print default function

#if !defined(SPECIFY_DISABLE_CSP) && !defined(SPECIFY_PC_ENVIRONMENT)
#ifndef DISABLE_GMP_DEV_PRINT_DEFAULT
// implement the gmp_debug_print routine.
size_gt gmp_base_print_default(const char* p_fmt, ...)
{
    // if no one was specified to output, just ignore the request.
    if (debug_uart == NULL)
    {
        return 0;
    }

    // size_gt size = (size_gt)strlen(p_fmt);

    static byte_gt str[GMP_BASE_PRINT_CHAR_EXT];
    memset(str, 0, GMP_BASE_PRINT_CHAR_EXT);

    va_list vArgs;
    va_start(vArgs, p_fmt);
    vsprintf((char*)str, (char const*)p_fmt, vArgs);
    va_end(vArgs);

    size_gt length = (size_gt)strlen((char*)str);

    gmp_hal_uart_write(debug_uart, str, length, 10);

    return length;
}
#endif // DISABLE_GMP_DEV_PRINT_DEFAULT
#endif // !SPECIFY_DISABLE_CSP && !SPECIFY_PC_ENVIRONMENT

/////////////////////////////////////////////////////////////////
// channel
//

///**
// * @brief initialize a half duplex channel
// * @param channel half_duplex_ift handle
// * @param buf
// * @param length
// * @param capacity
// */
//void gmp_dev_init_half_duplex_channel(half_duplex_ift *channel, byte_gt *buf, size_gt length, size_gt capacity)
//{
//    channel->buf = buf;
//    channel->length = length;
//    channel->capacity = capacity;
//}
//
///**
// * @brief initialize a duplex channel
// * @param channel duplex_ift handle
// * @param tx_buf transimit buffer
// * @param rx_buf receive buffer
// * @param length length of transmit and receive buffer
// * @param capacity capacity of tx and rx buffer
// */
//void gmp_dev_init_duplex_channel(duplex_ift *channel, byte_gt *tx_buf, byte_gt *rx_buf, size_gt length,
//                                 size_gt capacity)
//{
//    channel->tx_buf = tx_buf;
//    channel->rx_buf = rx_buf;
//    channel->length = length;
//    channel->capacity = capacity;
//}
//
///**
// * @brief initialize a half duplex with address interface
// * @param channel half duplex with address interface handle
// * @param address initialize a address
// * @param msg msg source address
// * @param length length of address
// */
//void gmp_dev_init_half_duplex_with_addr_channel(half_duplex_with_addr_ift *channel, addr32_gt address, byte_gt *msg,
//                                                size_gt length)
//{
//    channel->address = address;
//    channel->msg = msg;
//    channel->length = length;
//}
//
///**
// * @brief initialize iic memeory interface object.
// * @param channel handle of IIC memory
// * @param dev_addr IIC device Address
// * @param mem_addr memory address of IIC device
// * @param mem_length mem address length
// * @param msg message object
// * @param length length of message
// */
//void gmp_dev_init_iic_memory_channel(iic_memory_ift *channel, addr32_gt dev_addr, addr32_gt mem_addr,
//                                     fast_gt mem_length, byte_gt *msg, size_gt length)
//{
//    channel->dev_addr = dev_addr;
//    channel->mem_addr = mem_addr;
//    channel->mem_length = mem_length;
//    channel->msg = msg;
//    channel->length = length;
//}
//
///**
// * @brief initialize a can interface object
// * @param channel can interface handle
// * @param id can address
// * @param properties can frame type
// * @param length can data length
// */
//void gmp_dev_init_can_channel(can_ift *channel, addr32_gt id, uint32_t properties)
//{
//    channel->id = id;
//    channel->properties = properties;
//    channel->length = 0;
//    memset(channel->data, 0, 8);
//}
//

//////////////////////////////////////////////////////////////////////////
// CAN Deque

#include <core/dev/peripheral_port.h>

//void gmp_can_deque_init(gmp_can_deque_t* dq, gmp_can_msg_t* buf, uint16_t cap)
//{
//    if (dq == NULL || buf == NULL)
//        return;
//
//    dq->buffer = buf;
//    dq->capacity = cap;
//    dq->head = 0;
//    dq->tail = 0;
//    dq->count = 0;
//}
//
//ec_gt gmp_can_deque_push_back(gmp_can_deque_t* dq, const gmp_can_msg_t* msg)
//{
//    ec_gt status = GMP_EC_OK;
//
//    /* Enter critical section to ensure atomic index update */
//    gmp_base_enter_critical();
//
//    if (dq->count >= dq->capacity)
//    {
//        status = GMP_EC_DEQUE_FULL;
//    }
//    else
//    {
//        /* Insert at the tail */
//        dq->buffer[dq->tail] = *msg;
//        /* Move tail forward circularly */
//        dq->tail = (uint16_t)(dq->tail + 1) % dq->capacity;
//        dq->count++;
//    }
//
//    gmp_base_leave_critical();
//    return status;
//}
//
//ec_gt gmp_can_deque_push_front(gmp_can_deque_t* dq, const gmp_can_msg_t* msg)
//{
//    ec_gt status = GMP_EC_OK;
//
//    gmp_base_enter_critical();
//
//    if (dq->count >= dq->capacity)
//    {
//        status = GMP_EC_DEQUE_FULL;
//    }
//    else
//    {
//        /* Move head backward circularly */
//        dq->head = (uint16_t)(dq->head + dq->capacity - 1) % dq->capacity;
//        /* Insert at the new head */
//        dq->buffer[dq->head] = *msg;
//        dq->count++;
//    }
//
//    gmp_base_leave_critical();
//    return status;
//}
//
//ec_gt gmp_can_deque_pop_front(gmp_can_deque_t* dq, gmp_can_msg_t* msg_ret)
//{
//    ec_gt status = GMP_EC_OK;
//
//    gmp_base_enter_critical();
//
//    if (dq->count == 0)
//    {
//        status = GMP_EC_DEQUE_EMPTY;
//    }
//    else
//    {
//        /* Retrieve from the head */
//        if (msg_ret != NULL)
//        {
//            *msg_ret = dq->buffer[dq->head];
//        }
//        /* Move head forward circularly */
//        dq->head = (uint16_t)(dq->head + 1) % dq->capacity;
//        dq->count--;
//    }
//
//    gmp_base_leave_critical();
//    return status;
//}
//
//
//void gmp_can_node_init(gmp_can_node_t* node, can_halt bus, gmp_can_msg_t* tx_buf, uint16_t tx_cap,
//                       gmp_can_msg_t* rx_buf, uint16_t rx_cap)
//{
//    if (node == NULL)
//        return;
//
//    node->bus = bus;
//    gmp_can_deque_init(&node->tx_deque, tx_buf, tx_cap);
//    gmp_can_deque_init(&node->rx_slow_queue, rx_buf, rx_cap);
//
//    node->fast_rx_mask = 0;
//    node->fast_rx_id = 0;
//    node->fast_rx_callback = NULL;
//}
//
//void gmp_can_node_set_fast_path(gmp_can_node_t* node, uint32_t id, uint32_t mask, void (*cb)(const gmp_can_msg_t*))
//{
//    node->fast_rx_id = id;
//    node->fast_rx_mask = mask;
//    node->fast_rx_callback = cb;
//}
//
///**
// * @details 当物理硬件邮箱发送完成触发中断时，调用此函数。
// * 它会从双端队列的头部（pop_front）取消息。由于 PDO 是通过 push_front 存入的，
// * 因此它们会比 SDO 更早被泵送到硬件邮箱中。
// */
//void gmp_can_node_tx_isr_pump(gmp_can_node_t* node)
//{
//    gmp_can_msg_t next_msg;
//
//    /* 如果队列中有待发消息，则尝试直接写入硬件 */
//    if (gmp_can_deque_pop_front(&node->tx_deque, &next_msg) == GMP_EC_OK)
//    {
//        /* 调用 CSP 层的物理写函数。如果此时硬件邮箱又满了（极罕见），
//         * 则需要重新压回队首，等待下一次中断触发。 */
//        if (gmp_hal_can_bus_write(node->bus, &next_msg) != GMP_EC_OK)
//        {
//            gmp_can_deque_push_front(&node->tx_deque, &next_msg);
//        }
//    }
//}
//
///**
// * @details 当物理硬件收到任何消息触发中断时，调用此函数。
// * 它根据用户设定的掩码进行分流：
// * 1. 命中快路径：直接原地执行回调（适用于电机的电流、位置同步）。
// * 2. 未命中：压入慢速队列，等待主循环处理（适用于参数配置）。
// */
//void gmp_can_node_rx_isr_router(gmp_can_node_t* node, const gmp_can_msg_t* rx_msg)
//{
//    /* 检查 ID 是否匹配快速通道规则 */
//    if (((rx_msg->id & node->fast_rx_mask) == (node->fast_rx_id & node->fast_rx_mask)) &&
//        (node->fast_rx_callback != NULL))
//    {
//        /* 立即响应实时消息 */
//        node->fast_rx_callback(rx_msg);
//    }
//    else
//    {
//        /* 否则存入非实时队列 */
//        gmp_can_deque_push_back(&node->rx_slow_queue, rx_msg);
//    }
//}
//
///**
// * @brief  Transmits a CAN message via the node.
// * @note   This is a non-blocking call. If the hardware is busy, the message is
// * queued in the software deque based on its priority.
// * * @param  node     Pointer to the logical CAN node.
// * @param  msg      The CAN message to transmit.
// * @param  priority Transmission priority (Normal or High).
// * @return ec_gt    GMP_EC_OK on success, or GMP_EC_DEQUE_FULL if software buffer is overflowed.
// */
//ec_gt gmp_can_node_transmit(gmp_can_node_t* node, const gmp_can_msg_t* msg, gmp_can_priority_et priority)
//{
//    if (node == NULL || msg == NULL)
//        return GMP_EC_GENERAL_ERROR;
//
//    ec_gt status;
//
//    /* 1. 尝试直接写入硬件 CSP 层 (Layer 1) */
//    /* 注意：此函数必须是原子性的，由 CSP 实现决定 */
//    status = gmp_hal_can_bus_write(node->bus, msg);
//
//    /* 2. 如果硬件邮箱忙，则根据优先级存入软件队列 */
//    if (status == GMP_EC_BUSY)
//    {
//        if (priority == GMP_CAN_PRIORITY_HIGH)
//        {
//            /* 高优先级：插到队首，下次 TX 中断第一个发它 */
//            status = gmp_can_deque_push_front(&node->tx_deque, msg);
//        }
//        else
//        {
//            /* 普通优先级：排在队尾 */
//            status = gmp_can_deque_push_back(&node->tx_deque, msg);
//        }
//    }
//
//    return status;
//}
//
///**
// * @brief  Reads a non-real-time message from the node's slow receive queue.
// * @note   To be called in the main loop or low-priority background tasks.
// * * @param  node     Pointer to the logical CAN node.
// * @param  msg_ret  Pointer to store the retrieved message.
// * @return ec_gt    GMP_EC_OK if a message was read, GMP_EC_DEQUE_EMPTY otherwise.
// */
//ec_gt gmp_can_node_receive_slow(gmp_can_node_t* node, gmp_can_msg_t* msg_ret)
//{
//    if (node == NULL || msg_ret == NULL)
//        return GMP_EC_GENERAL_ERROR;
//
//    /* 从慢速队列头部取出消息 */
//    return gmp_can_deque_pop_front(&node->rx_slow_queue, msg_ret);
//}
