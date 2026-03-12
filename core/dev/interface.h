/**
 * @file devif.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file will define all the device interface (Dev-IF)
// All the initialize function is implemented in dev_util.c

#ifndef _FILE_DEVICE_INTERFACE_H_
#define _FILE_DEVICE_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// This file provide a set of structure type which owns a suffix ift (interface type).

// This file is independent of the other modules.

// ....................................................................//

///**
// * @brief half Duplex communication.
// * such as half duplex SPI, half duplex UART (RS485)
// */
//typedef struct _tag_half_duplex_channel_t
//{
//    /**
//     * @brief this is a pointer of data to be transmit
//     */
//    data_gt *buf;
//
//    // length of the message
//    size_gt length;
//
//    // capacity of the message
//    size_gt capacity;
//
//} half_duplex_ift;
//
///**
// * @brief initialize a half duplex interface
// * @param channel half_duplex_ift handle
// * @param buf
// * @param length
// * @param capacity
// */
//void gmp_dev_init_half_duplex_channel(half_duplex_ift *channel, data_gt *buf, size_gt length, size_gt capacity);
//
//// ....................................................................//
//// Duplex communication
//// Such as SPI, UART, USART
//typedef struct _tag_duplex_channel_t
//{
//    // transmit buffer
//    data_gt *tx_buf;
//
//    // receive buffer
//    data_gt *rx_buf;
//
//    // length of the buffer (tx and rx buffer have the same length)
//    size_gt length;
//
//    // capacity of the buffer (tx and rx buf have the same capacity)
//    size_gt capacity;
//
//} duplex_ift;
//
///**
// * @brief initialize a duplex interface
// * @param channel duplex_ift handle
// * @param tx_buf transimit buffer
// * @param rx_buf receive buffer
// * @param length length of transmit and receive buffer
// * @param capacity capacity of tx and rx buffer
// */
//void gmp_dev_init_duplex_channel(duplex_ift *channel, data_gt *tx_buf, data_gt *rx_buf, size_gt length,
//                                 size_gt capacity);
//
//// ....................................................................//
//// half duplex with Address
//// such as IIC
//typedef struct _tag_half_duplex_with_addr_channel_t
//{
//    // Address of device
//    addr32_gt address;
//
//    // message buffer
//    data_gt *msg;
//
//    // length of buffer
//    size_gt length;
//
//} half_duplex_with_addr_ift;
//
///**
// * @brief initialize a half duplex with address interface
// * @param channel half duplex with address interface handle
// * @param address initialize a address
// * @param msg msg source address
// * @param length length of address
// */
//void gmp_dev_init_half_duplex_with_addr_channel(half_duplex_with_addr_ift *channel, addr32_gt address, data_gt *msg,
//                                                size_gt length);
//
//// ....................................................................//
//// half duplex with dual address for IIC memeory mode
//typedef struct _tag_iic_memory_channel_t
//{
//    // device address
//    addr32_gt dev_addr;
//
//    // device memory address
//    addr32_gt mem_addr;
//
//    // device memory address length
//    fast_gt mem_length;
//
//    // message buffer
//    data_gt *msg;
//
//    // length of buffer
//    size_gt length;
//
//} iic_memory_ift;
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
//                                     fast_gt mem_length, data_gt *msg, size_gt length);
//
//// ....................................................................//
//// CAN interface
//typedef struct _tag_can_channel_t
//{
//    // can address
//    // std ID is a 11-bit number, 0 ~ 0x7FF
//    // ext ID is a 29-bit number, 0 ~ 0x1FFF_FFFF
//    addr32_gt id;
//
//    // can frame type
//    // RTR flag: remote flag
//    // IDE flag; extension flag
//    uint32_t properties;
//
//    // can data length
//    fast_gt length;
//
//    // can data buffer
//    data_gt data[8];
//
//} can_ift;
//
///**
// * @brief initialize a can interface object
// * @param channel can interface handle
// * @param id can address
// * @param properties can frame type
// * @param length can data length
// */
//void gmp_dev_init_can_channel(can_ift *channel, addr32_gt id, uint32_t properties);
//
//// This following file provide a peripheral slot
//// This file will provide a standard communication with MATLAB Simulink.
//// organize a set of peripherals by peripheral_plugin;
//// All the peripheral_plugin are listed by peripheral slot.
//
//// ....................................................................//
//// Slot interface
//// This struct contains a fixed length buffer
//typedef struct _tag_slot_interface_t
//{
//    // slot size
//    addr_gt length;
//
//    // slot buffer pointer
//    data_gt *buffer;
//} interface_slot_ift;
//
//// ....................................................................//
//// Strip for slot interface
//// This struct contains a list of slots.
//typedef struct _tag_strip_interface_t
//{
//    // strip size
//    // how many slots the slot has.
//    uint32_t size;
//
//    // pointer list of slot interface
//    interface_slot_ift **slot_it;
//} interface_strip_ift;

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_DEVICE_INTERFACE_H_
