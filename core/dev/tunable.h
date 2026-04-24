/**
 * @file gmp_tunable.h
 * @brief GMP Tunable Application Layer - Variable & Memory Access Dictionary.
 * @details Implements the ASAP2-like data dictionary and protocol dispatcher.
 * Supports static ROM-based variable registration and dynamic RAM-based backdoors.
 */

#include <core/dev/datalink.h>

#ifndef _FILE_GMP_TUNABLE_H
#define _FILE_GMP_TUNABLE_H

// ---------------------------------------------------------
// 1. Command Definitions (CMD)
// ---------------------------------------------------------
#define GMP_CMD_VAR_INFO_REQ  0x10 ///< PC requests variable physical address and metadata
#define GMP_CMD_VAR_INFO_ACK  0x11 ///< MCU replies with metadata
#define GMP_CMD_VAR_READ_REQ  0x12 ///< PC requests variable value
#define GMP_CMD_VAR_READ_ACK  0x13 ///< MCU replies with variable value
#define GMP_CMD_VAR_WRITE_REQ 0x14 ///< PC requests to write variable value
#define GMP_CMD_VAR_WRITE_ACK 0x15 ///< MCU replies with write status

#define GMP_CMD_BACKDOOR_CFG 0x19 ///< PC configures a backdoor channel
#define GMP_CMD_BACKDOOR_ACK 0x1A ///< MCU replies with backdoor config status

#define GMP_CMD_MEM_READ_REQ  0x20 ///< PC requests raw memory block
#define GMP_CMD_MEM_READ_ACK  0x21 ///< MCU replies with memory block data
#define GMP_CMD_MEM_WRITE_REQ 0x22 ///< PC requests to write memory block
#define GMP_CMD_MEM_WRITE_ACK 0x23 ///< MCU replies with memory write status

// ---------------------------------------------------------
// 2. Data Types & Permissions
// ---------------------------------------------------------

/**
 * @brief  Supported variable data types for the dictionary.
 * @note   These map to byte/word lengths for memcpy and network transfer.
 */
typedef enum
{
    GMP_VAR_TYPE_UINT16 = 0x01, ///< 16-bit unsigned (2 bytes on wire)
    GMP_VAR_TYPE_INT16 = 0x02,  ///< 16-bit signed   (2 bytes on wire)
    GMP_VAR_TYPE_UINT32 = 0x03, ///< 32-bit unsigned (4 bytes on wire)
    GMP_VAR_TYPE_INT32 = 0x04,  ///< 32-bit signed   (4 bytes on wire)
    GMP_VAR_TYPE_FLOAT = 0x05   ///< 32-bit IEEE 754 (4 bytes on wire)
} gmp_var_type_t;

/**
 * @brief  Access permissions for dictionary entries.
 */
typedef enum
{
    GMP_VAR_ACCESS_RO = 0x00, ///< Read-Only (e.g., ADC results, Status flags)
    GMP_VAR_ACCESS_RW = 0x01  ///< Read-Write (e.g., PI Gains, Targets)
} gmp_var_access_t;

/**
 * @brief  Write Request Status Codes.
 */
typedef enum
{
    GMP_VAR_STATUS_OK = 0x00,         ///< Write successful
    GMP_VAR_STATUS_ERR_RO = 0x01,     ///< Error: Variable is Read-Only
    GMP_VAR_STATUS_ERR_NO_ID = 0x02,  ///< Error: Variable ID not found
    GMP_VAR_STATUS_ERR_BAD_LEN = 0x03 ///< Error: Payload length mismatch
} gmp_var_status_t;

/**
 * @brief  Memory Access Status Codes.
 */
typedef enum
{
    GMP_MEM_STATUS_OK = 0x00,      ///< Memory access successful
    GMP_MEM_STATUS_ERR_RO = 0x01,  ///< Error: Memory region is Read-Only
    GMP_MEM_STATUS_ERR_OOB = 0x02, ///< Error: Address Out Of Bounds (Not in whitelist)
    GMP_MEM_STATUS_ERR_MTU = 0x03  ///< Error: Requested length exceeds Datalink MTU
} gmp_mem_status_t;

/**
 * @brief  Access permissions for memory regions.
 */
typedef enum
{
    GMP_MEM_ATTR_RO = 0x00, ///< Read-Only Region (e.g., Flash, Calibration Data)
    GMP_MEM_ATTR_RW = 0x01  ///< Read-Write Region (e.g., RAM Buffers)
} gmp_mem_attr_t;

// ---------------------------------------------------------
// 3. Dictionary Entry Structure
// ---------------------------------------------------------

/**
 * @brief  A single entry in the Variable Data Dictionary.
 * @details Stores the physical memory pointer and its metadata. 
 */
typedef struct
{
    void* var_ptr;           ///< Physical address pointer of the variable
    gmp_var_type_t var_type; ///< Data type for length deduction and casting
    gmp_var_access_t access; ///< Read/Write permission
} gmp_var_entry_t;

/**
 * @brief  A single entry in the Memory Protection Whitelist.
 * @details Defines a safe memory region that the PC is allowed to access.
 */
typedef struct
{
    uint32_t start_addr; ///< Start physical address of the allowed region
    uint32_t size;       ///< Size of the allowed region (in MAU / Bytes)
    gmp_mem_attr_t attr; ///< Read/Write permission for this region
} gmp_mem_region_t;

// ---------------------------------------------------------
// 4. API Declarations
// ---------------------------------------------------------

/**
 * @brief  Initialize the Tunable application layer.
 * @param  dl_ctx Pointer to the underlying initialized datalink context.
 */
void gmp_tunable_init(gmp_datalink_t* dl_ctx);

/**
 * @brief  The core application RX callback, injected into datalink.
 * @details Acts as the central dispatcher for all tunable protocol commands.
 */
void gmp_tunable_rx_callback(uint16_t target_id, uint16_t cmd, const data_gt* payload, size_gt len);

// ---------------------------------------------------------
// 5. External Dictionary Declarations (User Provided)
// ---------------------------------------------------------
extern const gmp_var_entry_t gmp_static_dict[];
extern const uint16_t gmp_static_dict_size;

extern const gmp_mem_region_t gmp_mem_whitelist[];
extern const uint16_t gmp_mem_whitelist_size;

#endif // _FILE_GMP_TUNABLE_H
