/**
 * @file tunable.h
 * @brief GMP Tunable Application Layer - Decoupled Variable, Memory, and Flex Modules.
 * @details Implements object-oriented tunable modules with short-circuit dispatcher logic.
 * Fully compatible with DSP/ARM architectures via data_gt mapping.
 * * * HOW TO USE THIS MODULE:
 * 1. Definition: Define your static variable dictionaries (`gmp_var_entry_t` array), 
 * memory whitelists (`gmp_mem_region_t` array), and RAM arrays for flex backdoors.
 * 2. Initialization: Instantiate the desired module contexts (e.g., `gmp_tunable_var_t`) 
 * and call their respective init functions. You can assign the SAME `base_cmd` 
 * to all modules (e.g., 0x10) because their internal offsets are designed to not overlap.
 * 3. Dispatching: In your datalink application RX callback, chain the module callbacks 
 * sequentially. Use the short-circuit return value (`GMP_TUNABLE_HANDLED`) to stop 
 * propagation once a module handles the command. 
 * * Example:
 * if (gmp_tunable_var_rx_cb(...) == GMP_TUNABLE_HANDLED) return;
 * if (gmp_tunable_flex_rx_cb(...) == GMP_TUNABLE_HANDLED) return;
 * if (gmp_tunable_mem_rx_cb(...) == GMP_TUNABLE_HANDLED) return;
 */

#ifndef _FILE_GMP_TUNABLE_H
#define _FILE_GMP_TUNABLE_H

#include <core/dev/datalink.h>

// ---------------------------------------------------------
// 1. Core Macros & Return Codes
// ---------------------------------------------------------
#define GMP_TUNABLE_HANDLED 0 ///< Command was successfully handled, stop dispatching
#define GMP_TUNABLE_PASS    1 ///< Command does not belong to this module, pass to next

// ---------------------------------------------------------
// 3. Command Offset Definitions
// ---------------------------------------------------------
// Offsets for Variable & Flex Backdoor Modules (0 ~ 5)
#define GMP_TUNABLE_OFFSET_INFO_REQ      0 ///< Request dictionary info
#define GMP_TUNABLE_OFFSET_INFO_ACK      1 ///< Acknowledge dictionary info
#define GMP_TUNABLE_OFFSET_VAR_READ_REQ  2 ///< Request to read variable
#define GMP_TUNABLE_OFFSET_VAR_READ_ACK  3 ///< Acknowledge variable read
#define GMP_TUNABLE_OFFSET_VAR_WRITE_REQ 4 ///< Request to write variable
#define GMP_TUNABLE_OFFSET_VAR_WRITE_ACK 5 ///< Acknowledge variable write

// Offsets for Memory Module (6 ~ 9)
#define GMP_TUNABLE_OFFSET_MEM_READ_REQ  6 ///< Request to read memory block
#define GMP_TUNABLE_OFFSET_MEM_READ_ACK  7 ///< Acknowledge memory block read
#define GMP_TUNABLE_OFFSET_MEM_WRITE_REQ 8 ///< Request to write memory block
#define GMP_TUNABLE_OFFSET_MEM_WRITE_ACK 9 ///< Acknowledge memory block write


// ---------------------------------------------------------
// 4. Data Types & Permissions
// ---------------------------------------------------------

/**
 * @brief Supported variable data types for the dictionary.
 */
typedef enum
{
    GMP_VAR_TYPE_UINT16 = 0x01, ///< 16-bit unsigned
    GMP_VAR_TYPE_INT16 = 0x02,  ///< 16-bit signed
    GMP_VAR_TYPE_UINT32 = 0x03, ///< 32-bit unsigned
    GMP_VAR_TYPE_INT32 = 0x04,  ///< 32-bit signed
    GMP_VAR_TYPE_FLOAT = 0x05   ///< 32-bit IEEE 754 Float
} gmp_var_type_t;

/**
 * @brief Unified memory and variable access permissions.
 */
typedef enum
{
    GMP_TUNABLE_ACCESS_RO = 0x00, ///< Read-Only Region/Variable
    GMP_TUNABLE_ACCESS_RW = 0x01  ///< Read-Write Region/Variable
} gmp_tunable_access_t;

// ---------------------------------------------------------
// 5. Dictionary Entry Structures
// ---------------------------------------------------------

typedef struct
{
    void* var_ptr;               ///< Physical address pointer
    gmp_var_type_t var_type;     ///< Data type
    gmp_tunable_access_t access; ///< Read/Write permission
} gmp_var_entry_t;

typedef struct
{
    uint32_t start_addr;         ///< Start physical address
    uint32_t size;               ///< Size of the region (in MAU / data_gt elements)
    gmp_tunable_access_t access; ///< Read/Write permission
} gmp_mem_region_t;

// =========================================================
// MODULE 1: VARIABLE TUNABLE (Static Dictionary)
// =========================================================
typedef struct
{
    gmp_datalink_t* dl_ctx;
    uint16_t base_cmd;
    const gmp_var_entry_t* static_dict;
    uint16_t static_dict_size;
} gmp_tunable_var_t;

/**
 * @brief  Initialize the static variable tunable module.
 * @param  ctx        Pointer to the variable tunable context to initialize.
 * @param  dl_ctx     Pointer to the bound datalink instance for transmitting ACKs.
 * @param  base_cmd   The shared base command offset for the tunable subsystem (e.g., 0x10).
 * @param  dict       Pointer to the static ROM-based variable dictionary array.
 * @param  dict_size  Number of entries in the static variable dictionary.
 */
void gmp_tunable_var_init(gmp_tunable_var_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd,
                          const gmp_var_entry_t* dict, uint16_t dict_size);

/**
 * @brief  Receive callback dispatcher for the variable tunable module.
 * @param  ctx        Pointer to the initialized variable tunable context.
 * @param  target_id  The destination ID of the received frame.
 * @param  cmd        The command byte extracted from the frame header.
 * @param  payload    Pointer to the payload data array.
 * @param  len        Length of the payload in data_gt elements.
 * @return fast_gt    GMP_TUNABLE_HANDLED (0) if processed, GMP_TUNABLE_PASS (1) if ignored.
 */
fast_gt gmp_tunable_var_rx_cb(gmp_tunable_var_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                              size_gt len);

// =========================================================
// MODULE 2: MEMORY TUNABLE (Whitelist Protection)
// =========================================================
typedef struct
{
    gmp_datalink_t* dl_ctx;
    uint16_t base_cmd;
    const gmp_mem_region_t* whitelist;
    uint16_t whitelist_size;
} gmp_tunable_mem_t;

/**
 * @brief  Initialize the memory protection and access tunable module.
 * @param  ctx            Pointer to the memory tunable context to initialize.
 * @param  dl_ctx         Pointer to the bound datalink instance.
 * @param  base_cmd       The shared base command offset for the tunable subsystem.
 * @param  whitelist      Pointer to the static ROM-based memory whitelist array.
 * @param  whitelist_size Number of allowed regions in the whitelist.
 */
void gmp_tunable_mem_init(gmp_tunable_mem_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd,
                          const gmp_mem_region_t* whitelist, uint16_t whitelist_size);

/**
 * @brief  Receive callback dispatcher for the memory tunable module.
 * @param  ctx        Pointer to the initialized memory tunable context.
 * @param  target_id  The destination ID of the received frame.
 * @param  cmd        The command byte extracted from the frame header.
 * @param  payload    Pointer to the payload data array.
 * @param  len        Length of the payload in data_gt elements.
 * @return fast_gt    GMP_TUNABLE_HANDLED (0) if processed, GMP_TUNABLE_PASS (1) if ignored.
 */
fast_gt gmp_tunable_mem_rx_cb(gmp_tunable_mem_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                              size_gt len);

// =========================================================
// MODULE 3: FLEX VARIABLE TUNABLE (Dynamic Backdoor)
// =========================================================
typedef struct
{
    gmp_datalink_t* dl_ctx;
    uint16_t base_cmd;
    gmp_var_entry_t* backdoor_dict;
    uint16_t max_channels;
} gmp_tunable_flex_t;

/**
 * @brief  Initialize the dynamic flex (backdoor) tunable module.
 * @param  ctx          Pointer to the flex tunable context to initialize.
 * @param  dl_ctx       Pointer to the bound datalink instance.
 * @param  base_cmd     The shared base command offset for the tunable subsystem.
 * @param  ram_dict     Pointer to a RAM-allocated array for storing dynamic backdoor channels.
 * @param  max_channels Maximum number of backdoor channels (size of the ram_dict array).
 */
void gmp_tunable_flex_init(gmp_tunable_flex_t* ctx, gmp_datalink_t* dl_ctx, uint16_t base_cmd,
                           gmp_var_entry_t* ram_dict, uint16_t max_channels);

/**
 * @brief  Receive callback dispatcher for the dynamic flex tunable module.
 * @param  ctx        Pointer to the initialized flex tunable context.
 * @param  target_id  The destination ID of the received frame.
 * @param  cmd        The command byte extracted from the frame header.
 * @param  payload    Pointer to the payload data array.
 * @param  len        Length of the payload in data_gt elements.
 * @return fast_gt    GMP_TUNABLE_HANDLED (0) if processed, GMP_TUNABLE_PASS (1) if ignored.
 */
fast_gt gmp_tunable_flex_rx_cb(gmp_tunable_flex_t* ctx, uint16_t target_id, uint16_t cmd, const data_gt* payload,
                               size_gt len);

#endif // _FILE_GMP_TUNABLE_H
