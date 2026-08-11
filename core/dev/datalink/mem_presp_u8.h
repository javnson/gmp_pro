/**
 * @file mem_presp_u8.h
 * @brief Argos Memory Perspective (Direct Memory Access via Datalink)
 * @details Provides safe, cross-platform DMA-like access to the MCU's physical memory 
 * using a sandbox whitelist mechanism to prevent illegal memory operations.
 */

#ifndef GMP_MEM_PERSP_U8_H
#define GMP_MEM_PERSP_U8_H

#include <core/dev/datalink.h>

#if GMP_PORT_DATA_SIZE_PER_BYTES != 1
#error "mem_presp_u8.h requires GMP_PORT_DATA_SIZE_PER_BYTES == 1"
#endif

/** @brief Read-only memory access permission */
#define GMP_MEM_PERM_RO 0x00
/** @brief Read-write memory access permission */
#define GMP_MEM_PERM_RW 0x01

/** @brief Command offset used to discover registered memory regions. */
#define GMP_MEM_DISCOVERY_CMD_OFFSET 1U

/**
 * @brief Memory Sandbox Region definition.
 * @details Defines a safe memory boundary for external access.
 */
typedef struct
{
    void* base_addr;      /**< Native physical base pointer (avoids link-time multiplication issues) */
    uint32_t byte_length; /**< Allowed access length in bytes */
    fast16_gt perm;       /**< Permission attribute (GMP_MEM_PERM_RO or GMP_MEM_PERM_RW) */
    const char* name;     /**< Optional English display name; NULL or empty selects a generated name */
} gmp_mem_region_t;

/**
 * @brief Memory Perspective Service Context (Class Context).
 * @details Manages the datalink binding and sandbox whitelist configuration.
 */
typedef struct
{
    gmp_datalink_t* dl_ctx;          /**< Bound datalink communication object */
    uint16_t base_cmd;               /**< Base command ID; base + 1 multiplexes writes and one-byte discovery queries */
    const gmp_mem_region_t* regions; /**< Array of registered sandbox whitelist regions */
    fast16_gt region_count;          /**< Number of regions in the whitelist array */
} gmp_mem_persp_t;

// =========================================================
// API Declarations
// =========================================================

/**
 * @brief Initialize the memory perspective service object.
 * @param ctx Pointer to the memory perspective context to initialize.
 * @param dl Pointer to the bound datalink communication object.
 * @param base_cmd The base command ID to be occupied by this service.
 * @param regions Pointer to the array of memory sandbox regions (whitelist).
 * @param region_count Number of regions in the whitelist array.
 */
void gmp_mem_persp_init(gmp_mem_persp_t* ctx, gmp_datalink_t* dl, uint16_t base_cmd, const gmp_mem_region_t* regions,
                        fast16_gt region_count);

/**
 * @brief Memory perspective reception callback.
 * @details Must be placed within the datalink RX_OK event handling chain.
 * @param ctx Pointer to the memory perspective context.
 * @return fast_gt Returns 1 if the command belongs to this object and is handled, 0 otherwise.
 */
fast_gt gmp_mem_persp_rx_cb(gmp_mem_persp_t* ctx);

#endif // GMP_MEM_PERSP_U8_H
