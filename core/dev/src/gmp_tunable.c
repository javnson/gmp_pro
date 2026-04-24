#include <core/dev/tunable.h>

#define GMP_BACKDOOR_BASE_ID 0xFFF0
#define GMP_BACKDOOR_MAX_CH  4

// Internal reference to the datalink layer for sending ACKs
static gmp_datalink_t* p_dl_ctx = NULL;

// Dynamic RAM array for backdoor channels
static gmp_var_entry_t gmp_backdoor_dict[GMP_BACKDOOR_MAX_CH];

#ifndef GMP_ENABLE_STATIC_VARIABLE_DICT
const gmp_var_entry_t gmp_static_dict[];
const uint16_t gmp_static_dict_size;
#endif // GMP_ENABLE_STATIC_VARIABLE_DICT

#ifndef GMP_ENABLE_STATIC_MEM_DICT
const gmp_mem_region_t gmp_mem_whitelist[];
const uint16_t gmp_mem_whitelist_size;
#endif // GMP_ENABLE_STATIC_MEM_DICT

// =========================================================
// INTERNAL HELPERS
// =========================================================

/**
 * @brief  Internal helper to fetch a dictionary entry by its ID.
 * @return Pointer to the entry, or NULL if ID is invalid/unconfigured.
 */
static const gmp_var_entry_t* get_dict_entry(uint16_t var_id) {
    // 1. Check Danger Zone: Backdoor Channels
    if (var_id >= GMP_BACKDOOR_BASE_ID && var_id < (GMP_BACKDOOR_BASE_ID + GMP_BACKDOOR_MAX_CH)) {
        uint16_t ch = var_id - GMP_BACKDOOR_BASE_ID;
        if (gmp_backdoor_dict[ch].var_ptr != NULL) {
            return &gmp_backdoor_dict[ch];
        }
        return NULL; 
    }
    
    // 2. Check Static ROM Dictionary
    if (var_id < gmp_static_dict_size) {
        if (gmp_static_dict[var_id].var_ptr != NULL) {
            return &gmp_static_dict[var_id];
        }
    }
    return NULL;
}

// =========================================================
// COMMAND HANDLERS
// =========================================================

static void process_var_info_req(uint16_t target_id, const data_gt* payload, size_gt len) {
    if (len < 2) return;

    uint16_t var_id = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);
    const gmp_var_entry_t* entry = get_dict_entry(var_id);

    if (entry != NULL) {
        data_gt tx_payload[8];
        uint32_t addr = (uint32_t)((uintptr_t)entry->var_ptr);

        tx_payload[0] = var_id & 0xFF;
        tx_payload[1] = (var_id >> 8) & 0xFF;
        // Endian-safe packing of physical 32-bit address
        tx_payload[2] = addr & 0xFF;
        tx_payload[3] = (addr >> 8) & 0xFF;
        tx_payload[4] = (addr >> 16) & 0xFF;
        tx_payload[5] = (addr >> 24) & 0xFF;
        tx_payload[6] = (data_gt)entry->var_type;
        tx_payload[7] = (data_gt)entry->access;

        gmp_datalink_send(p_dl_ctx, target_id, GMP_CMD_VAR_INFO_ACK, tx_payload, 8);
    }
}

static void process_var_read_req(uint16_t target_id, const data_gt* payload, size_gt len) {
    if (len < 2) return;

    uint16_t var_id = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);
    const gmp_var_entry_t* entry = get_dict_entry(var_id);

    if (entry != NULL) {
        data_gt tx_payload[6]; // ID(2) + max Data(4)
        tx_payload[0] = var_id & 0xFF;
        tx_payload[1] = (var_id >> 8) & 0xFF;
        
        size_gt tx_len = 2;

        // Strict type punning and extraction to avoid alignment faults on DSP
        switch (entry->var_type) {
            case GMP_VAR_TYPE_UINT16:
            case GMP_VAR_TYPE_INT16: {
                uint16_t val = *((uint16_t*)entry->var_ptr);
                tx_payload[tx_len++] = val & 0xFF;
                tx_payload[tx_len++] = (val >> 8) & 0xFF;
                break;
            }
            case GMP_VAR_TYPE_UINT32:
            case GMP_VAR_TYPE_INT32:
            case GMP_VAR_TYPE_FLOAT: {
                uint32_t val = *((uint32_t*)entry->var_ptr);
                tx_payload[tx_len++] = val & 0xFF;
                tx_payload[tx_len++] = (val >> 8) & 0xFF;
                tx_payload[tx_len++] = (val >> 16) & 0xFF;
                tx_payload[tx_len++] = (val >> 24) & 0xFF;
                break;
            }
        }
        gmp_datalink_send(p_dl_ctx, target_id, GMP_CMD_VAR_READ_ACK, tx_payload, tx_len);
    }
}

static void process_var_write_req(uint16_t target_id, const data_gt* payload, size_gt len) {
    if (len < 2) return;

    uint16_t var_id = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8);
    const gmp_var_entry_t* entry = get_dict_entry(var_id);
    
    gmp_var_status_t status = GMP_VAR_STATUS_OK;

    if (entry == NULL) {
        status = GMP_VAR_STATUS_ERR_NO_ID;
    } else if (entry->access == GMP_VAR_ACCESS_RO) {
        status = GMP_VAR_STATUS_ERR_RO;
    } else {
        // Perform Endian-safe combination and memory write
        switch (entry->var_type) {
            case GMP_VAR_TYPE_UINT16:
            case GMP_VAR_TYPE_INT16: {
                if (len < 4) { status = GMP_VAR_STATUS_ERR_BAD_LEN; break; }
                uint16_t val = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8);
                *((uint16_t*)entry->var_ptr) = val;
                break;
            }
            case GMP_VAR_TYPE_UINT32:
            case GMP_VAR_TYPE_INT32:
            case GMP_VAR_TYPE_FLOAT: {
                if (len < 6) { status = GMP_VAR_STATUS_ERR_BAD_LEN; break; }
                uint32_t val = (payload[2] & 0xFF) | ((payload[3] & 0xFF) << 8) | 
                               ((payload[4] & 0xFF) << 16) | ((payload[5] & 0xFF) << 24);
                *((uint32_t*)entry->var_ptr) = val;
                break;
            }
        }
    }

    // Always reply with an ACK status
    data_gt tx_payload[3];
    tx_payload[0] = var_id & 0xFF;
    tx_payload[1] = (var_id >> 8) & 0xFF;
    tx_payload[2] = (data_gt)status;
    gmp_datalink_send(p_dl_ctx, target_id, GMP_CMD_VAR_WRITE_ACK, tx_payload, 3);
}

static void process_backdoor_cfg(uint16_t target_id, const data_gt* payload, size_gt len) {
    if (len < 6) return;

    uint8_t ch = payload[0] & 0xFF;
    if (ch >= GMP_BACKDOOR_MAX_CH) return;

    // Reconstruct 32-bit physical address
    uint32_t addr = (payload[1] & 0xFF) | ((payload[2] & 0xFF) << 8) | 
                    ((payload[3] & 0xFF) << 16) | ((payload[4] & 0xFF) << 24);
    
    uint8_t type = payload[5] & 0xFF;

    // Mount to RAM dictionary (Always grant RW to backdoor!)
    gmp_backdoor_dict[ch].var_ptr  = (void*)((uintptr_t)addr);
    gmp_backdoor_dict[ch].var_type = (gmp_var_type_t)type;
    gmp_backdoor_dict[ch].access   = GMP_VAR_ACCESS_RW; 

    data_gt tx_payload[2] = {ch, GMP_VAR_STATUS_OK};
    gmp_datalink_send(p_dl_ctx, target_id, GMP_CMD_BACKDOOR_ACK, tx_payload, 2);
}

// =========================================================
// API IMPLEMENTATIONS
// =========================================================

void gmp_tunable_init(gmp_datalink_t* dl_ctx) {
    p_dl_ctx = dl_ctx;
    memset(gmp_backdoor_dict, 0, sizeof(gmp_backdoor_dict));
}

void gmp_tunable_rx_callback(uint16_t target_id, uint16_t cmd, const data_gt* payload, size_gt len) {
    // Top-level Dispatcher
    switch(cmd) {
        case GMP_CMD_VAR_INFO_REQ:
            process_var_info_req(target_id, payload, len);
            break;
        case GMP_CMD_VAR_READ_REQ:
            process_var_read_req(target_id, payload, len);
            break;
        case GMP_CMD_VAR_WRITE_REQ:
            process_var_write_req(target_id, payload, len);
            break;
        case GMP_CMD_BACKDOOR_CFG:
            process_backdoor_cfg(target_id, payload, len);
            break;
        case GMP_CMD_MEM_READ_REQ:
            process_mem_read_req(target_id, payload, len);
            break;
        case GMP_CMD_MEM_WRITE_REQ:
            process_process_mem_write_req(target_id, payload, len);
            break;
        default:
            // Future extensions (Memory read, Simulation step) will go here
            break;
    }
}

// =========================================================
// MEMORY PROTECTION UNIT (SOFTWARE MPU)
// =========================================================

/**
 * @brief  Validates if a requested memory block is strictly within the whitelist.
 * @param  req_addr The requested starting physical address.
 * @param  req_len  The requested length.
 * @param  is_write Flag indicating if the request is a write operation (requires RW attr).
 * @return gmp_mem_status_t Status of the validation (OK, OOB, RO).
 */
static gmp_mem_status_t check_mem_access(uint32_t req_addr, uint32_t req_len, fast_gt is_write) {
    uint16_t i;
    for (i = 0; i < gmp_mem_whitelist_size; i++) {
        uint32_t region_start = gmp_mem_whitelist[i].start_addr;
        uint32_t region_end   = region_start + gmp_mem_whitelist[i].size;
        
        // Check if the requested block is COMPLETELY inside this region
        if (req_addr >= region_start && (req_addr + req_len) <= region_end) {
            if (is_write && gmp_mem_whitelist[i].attr == GMP_MEM_ATTR_RO) {
                return GMP_MEM_STATUS_ERR_RO;
            }
            return GMP_MEM_STATUS_OK;
        }
    }
    return GMP_MEM_STATUS_ERR_OOB;
}

// =========================================================
// COMMAND HANDLERS (MEMORY ACCESS)
// =========================================================

static void process_mem_read_req(uint16_t target_id, const data_gt* payload, size_gt len) {
    if (len < 6) return; // Addr(4) + Len(2)

    uint32_t req_addr = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8) | 
                        ((payload[2] & 0xFF) << 16) | ((payload[3] & 0xFF) << 24);
    uint16_t req_len  = (payload[4] & 0xFF) | ((payload[5] & 0xFF) << 8);

    gmp_mem_status_t status = check_mem_access(req_addr, req_len, 0);

    // Limit length to MTU payload limits (Need space for Addr(4) + Len(2) + Status(1))
    if (req_len > (GMP_DL_MTU - 7)) {
        status = GMP_MEM_STATUS_ERR_MTU;
        req_len = 0; 
    }

    // Prepare ACK Payload: [Status(1)] [Addr(4)] [Len(2)] [Data...]
    data_gt tx_payload[GMP_DL_MTU];
    tx_payload[0] = (data_gt)status;
    tx_payload[1] = payload[0];
    tx_payload[2] = payload[1];
    tx_payload[3] = payload[2];
    tx_payload[4] = payload[3];
    tx_payload[5] = payload[4];
    tx_payload[6] = payload[5];

    size_gt tx_idx = 7;

    if (status == GMP_MEM_STATUS_OK) {
        // Safe Memory Copy (Avoids unaligned access faults on strict architectures)
        // Note: Cast to uintptr_t ensures safe integer-to-pointer conversion
        uint8_t* ptr = (uint8_t*)((uintptr_t)req_addr);
        uint16_t i;
        for (i = 0; i < req_len; i++) {
            tx_payload[tx_idx++] = (data_gt)(ptr[i] & 0xFF);
        }
    }

    gmp_datalink_send(p_dl_ctx, target_id, GMP_CMD_MEM_READ_ACK, tx_payload, tx_idx);
}

static void process_mem_write_req(uint16_t target_id, const data_gt* payload, size_gt len) {
    if (len < 6) return;

    uint32_t req_addr = (payload[0] & 0xFF) | ((payload[1] & 0xFF) << 8) | 
                        ((payload[2] & 0xFF) << 16) | ((payload[3] & 0xFF) << 24);
    uint16_t req_len  = (payload[4] & 0xFF) | ((payload[5] & 0xFF) << 8);

    gmp_mem_status_t status = check_mem_access(req_addr, req_len, 1);

    // Verify if the payload actually contains the data it claims to have
    if (len < (6 + req_len)) {
        status = GMP_MEM_STATUS_ERR_MTU;
    }

    if (status == GMP_MEM_STATUS_OK) {
        // Safe Memory Copy (Unaligned safe)
        uint8_t* ptr = (uint8_t*)((uintptr_t)req_addr);
        uint16_t i;
        for (i = 0; i < req_len; i++) {
            ptr[i] = (uint8_t)(payload[6 + i] & 0xFF);
        }
    }

    // Prepare ACK Payload: [Status(1)] [Addr(4)] [Len(2)]
    data_gt tx_payload[7];
    tx_payload[0] = (data_gt)status;
    tx_payload[1] = payload[0];
    tx_payload[2] = payload[1];
    tx_payload[3] = payload[2];
    tx_payload[4] = payload[3];
    tx_payload[5] = payload[4];
    tx_payload[6] = payload[5];

    gmp_datalink_send(p_dl_ctx, target_id, GMP_CMD_MEM_WRITE_ACK, tx_payload, 7);
}
