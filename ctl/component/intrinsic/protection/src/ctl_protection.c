
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Protection
#include <ctl/component/intrinsic/protection/protection.h>

void ctl_init_protection_monitor(ctl_protection_monitor_t* mon, const ctl_protection_item_t* item_set,
                                 uint32_t num_items)
{
    mon->item_set = item_set;
    mon->num_items = num_items;
    mon->fault_index = -1; // Initialize with no fault
}

//////////////////////////////////////////////////////////////////////////
// Protection slot

#include <ctl/component/intrinsic/protection/protection_slot.h>

/**
 * @brief Initializes a single-threshold protection node.
 */
void ctl_init_prot_single(ctl_prot_single_t* node, uint32_t status_bit, parameter_gt threshold, uint16_t trip_limit)
{
    node->is_enabled = 0;
    node->threshold = float2ctrl(threshold);
    node->trip_limit_count = trip_limit;
    node->current_count = 0;
    node->status_bit = status_bit;
    node->fault_record_val = float2ctrl(0.0f);
}

/**
 * @brief Initializes a window protection node.
 */
void ctl_init_prot_window(ctl_prot_window_t* node, uint32_t status_bit, parameter_gt sup, parameter_gt inf,
                          uint16_t trip_limit)
{
    node->is_enabled = 0;
    node->sup = float2ctrl(sup);
    node->inf = float2ctrl(inf);
    node->trip_limit_count = trip_limit;
    node->current_count = 0;
    node->status_bit = status_bit;
    node->fault_record_val = float2ctrl(0.0f);
}

/**
 * @brief Initializes a vector protection node.
 */
void ctl_init_prot_vector(ctl_prot_vector_t* node, uint32_t status_bit, parameter_gt threshold, uint16_t trip_limit)
{
    node->is_enabled = 0;
    node->threshold = float2ctrl(threshold);
    node->threshold_sq = ctl_mul(node->threshold, node->threshold); // Pre-calc!
    node->trip_limit_count = trip_limit;
    node->current_count = 0;
    node->status_bit = status_bit;
    ctl_vector2_clear(&node->fault_record_val);
}

/**
 * @brief Initializes a thermal/stress protection node.
 * @note No trip_limit_count is used here, as the time aspect is embedded in the integral.
 */
void ctl_init_prot_thermal(ctl_prot_thermal_t* node, uint32_t status_bit, parameter_gt rated_value,
                           parameter_gt thermal_limit)
{
    node->is_enabled = 0;
    node->rated_value = float2ctrl(rated_value);
    node->thermal_limit = float2ctrl(thermal_limit);
    node->thermal_acc = float2ctrl(0.0f);
    node->status_bit = status_bit;
    node->fault_record_val = float2ctrl(0.0f);
}
