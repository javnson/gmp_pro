
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
// Sag - Swell

#include <ctl/component/intrinsic/protection/sag_swell.h>

void ctl_init_voltage_event_detector(ctl_voltage_event_detector_t* ved, parameter_gt sag_threshold,
                                     parameter_gt swell_threshold, parameter_gt sag_delay_ms,
                                     parameter_gt swell_delay_ms, parameter_gt nominal_freq, parameter_gt fs)
{
    // Initialize the SOGI. A damping factor of sqrt(2) is a common choice.
    const parameter_gt SOGI_DAMPING_FACTOR = 1.414f;
    ctl_init_discrete_sogi(&ved->sogi, SOGI_DAMPING_FACTOR, nominal_freq, fs);

    // Set thresholds
    ved->sag_threshold = float2ctrl(sag_threshold);
    ved->swell_threshold = float2ctrl(swell_threshold);

    // Convert delay from milliseconds to number of samples
    ved->sag_delay_samples = (uint32_t)(sag_delay_ms * fs / 1000.0f);
    ved->swell_delay_samples = (uint32_t)(swell_delay_ms * fs / 1000.0f);

    // Clear all states
    ctl_clear_voltage_event_detector(ved);
}

//////////////////////////////////////////////////////////////////////////
// ITOC protection
#include <ctl/component/intrinsic/protection/itoc_protection.h>

void ctl_init_trip_protector(ctl_trip_protector_t* prot, const ctrl_gt* source, parameter_gt level_ltd,
                             parameter_gt delay_ltd_ms, parameter_gt level_std, parameter_gt delay_std_ms,
                             parameter_gt level_inst, parameter_gt fs)
{
    prot->source = source;

    // Set trip levels
    prot->level_ltd = float2ctrl(level_ltd);
    prot->level_std = float2ctrl(level_std);
    prot->level_inst = float2ctrl(level_inst);

    // Convert delay times from milliseconds to sample counts
    prot->delay_ltd_samples = (uint32_t)(delay_ltd_ms * fs / 1000.0f);
    prot->delay_std_samples = (uint32_t)(delay_std_ms * fs / 1000.0f);

    // Clear initial states
    ctl_clear_trip_protector_fault(prot);
}


