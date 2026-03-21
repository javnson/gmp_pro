#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// ITOC protection
#include <ctl/component/intrinsic/protection/itoc_protection.h>

void ctl_init_trip_protector(ctl_trip_protector_t* prot, const ctrl_gt* source, parameter_gt level_ltd,
                             parameter_gt delay_ltd_ms, parameter_gt level_std, parameter_gt delay_std_ms,
                             parameter_gt level_inst, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0f); // ·À´ô±£»¤

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
