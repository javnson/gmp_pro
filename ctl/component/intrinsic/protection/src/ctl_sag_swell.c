
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Sag - Swell

#include <ctl/component/intrinsic/protection/sag_swell.h>

void ctl_init_voltage_event_detector(ctl_voltage_event_detector_t* ved, parameter_gt sag_threshold,
                                     parameter_gt swell_threshold, parameter_gt sag_delay_ms,
                                     parameter_gt swell_delay_ms, parameter_gt nominal_freq, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0f); // ·À´ô±£»¤

    // Initialize the SOGI. A damping factor of sqrt(2) is a common choice.
    const parameter_gt SOGI_DAMPING_FACTOR = CTL_PARAM_CONST_SQRT2;
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
