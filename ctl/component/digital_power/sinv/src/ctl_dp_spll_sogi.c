
#include <ctl/component/digital_power/single_phase/spll.h>

void ctl_init_single_phase_pll(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt Ti, parameter_gt fc,
                               parameter_gt fg, parameter_gt fs)
{
    // Clear the SPLL structure to ensure a clean state.
    ctl_clear_single_phase_pll(spll);

    // Initialize a discrete SOGI for generating alpha-beta orthogonal signals.
    ctl_init_discrete_sogi(&spll->sogi, 0.5, fg, fs);

    // Initialize a low-pass filter for the q-axis component (Uq) of the SOGI output.
    ctl_init_lp_filter(&spll->filter_uq, fs, fc);

    // Initialize the PI controller for the phase-locking loop.
    ctl_init_pid_Tmode(&spll->spll_ctrl, gain, Ti, 0, fs);

    // Pre-calculate the normalized grid frequency as a feed-forward term for the VCO.
    spll->freq_sf = float2ctrl(fg / fs);
}

void ctl_init_single_phase_pll_T(ctl_single_phase_pll* spll, parameter_gt gain, parameter_gt ki, parameter_gt fc,
                                 parameter_gt fg, parameter_gt fs)
{
    // Clear the SPLL structure to ensure a clean state.
    ctl_clear_single_phase_pll(spll);

    // Initialize a discrete SOGI for generating alpha-beta orthogonal signals.
    ctl_init_discrete_sogi(&spll->sogi, 0.5, fg, fs);

    // Initialize a low-pass filter for the q-axis component (Uq) of the SOGI output.
    ctl_init_lp_filter(&spll->filter_uq, fs, fc);

    // Initialize the PI controller for the phase-locking loop.
    ctl_init_pid(&spll->spll_ctrl, gain, ki, 0, fs);

    // Pre-calculate the normalized grid frequency as a feed-forward term for the VCO.
    spll->freq_sf = float2ctrl(fg / fs);
}
