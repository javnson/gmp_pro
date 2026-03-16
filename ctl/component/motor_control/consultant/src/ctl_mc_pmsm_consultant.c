
#include <gmp_core.h>

#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

/**
 * @brief Initializes and validates the PMSM physical model.
 * @details Derives critical operational constraints such as the characteristic 
 * current, which dictates the infinite-speed field weakening capability.
 */
void ctl_consultant_pmsm_init(ctl_consultant_pmsm_t* motor, uint32_t pp, parameter_gt rs, parameter_gt ld,
                              parameter_gt lq, parameter_gt flux)
{
    // 1. Strict Physical Validation (Gatekeeper)
    // A motor cannot have zero pole pairs, negative resistance, or negative inductance.
    gmp_base_assert(pp > 0);
    gmp_base_assert(rs > 0.0f);
    gmp_base_assert(ld > 0.0f);
    gmp_base_assert(lq > 0.0f);
    gmp_base_assert(flux > 0.0f);

    // 2. Base Parameter Assignment
    motor->pole_pairs = pp;
    motor->Rs = rs;
    motor->Ld = ld;
    motor->Lq = lq;
    motor->flux_linkage = flux;

    // 3. Intrinsic Properties Derivation
    motor->saliency_ratio = lq / ld;

    // Determine motor type: Surface (SPM) vs Interior (IPM)
    // We use a small epsilon to account for floating-point inaccuracies
    if ((lq - ld) > 1e-7f)
    {
        motor->is_ipm = 1;
        // Characteristic current is the center of the voltage limit ellipses.
        // It's the Id current required to completely cancel the permanent magnet flux.
        motor->char_current = flux / (lq - ld);
    }
    else
    {
        motor->is_ipm = 0;
        // For SPM, Ld == Lq. The characteristic current is theoretically infinite.
        // We set it to a very large number to prevent division by zero in downstream algorithms.
        motor->char_current = 1e6f;
    }
}
