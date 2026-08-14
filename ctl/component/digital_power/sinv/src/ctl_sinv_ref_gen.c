
#include <ctl/math_block/gmp_math.h>

#include <ctl/component/digital_power/sinv/sinv_ref_gen.h>

/**
 * @brief Initializes the Reference Generator with safety limits and slope restrictions.
 * 
 * @param[out] gen Pointer to the generator instance.
 * @param[in]  i_max Maximum allowed peak current magnitude (e.g., 1.5 * rated peak).
 * @param[in]  v_mag_min Minimum voltage magnitude to allow power calculation (e.g., 0.1 pu).
 * @param[in]  p_slope Max rate of change for Active Power (Units/sec). Pass large value to disable.
 * @param[in]  q_slope Max rate of change for Reactive Power (Units/sec). Pass large value to disable.
 * @param[in]  fs System sampling frequency (Hz) to calculate per-step delta.
 */
void ctl_init_sinv_ref_gen(ctl_sinv_ref_gen_t* gen, parameter_gt i_max, parameter_gt v_mag_min, parameter_gt p_slope,
                           parameter_gt q_slope, parameter_gt fs)
{
    gen->i_max = real2ctrl(i_max);
    gen->i_max_sq = real2ctrl(i_max * i_max);
    gen->v_mag_min = real2ctrl(v_mag_min);

    // Initialize slope limiters (symmetric slopes assumed: +max, -max)
    ctl_init_slope_limiter(&gen->p_slope_lim, p_slope, -p_slope, fs);
    ctl_init_slope_limiter(&gen->q_slope_lim, q_slope, -q_slope, fs);

    gen->i_ref_inst = CTL_CTRL_CONST_ZERO;
    gen->i_p_mag = CTL_CTRL_CONST_ZERO;
    gen->i_q_mag = CTL_CTRL_CONST_ZERO;
    gen->flag_over_current = 0;
}
