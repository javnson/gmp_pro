/**
 * @file ctl_motor_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Encoder module

#include <ctl/component/motor_control/basic/encoder.h>

// Absolute rotation position encoder
//

void ctl_init_pos_encoder(pos_encoder_t* enc, uint16_t poles, uint32_t position_base)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = position_base;
}

void ctl_init_multiturn_pos_encoder(pos_multiturn_encoder_t* enc, uint16_t poles, uint32_t position_base)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = position_base;
}

void ctl_init_autoturn_pos_encoder(pos_autoturn_encoder_t* enc, uint16_t poles, uint32_t position_base)
{
    enc->encif.position = 0;
    enc->encif.elec_position = 0;
    enc->encif.revolutions = 0;

    enc->offset = 0;

    enc->pole_pairs = poles;
    enc->position_base = position_base;
}

//
// Speed position encoder
//

//void ctl_init_spd_encoder(spd_encoder_t *enc, parameter_gt speed_base)
//{
//    enc->speed_base = speed_base;
//    enc->encif.speed = 0;
//    enc->speed_krpm = 0;
//}

void ctl_init_spd_calculator(
    // speed calculator objects
    spd_calculator_t* sc,
    // link to a position encoder
    rotation_ift* pos_encif,
    // control law frequency, unit Hz
    parameter_gt control_law_freq,
    // division of control law frequency, unit ticks
    uint32_t speed_calc_div,
    // Speed per unit base value, unit rpm
    parameter_gt rated_speed_rpm,
    // just set this value to 1.
    // generally, speed_filter_fc approx to speed_calc freq divided by 5
    parameter_gt speed_filter_fc)
{
    uint32_t maximum_div = (uint32_t)rated_speed_rpm / 30;
    if (speed_calc_div < maximum_div)
    {
        maximum_div = speed_calc_div;
    }

    sc->old_position = 0;
    sc->encif.speed = 0;

    sc->scale_factor = float2ctrl(60.0f * control_law_freq / maximum_div / rated_speed_rpm);
    ctl_init_lp_filter(&sc->spd_filter, control_law_freq / maximum_div, speed_filter_fc);
    ctl_init_divider(&sc->div, maximum_div);

    sc->pos_encif = pos_encif;
}

void ctl_init_spd_calculator_elecpos(
    // speed calculator objects
    spd_calculator_t* sc,
    // link to a position encoder
    rotation_ift* pos_encif,
    // control law frequency, unit Hz
    parameter_gt control_law_freq,
    // division of control law frequency, unit ticks
    uint32_t speed_calc_div,
    // Speed per unit base value, unit rpm
    parameter_gt rated_speed_rpm,
    // pole pairs, if you pass a elec-angle,
    uint16_t pole_pairs,
    // just set this value to 1.
    // generally, speed_filter_fc approx to speed_calc freq divided by 5
    parameter_gt speed_filter_fc)
{
    uint32_t maximum_div = (uint32_t)rated_speed_rpm / 30;
    if (speed_calc_div < maximum_div)
    {
        maximum_div = speed_calc_div;
    }

    sc->old_position = 0;
    sc->encif.speed = 0;

    sc->scale_factor = float2ctrl(60.0f * control_law_freq / maximum_div / pole_pairs / rated_speed_rpm);
    ctl_init_lp_filter(&sc->spd_filter, control_law_freq / maximum_div, speed_filter_fc);
    ctl_init_divider(&sc->div, maximum_div);

    sc->pos_encif = pos_encif;
}

//////////////////////////////////////////////////////////////////////////
// const f module

#include <ctl/component/motor_control/basic/vf_generator.h>

void ctl_init_const_f_controller(ctl_const_f_controller* ctrl, parameter_gt frequency, parameter_gt isr_freq)
{
    // ctl_setup_ramp_gen(&ctrl->rg, float2ctrl(frequency / isr_freq), 1, 0);

    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, frequency, 1, 0);
}

// Const slope Frequency module

void ctl_init_const_slope_f_controller(
    // controller object
    ctl_slope_f_controller* ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // ISR frequency
    parameter_gt isr_freq)
{
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // init ramp frequency is 0
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    ctrl->target_frequency = frequency / isr_freq;

    ctl_init_slope_limiter(&ctrl->freq_slope, float2ctrl(freq_slope / isr_freq), -float2ctrl(freq_slope / isr_freq),
                           isr_freq);
}

// change target frequency
void ctl_set_slope_f_freq(
    // Const VF controller
    ctl_slope_f_controller* ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq)
{
    ctrl->target_frequency = float2ctrl(target_freq / isr_freq);
}

/**
 * @brief Initializes the Constant Slope Frequency Controller (PU).
 * @param[out] ctrl Pointer to the ctl_slope_f_pu_controller object.
 * @param[in] frequency The initial target frequency in Hz.
 * @param[in] freq_slope The maximum rate of frequency change in Hz/s.
 * @param[in] rated_krpm The motor rated speed in krpm (Base value source).
 * @param[in] pole_pairs The motor pole pairs (Base value source).
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 */
void ctl_init_const_slope_f_pu_controller(ctl_slope_f_pu_controller* ctrl, parameter_gt frequency,
                                          parameter_gt freq_slope, parameter_gt rated_krpm, parameter_gt pole_pairs,
                                          parameter_gt isr_freq)
{
    // 1. Calculate Base Frequency (Rated Electrical Hz)
    // f_base = (RPM * Poles) / 60
    // rated_krpm is in 1000 RPM
    parameter_gt base_freq_hz = (rated_krpm * 1000.0f * pole_pairs) / 60.0f;

    // Prevent divide by zero if user inputs bad data
    if (base_freq_hz < 0.1f)
        base_freq_hz = 1.0f;

    // 2. Clear Positions
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // 3. Init Ramp Generator
    // The slope will be updated in the step function, init with 0.
    // Range is [0, 1) for electrical angle.
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    // 4. Calculate Conversion Ratio
    // This ratio converts "1.0 pu frequency" into "step size per ISR tick"
    // step = (f_base / f_isr)
    ctrl->ratio_freq_pu_to_step = float2ctrl(base_freq_hz / isr_freq);

    // 5. Initialize Target Frequency in PU
    // target_pu = target_hz / base_hz
    ctrl->target_freq_pu = float2ctrl(frequency / base_freq_hz);

    // 6. Initialize Slope Limiter
    // The limiter needs to limit the change of PU per Tick.
    // Max Change (Hz/s) = freq_slope
    // Max Change (PU/s) = freq_slope / base_freq_hz
    ctrl_gt slope_limit_per_tick = float2ctrl(freq_slope / base_freq_hz);

    ctl_init_slope_limiter(&ctrl->freq_slope, slope_limit_per_tick, -slope_limit_per_tick, isr_freq);

    // Set initial output of limiter to current target (assuming instant start if needed, or 0)
    // Usually start from 0 for soft start
    ctrl->freq_slope.out = 0;
    ctrl->current_freq_pu = 0;
}

// VF controller

void ctl_init_const_vf_controller(
    // controller object
    ctl_const_vf_controller* ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // voltage range
    ctrl_gt voltage_bound,
    // Voltage Frequency constant
    // unit p.u./Hz, p.u.
    ctrl_gt voltage_over_frequency, ctrl_gt voltage_bias,
    // ISR frequency
    parameter_gt isr_freq)
{
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // init ramp frequency is 0
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    ctrl->target_frequency = frequency / isr_freq;
    ctrl->target_voltage = 0;

#if !defined CTRL_GT_IS_FIXED
    ctrl->v_over_f = float2ctrl(voltage_over_frequency * isr_freq);
#elif defined CTRL_GT_IS_FLOAT
    ctrl->v_over_f = float2ctrl(voltage_over_frequency * isr_freq / (2 ^ GLOBAL_Q));
#else
#error("The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED.")
#endif // CTRL_GT_IS_XXX
    ctrl->v_bias = voltage_bias;

    ctl_init_slope_limiter(&ctrl->freq_slope, float2ctrl(freq_slope / isr_freq), -float2ctrl(freq_slope / isr_freq),
                           isr_freq);

    ctl_init_saturation(&ctrl->volt_sat, voltage_bound, -voltage_bound);
}

// change target frequency
void ctl_set_const_vf_target_freq(
    // Const VF controller
    ctl_const_vf_controller* ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq)
{
    ctrl->target_frequency = float2ctrl(target_freq / isr_freq);
}

//////////////////////////////////////////////////////////////////////////
// Motor protection module

#include <ctl/component/motor_control/basic/mtr_protection.h>

void ctl_init_mtr_protect(ctl_mtr_protect_t* prot, parameter_gt fs)
{
    // Default safe values (User should overwrite these)
    prot->limit_ov_pu = float2ctrl(1.2f);
    prot->limit_uv_pu = float2ctrl(0.5f);
    prot->limit_oc_sq_pu = float2ctrl(1.5f * 1.5f);  // 1.5x Overload
    prot->limit_dev_sq_pu = float2ctrl(0.5f * 0.5f); // 0.5pu Deviation
    prot->limit_mtr_ot = 1000;                       // Raw ADC value
    prot->limit_inv_ot = 1000;

    // Default Filtering
    prot->limit_cnt_ov = (uint32_t)(fs / 2000); // Very Fast
    prot->limit_cnt_oc = (uint32_t)(fs / 2000); // Very Fast
    prot->limit_cnt_dev = (uint32_t)(fs / 5);   // Very Slow (e.g., 200ms @ 10kHz)

    prot->limit_cnt_uv = (uint32_t)(fs / 10); // Slow
    prot->limit_cnt_mtr_ot = (uint32_t)fs;
    prot->limit_cnt_inv_ot = (uint32_t)fs;

    // Ensure minimum count is 5 to prevent false triggering on noise
    if (prot->limit_cnt_ov <= 5)
        prot->limit_cnt_ov = 5;
    if (prot->limit_cnt_oc <= 5)
        prot->limit_cnt_oc = 5;

    // Enable all protections by default (Mask = 0)
    prot->error_mask.all = 0;

    ctl_clear_mtr_protect(prot);
}

void ctl_attach_mtr_protect_port(ctl_mtr_protect_t* prot, ctrl_gt* u_dc, ctl_vector2_t* i_meas, ctl_vector2_t* i_ref,
                                 adc_ift* mtr_temp, adc_ift* inv_temp)
{
    prot->ptr_udc = u_dc;
    prot->ptr_idq = i_meas;
    prot->ptr_ref = i_ref;
    prot->ptr_mtr_temp = mtr_temp;
    prot->ptr_inv_temp = inv_temp;
}


/**
 * @brief Main Loop Protection Dispatch. 
 * This calling frequency of this function should be less than 1kHz.
 * @return 1 if fault active, 0 if safe
 */
fast_gt ctl_dispatch_mtr_protect_slow(ctl_mtr_protect_t* prot)
{
    // 1. Latch Check
    if ((prot->error_code.all & (~prot->error_mask.all)) != MTR_PROT_NONE)
        return 1;

    // 2. Check Under Voltage
    ctrl_gt u_dc = *(prot->ptr_udc);
    if (ctl_mtr_protect_debounce(ctl_is_less(u_dc, prot->limit_uv_pu), &prot->cnt_uv, prot->limit_cnt_uv))
    {
        prot->error_code.bit.under_voltage = 1;

        if (!prot->error_mask.bit.under_voltage)
        {
            return 1;
        }
    }

    // 3. Check Motor Over Temp
    if (prot->ptr_mtr_temp)
    {
        // Assuming ptr_mtr_temp->value holds the data
        adc_gt temp = prot->ptr_mtr_temp->value;
        if (ctl_mtr_protect_debounce(ctl_is_greater_ot(temp, prot->limit_mtr_ot), &prot->cnt_mtr_ot,
                                     prot->limit_cnt_mtr_ot))
        {
            prot->error_code.bit.mtr_over_temp = 1;

            if (!prot->error_mask.bit.mtr_over_temp)
            {
                return 1;
            }
        }
    }

    // 4. Check Inverter Over Temp
    if (prot->ptr_inv_temp)
    {
        adc_gt temp = prot->ptr_inv_temp->value; // Fixed: was reading mtr_temp
        if (ctl_mtr_protect_debounce(ctl_is_greater_ot(temp, prot->limit_inv_ot), &prot->cnt_inv_ot,
                                     prot->limit_cnt_inv_ot))
        {
            prot->error_code.bit.inv_over_temp = 1;

            if (!prot->error_mask.bit.inv_over_temp)
            {
                return 1;
            }
        }
    }

    return 0;
}
