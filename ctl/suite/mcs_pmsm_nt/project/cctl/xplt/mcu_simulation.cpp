/**
 * @file mcu_simulation.cpp
 * @brief MCU peripheral behavior for the direct PMSM CCTL simulation.
 */

#include <mcu_simulation.hpp>

#include <csp.general.h>
#include <xplt.peripheral.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mcs::cctl_xplt
{

::cctl::ti_adc_config<sim_real_gt> mcu_simulation::make_adc_config()
{
    ::cctl::ti_adc_config<sim_real_gt> config;
    config.resolution_bits = CCTL_SIM_ADC_RESOLUTION_BITS;
    config.reference_voltage_v = sim_real_gt(CCTL_SIM_ADC_REFERENCE_V);
    return config;
}

::cctl::ti_epwm_config<sim_real_gt>
mcu_simulation::make_epwm_config(bool adc_trigger)
{
    ::cctl::ti_epwm_config<sim_real_gt> config;
    config.time_base_clock_hz = sim_real_gt(CCTL_SIM_EPWM_TBCLK_HZ);
    config.period_count = CCTL_SIM_EPWM_PERIOD_COUNT;
    config.rising_edge_delay_count = CCTL_SIM_EPWM_DBRED_COUNT;
    config.falling_edge_delay_count = CCTL_SIM_EPWM_DBFED_COUNT;
    config.upper_active_above_compare = true;
    config.adc_trigger_event =
        adc_trigger ? ::cctl::ti_epwm_trigger_event::compare_b_up
                    : ::cctl::ti_epwm_trigger_event::disabled;
    config.adc_trigger_compare_count = CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT;
    return config;
}

mcu_simulation::mcu_simulation()
    : adc_(make_adc_config()), eqep_(CCTL_SIM_EQEP_COUNTS_PER_REV),
      epwm_{::cctl::ti_epwm<sim_real_gt>(make_epwm_config(true)),
            ::cctl::ti_epwm<sim_real_gt>(make_epwm_config()),
            ::cctl::ti_epwm<sim_real_gt>(make_epwm_config())}
{
}

void mcu_simulation::initialize()
{
    adc_.initialize(make_adc_config());
    eqep_.initialize(CCTL_SIM_EQEP_COUNTS_PER_REV);
    epwm_[0].initialize(make_epwm_config(true));
    epwm_[1].initialize(make_epwm_config());
    epwm_[2].initialize(make_epwm_config());
    if (!verify_peripheral_models())
        throw std::runtime_error(
            "SDPE-configured TI peripheral self-test failed");
}

epwm_outputs mcu_simulation::sample_epwm(
    std::uint64_t absolute_tbclk_count)
{
    return {epwm_[0].sample_time_base_count(absolute_tbclk_count),
            epwm_[1].sample_time_base_count(absolute_tbclk_count),
            epwm_[2].sample_time_base_count(absolute_tbclk_count)};
}

void mcu_simulation::stage_adc_inputs(const adc_pin_voltages &inputs)
{
    adc_.set_input_voltage(CCTL_ADC_UDC, inputs.dc_link_voltage);
    adc_.set_input_voltage(CCTL_ADC_UA, inputs.phase_voltage[0]);
    adc_.set_input_voltage(CCTL_ADC_UB, inputs.phase_voltage[1]);
    adc_.set_input_voltage(CCTL_ADC_UC, inputs.phase_voltage[2]);
    adc_.set_input_voltage(CCTL_ADC_IA, inputs.phase_current[0]);
    adc_.set_input_voltage(CCTL_ADC_IB, inputs.phase_current[1]);
    adc_.set_input_voltage(CCTL_ADC_IC, inputs.phase_current[2]);
}

void mcu_simulation::trigger_adc(
    const std::function<void()> &interrupt_handler)
{
    adc_.trigger(interrupt_handler);
}

void mcu_simulation::transfer_adc_results_to_controller() const
{
    for (std::size_t channel = 0U; channel < CCTL_ADC_COUNT; ++channel)
        cctl_adc_result[channel] = static_cast<adc_gt>(adc_.result(channel));
}

void mcu_simulation::sample_encoder(sim_real_gt mechanical_angle_rad)
{
    cctl_encoder_position = eqep_.sample_mechanical_angle(mechanical_angle_rad);
}

void mcu_simulation::acknowledge_adc_interrupt() noexcept
{
    adc_.acknowledge_interrupt();
}

void mcu_simulation::transfer_pwm_from_controller() noexcept
{
    const bool enabled = csp_cctl_output_is_enabled() != 0;
    for (std::size_t phase = 0U; phase < epwm_.size(); ++phase)
    {
        epwm_[phase].set_compare_a(cctl_pwm_compare[phase]);
        epwm_[phase].set_enabled(enabled);
    }
}

bool mcu_simulation::adc_interrupt_pending() const noexcept
{
    return adc_.interrupt_pending();
}

std::uint64_t mcu_simulation::adc_trigger_count() const noexcept
{
    return adc_.trigger_count();
}

bool mcu_simulation::output_enabled() const noexcept
{
    return std::all_of(epwm_.begin(), epwm_.end(), [](const auto &module) {
        return module.enabled();
    });
}

bool mcu_simulation::all_low_sides_conducting(
    const epwm_outputs &outputs) noexcept
{
    return std::all_of(outputs.begin(), outputs.end(), [](const auto &phase) {
        return phase.upper == 0U && phase.lower != 0U;
    });
}

bool mcu_simulation::verify_peripheral_models()
{
    ::cctl::ti_adc<sim_real_gt, 1U> adc(make_adc_config());
    bool adc_interrupt_called = false;
    adc.set_input_voltage(0U, sim_real_gt(CCTL_SIM_ADC_REFERENCE_V) /
                                  sim_real_gt(2));
    adc.trigger([&adc_interrupt_called] { adc_interrupt_called = true; });
    if (!adc_interrupt_called || !adc.interrupt_pending() ||
        adc.trigger_count() != 1U ||
        adc.result(0U) != (adc.maximum_code() + 1U) / 2U)
        return false;
    adc.acknowledge_interrupt();
    if (adc.interrupt_pending())
        return false;

    ::cctl::ti_eqep<sim_real_gt> eqep(CCTL_SIM_EQEP_COUNTS_PER_REV);
    if (eqep.sample_mechanical_angle(
            sim_real_gt(0.5L * 3.14159265358979323846L)) !=
        CCTL_SIM_EQEP_COUNTS_PER_REV / 4U)
        return false;

    ::cctl::ti_epwm<sim_real_gt> time_epwm(make_epwm_config(true));
    ::cctl::ti_epwm<sim_real_gt> count_epwm(make_epwm_config(true));
    time_epwm.set_enabled(true);
    count_epwm.set_enabled(true);
    time_epwm.set_compare_a(CCTL_SIM_EPWM_PERIOD_COUNT / 2U);
    count_epwm.set_compare_a(CCTL_SIM_EPWM_PERIOD_COUNT / 2U);
    std::size_t adc_triggers = 0U;
    for (std::size_t index = 0U;
         index < 2U * (CCTL_SIM_EPWM_PERIOD_COUNT + 1U); ++index)
    {
        const auto timed = time_epwm.sample(
            sim_real_gt(index) / sim_real_gt(CCTL_SIM_EPWM_TBCLK_HZ));
        const auto counted = count_epwm.sample_time_base_count(index);
        if (timed.upper != counted.upper || timed.lower != counted.lower ||
            timed.adc_trigger != counted.adc_trigger ||
            (timed.upper != 0U && timed.lower != 0U))
            return false;
        if (timed.adc_trigger)
        {
            ++adc_triggers;
            if (timed.upper != 0U || timed.lower == 0U)
                return false;
        }
    }
    return adc_triggers == 1U &&
           std::abs(time_epwm.switching_frequency_hz() -
                    sim_real_gt(CCTL_SIM_CONTROL_FREQUENCY_HZ)) <
               sim_real_gt(1.0e-9L);
}

} // namespace mcs::cctl_xplt
