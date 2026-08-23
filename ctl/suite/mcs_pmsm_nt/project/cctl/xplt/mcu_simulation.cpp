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

mcu_simulation::mcu_simulation()
    : adc_(adc_type::make(CCTL_SIM_ADC_RESOLUTION_BITS,
                          sim_real_gt(CCTL_SIM_ADC_REFERENCE_V))),
      eqep_(eqep_type::make(CCTL_SIM_EQEP_COUNTS_PER_REV)),
      epwm_{epwm_type::make(
                sim_real_gt(CCTL_SIM_EPWM_TBCLK_HZ),
                CCTL_SIM_EPWM_PERIOD_COUNT, CCTL_SIM_EPWM_DBRED_COUNT,
                CCTL_SIM_EPWM_DBFED_COUNT, true,
                ::cctl::ti_epwm_trigger_event::compare_b_up,
                CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT),
            epwm_type::make(
                sim_real_gt(CCTL_SIM_EPWM_TBCLK_HZ),
                CCTL_SIM_EPWM_PERIOD_COUNT, CCTL_SIM_EPWM_DBRED_COUNT,
                CCTL_SIM_EPWM_DBFED_COUNT),
            epwm_type::make(
                sim_real_gt(CCTL_SIM_EPWM_TBCLK_HZ),
                CCTL_SIM_EPWM_PERIOD_COUNT, CCTL_SIM_EPWM_DBRED_COUNT,
                CCTL_SIM_EPWM_DBFED_COUNT)}
{
}

void mcu_simulation::initialize()
{
    adc_.reset();
    eqep_.reset();
    for (epwm_type &module : epwm_)
        module.reset();
    outputs_ = {};

    if (!verify_peripheral_models())
        throw std::runtime_error(
            "SDPE-configured TI peripheral self-test failed");
}

void mcu_simulation::set_adc_interrupt_handler(
    adc_type::interrupt_handler_type handler, void *context) noexcept
{
    adc_.set_interrupt_handler(handler, context);
}

const epwm_outputs &mcu_simulation::control_outputs(
    std::uint64_t absolute_tbclk_count)
{
    outputs_ = {epwm_[0].sample_time_base_count(absolute_tbclk_count),
                epwm_[1].sample_time_base_count(absolute_tbclk_count),
                epwm_[2].sample_time_base_count(absolute_tbclk_count)};
    return outputs_;
}

bool mcu_simulation::control_inputs(const adc_pin_voltages &inputs,
                                    sim_real_gt mechanical_angle_rad)
{
    /* ePWM decides whether the control transaction exists at this step. */
    if (!outputs_[0].adc_trigger)
        return false;
    if (outputs_[1].adc_trigger || outputs_[2].adc_trigger)
        throw std::runtime_error("more than one ePWM module drives ADC SOC");

    const std::array<sim_real_gt, 8U> voltages{
        inputs.dc_link_voltage, inputs.phase_voltage[0],
        inputs.phase_voltage[1], inputs.phase_voltage[2],
        inputs.phase_current[0], inputs.phase_current[1],
        inputs.phase_current[2], sim_real_gt(0)};
    adc_.set_input_voltages(voltages);

    /* Prepare MCU input registers before entering the synchronous ADC ISR. */
    eqep_.sample_to(mechanical_angle_rad, cctl_encoder_position);
    adc_.trigger_and_transfer(cctl_adc_result, CCTL_ADC_COUNT);

    /* gmp_base_ctl_step() has returned; publish its new PWM register values. */
    write_epwm_outputs_after_isr();
    adc_.acknowledge_interrupt();
    return true;
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

void mcu_simulation::write_epwm_outputs_after_isr() noexcept
{
    const bool enabled = csp_cctl_output_is_enabled() != 0;
    for (std::size_t phase = 0U; phase < epwm_.size(); ++phase)
        epwm_[phase].apply_control(cctl_pwm_compare[phase], enabled);
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
    auto adc = ::cctl::ti_adc<sim_real_gt, 1U>::make(
        CCTL_SIM_ADC_RESOLUTION_BITS,
        sim_real_gt(CCTL_SIM_ADC_REFERENCE_V));
    struct adc_test_context
    {
        bool interrupt_called{};
        std::uint32_t *transferred_result{};
    };
    std::uint32_t transferred_result = 0U;
    adc_test_context adc_context{false, &transferred_result};
    adc.set_interrupt_handler(
        [](void *context) {
            auto &test = *static_cast<adc_test_context *>(context);
            test.interrupt_called = *test.transferred_result != 0U;
        },
        &adc_context);
    adc.set_input_voltage(0U, sim_real_gt(CCTL_SIM_ADC_REFERENCE_V) /
                                  sim_real_gt(2));
    adc.trigger_and_transfer(&transferred_result, 1U);
    if (!adc_context.interrupt_called || !adc.interrupt_pending() ||
        adc.trigger_count() != 1U ||
        transferred_result != (adc.maximum_code() + 1U) / 2U ||
        adc.result(0U) != (adc.maximum_code() + 1U) / 2U)
        return false;
    adc.acknowledge_interrupt();
    if (adc.interrupt_pending())
        return false;

    auto eqep = ::cctl::ti_eqep<sim_real_gt>::make(
        CCTL_SIM_EQEP_COUNTS_PER_REV);
    if (eqep.sample_mechanical_angle(
            sim_real_gt(0.5L * 3.14159265358979323846L)) !=
        CCTL_SIM_EQEP_COUNTS_PER_REV / 4U)
        return false;

    auto time_epwm = ::cctl::ti_epwm<sim_real_gt>::make(
        sim_real_gt(CCTL_SIM_EPWM_TBCLK_HZ), CCTL_SIM_EPWM_PERIOD_COUNT,
        CCTL_SIM_EPWM_DBRED_COUNT, CCTL_SIM_EPWM_DBFED_COUNT, true,
        ::cctl::ti_epwm_trigger_event::compare_b_up,
        CCTL_SIM_ADC_TRIGGER_COMPARE_COUNT);
    auto count_epwm = time_epwm;
    time_epwm.apply_control(CCTL_SIM_EPWM_PERIOD_COUNT / 2U, true);
    count_epwm.apply_control(CCTL_SIM_EPWM_PERIOD_COUNT / 2U, true);
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
