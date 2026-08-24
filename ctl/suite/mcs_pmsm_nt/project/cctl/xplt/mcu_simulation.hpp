/**
 * @file mcu_simulation.hpp
 * @brief Host models of the MCU peripherals used by the PMSM CCTL project.
 */

#ifndef MCS_PMSM_NT_CCTL_MCU_SIMULATION_HPP
#define MCS_PMSM_NT_CCTL_MCU_SIMULATION_HPP

#include <csp.typedef.hpp>
#include <cctl/component/control_peripheral/peripheral_if.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace mcs::cctl_xplt
{

/** @brief Seven conditioned voltages wired to the simulated ADC pins. */
struct adc_pin_voltages
{
    sim_real_gt dc_link_voltage{};
    std::array<sim_real_gt, 3U> phase_voltage{};
    std::array<sim_real_gt, 3U> phase_current{};
};

/** @brief Gate and SOC outputs of the three complementary ePWM modules. */
using epwm_outputs = std::array<::cctl::ti_epwm_gate_pair, 3U>;

/**
 * @brief Aggregate simulation of the ADC, three ePWMs, and one eQEP.
 *
 * This object is the project boundary between continuous simulation values and
 * the C controller's memory-mapped peripheral representation.
 */
class mcu_simulation
{
  public:
    using adc_type = ::cctl::ti_adc<sim_real_gt, 8U>;
    using epwm_type = ::cctl::ti_epwm<sim_real_gt>;
    using eqep_type = ::cctl::ti_eqep<sim_real_gt>;

    /** Construct all peripherals from the project SDPE macros. */
    mcu_simulation();

    /** Reset peripherals and validate their configured timing contract. */
    void initialize();

    /** Bind the ADC-complete ISR once during simulated MCU construction. */
    void set_adc_interrupt_handler(adc_type::interrupt_handler_type handler,
                                   void *context = nullptr) noexcept;

    /** Sample all MCU output peripherals for the current plant step. */
    const epwm_outputs &control_outputs(std::uint64_t absolute_tbclk_count);

    /**
     * Service one SOC transaction: latch inputs, run the ISR, then write PWM.
     * This function is called only after control_outputs() reports ADC SOC.
     * @return True when this call completed an ADC conversion and ISR.
     */
    bool control_inputs(const adc_pin_voltages &adc_inputs,
                        sim_real_gt mechanical_angle_rad);

    /** @return Number of ADC SOC events observed since initialization. */
    std::uint64_t adc_trigger_count() const noexcept;

    /** @return True when the enable request has been applied to all ePWMs. */
    bool output_enabled() const noexcept;

    /** @return True when every phase has its lower switch conducting. */
    static bool all_low_sides_conducting(const epwm_outputs &outputs) noexcept;

  private:
    static bool verify_peripheral_models();
    void write_epwm_outputs_after_isr() noexcept;

    adc_type adc_;
    eqep_type eqep_;
    std::array<epwm_type, 3U> epwm_;
    epwm_outputs outputs_{};
};

} // namespace mcs::cctl_xplt

#endif /* MCS_PMSM_NT_CCTL_MCU_SIMULATION_HPP */
