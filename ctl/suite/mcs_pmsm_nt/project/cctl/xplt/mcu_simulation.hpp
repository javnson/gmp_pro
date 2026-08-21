/**
 * @file mcu_simulation.hpp
 * @brief Host models of the MCU peripherals used by the PMSM CCTL project.
 */

#ifndef MCS_PMSM_NT_CCTL_MCU_SIMULATION_HPP
#define MCS_PMSM_NT_CCTL_MCU_SIMULATION_HPP

#include <csp.typedef.hpp>
#include <cctl/peripheral_if/peripheral_if.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

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
    /** Construct all peripherals from the project SDPE macros. */
    mcu_simulation();

    /** Reset peripherals and validate their configured timing contract. */
    void initialize();

    /** Sample all three ePWM modules at one absolute TBCLK count. */
    epwm_outputs sample_epwm(std::uint64_t absolute_tbclk_count);

    /** Stage conditioned voltages without starting an ADC conversion. */
    void stage_adc_inputs(const adc_pin_voltages &inputs);

    /** Latch ADC inputs and immediately dispatch the configured ISR callback. */
    void trigger_adc(const std::function<void()> &interrupt_handler);

    /** Copy ADC result registers into the C controller peripheral storage. */
    void transfer_adc_results_to_controller() const;

    /** Sample rotor position into the C controller's eQEP register storage. */
    void sample_encoder(sim_real_gt mechanical_angle_rad);

    /** Acknowledge the simulated ADC interrupt. */
    void acknowledge_adc_interrupt() noexcept;

    /** Copy controller compare registers and CSP output state into ePWM. */
    void transfer_pwm_from_controller() noexcept;

    /** @return True while an ADC interrupt is pending acknowledgement. */
    bool adc_interrupt_pending() const noexcept;

    /** @return Number of ADC SOC events observed since initialization. */
    std::uint64_t adc_trigger_count() const noexcept;

    /** @return True when the enable request has been applied to all ePWMs. */
    bool output_enabled() const noexcept;

    /** @return True when every phase has its lower switch conducting. */
    static bool all_low_sides_conducting(const epwm_outputs &outputs) noexcept;

  private:
    static ::cctl::ti_adc_config<sim_real_gt> make_adc_config();
    static ::cctl::ti_epwm_config<sim_real_gt>
    make_epwm_config(bool adc_trigger = false);
    static bool verify_peripheral_models();

    ::cctl::ti_adc<sim_real_gt, 8U> adc_;
    ::cctl::ti_eqep<sim_real_gt> eqep_;
    std::array<::cctl::ti_epwm<sim_real_gt>, 3U> epwm_;
};

} // namespace mcs::cctl_xplt

#endif /* MCS_PMSM_NT_CCTL_MCU_SIMULATION_HPP */
