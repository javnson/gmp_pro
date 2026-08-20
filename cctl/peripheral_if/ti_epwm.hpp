#ifndef CCTL_PERIPHERAL_IF_TI_EPWM_HPP
#define CCTL_PERIPHERAL_IF_TI_EPWM_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace cctl
{

enum class ti_epwm_trigger_event
{
    disabled,
    compare_b_up,
    compare_b_down,
};

/** C2000 ePWM parameters used by the host-side center-aligned model. */
template <typename T> struct ti_epwm_config
{
    T time_base_clock_hz;
    std::uint32_t period_count;
    std::uint32_t rising_edge_delay_count;
    std::uint32_t falling_edge_delay_count;
    bool upper_active_above_compare;
    ti_epwm_trigger_event adc_trigger_event;
    std::uint32_t adc_trigger_compare_count;

    ti_epwm_config()
        : time_base_clock_hz(T(100e6)), period_count(2499U),
          rising_edge_delay_count(100U), falling_edge_delay_count(100U),
          upper_active_above_compare(true),
          adc_trigger_event(ti_epwm_trigger_event::disabled),
          adc_trigger_compare_count(0U)
    {
    }
};

struct ti_epwm_gate_pair
{
    std::uint32_t upper;
    std::uint32_t lower;
    bool adc_trigger;
};

/**
 * @brief Host model of one complementary C2000 ePWM channel.
 *
 * The time base counts up and down between zero and TBPRD. CMPA is shadowed
 * into the next sample immediately at the controller boundary. DBRED delays
 * upper-device turn-on and DBFED delays lower-device turn-on; turn-off remains
 * immediate, so overlap is impossible.
 */
template <typename T = double> class ti_epwm
{
  public:
    typedef T scalar_type;
    typedef ti_epwm_config<T> config_type;
    typedef ti_epwm_gate_pair output_type;

    ti_epwm()
    {
        initialize(config_type());
    }

    explicit ti_epwm(const config_type &config)
    {
        initialize(config);
    }

    void initialize(const config_type &config = config_type())
    {
        if (!(config.time_base_clock_hz > T(0)) ||
            !std::isfinite(config.time_base_clock_hz) || config.period_count == 0U ||
            config.adc_trigger_compare_count > config.period_count)
            throw std::invalid_argument("invalid TI ePWM configuration");
        config_ = config;
        half_cycle_count_ = T(config_.period_count) + T(1);
        full_cycle_count_ = T(2) * half_cycle_count_;
        full_cycle_count_integer_ =
            std::uint64_t(2) * (std::uint64_t(config_.period_count) + 1U);
        compare_a_ = config_.period_count / 2U;
        enabled_ = false;
        last_adc_trigger_cycle_ = std::numeric_limits<std::int64_t>::min();
    }

    void set_compare_a(std::int32_t compare_count) noexcept
    {
        const std::int32_t maximum = static_cast<std::int32_t>(config_.period_count);
        compare_a_ = static_cast<std::uint32_t>(std::max(0, std::min(compare_count, maximum)));
    }

    std::uint32_t compare_a() const noexcept
    {
        return compare_a_;
    }

    void set_enabled(bool enabled) noexcept
    {
        enabled_ = enabled;
    }

    bool enabled() const noexcept
    {
        return enabled_;
    }

    T switching_frequency_hz() const noexcept
    {
        return config_.time_base_clock_hz / full_cycle_count_;
    }

    T switching_period_s() const noexcept
    {
        return full_cycle_count_ / config_.time_base_clock_hz;
    }

    output_type sample(T time_s)
    {
        if (!std::isfinite(time_s))
            throw std::invalid_argument("non-finite TI ePWM sample time");
        const bool adc_trigger = sample_adc_trigger(time_s);
        T phase_count = std::fmod(time_s * config_.time_base_clock_hz,
                                  full_cycle_count_);
        if (phase_count < T(0))
            phase_count += full_cycle_count_;
        return sample_phase_count(phase_count, adc_trigger);
    }

    /**
     * Sample from an exact absolute TBCLK count.
     *
     * Fixed-step host simulations should prefer this overload. It preserves the
     * same carrier and SOC semantics as sample(time), while avoiding floating
     * point fmod/floor operations in every plant step.
     */
    output_type sample_time_base_count(std::uint64_t absolute_count) noexcept
    {
        const bool adc_trigger = sample_adc_trigger_count(absolute_count);
        const T phase_count =
            T(absolute_count % full_cycle_count_integer_);
        return sample_phase_count(phase_count, adc_trigger);
    }

    const config_type &config() const noexcept
    {
        return config_;
    }

  private:
    output_type sample_phase_count(T phase_count, bool adc_trigger) const noexcept
    {
        if (!enabled_)
            return output_type{0U, 0U, adc_trigger};

        const T compare = T(compare_a_);
        if (compare <= T(0))
            return active_pair(true, adc_trigger);
        if (compare >= half_cycle_count_ - T(1))
            return active_pair(false, adc_trigger);

        const T upper_rise = compare + T(config_.rising_edge_delay_count);
        const T upper_fall = full_cycle_count_ - compare;
        const T lower_second_rise = upper_fall + T(config_.falling_edge_delay_count);
        const bool center_pulse = phase_count >= upper_rise && phase_count < upper_fall;
        const bool edge_pulse = phase_count < compare || phase_count >= lower_second_rise;

        if (config_.upper_active_above_compare)
            return output_type{center_pulse ? 1U : 0U, edge_pulse ? 1U : 0U,
                               adc_trigger};
        return output_type{edge_pulse ? 1U : 0U, center_pulse ? 1U : 0U,
                           adc_trigger};
    }

    bool sample_adc_trigger(T time_s) noexcept
    {
        if (config_.adc_trigger_event == ti_epwm_trigger_event::disabled)
            return false;
        const T event_count =
            config_.adc_trigger_event == ti_epwm_trigger_event::compare_b_up
                ? T(config_.adc_trigger_compare_count)
                : full_cycle_count_ - T(config_.adc_trigger_compare_count);
        const T absolute_count = time_s * config_.time_base_clock_hz;
        if (absolute_count + T(1e-9) < event_count)
            return false;
        const std::int64_t cycle = static_cast<std::int64_t>(
            std::floor((absolute_count - event_count + T(1e-9)) /
                       full_cycle_count_));
        if (cycle == last_adc_trigger_cycle_)
            return false;
        last_adc_trigger_cycle_ = cycle;
        return true;
    }

    bool sample_adc_trigger_count(std::uint64_t absolute_count) noexcept
    {
        if (config_.adc_trigger_event == ti_epwm_trigger_event::disabled)
            return false;
        const std::uint64_t event_count =
            config_.adc_trigger_event == ti_epwm_trigger_event::compare_b_up
                ? std::uint64_t(config_.adc_trigger_compare_count)
                : full_cycle_count_integer_ -
                      std::uint64_t(config_.adc_trigger_compare_count);
        if (absolute_count < event_count)
            return false;
        const std::uint64_t cycle =
            (absolute_count - event_count) / full_cycle_count_integer_;
        if (last_adc_trigger_cycle_ >= 0 &&
            std::uint64_t(last_adc_trigger_cycle_) == cycle)
            return false;
        last_adc_trigger_cycle_ = static_cast<std::int64_t>(cycle);
        return true;
    }

    output_type active_pair(bool above_compare_active,
                            bool adc_trigger) const noexcept
    {
        const bool upper = config_.upper_active_above_compare == above_compare_active;
        return output_type{upper ? 1U : 0U, upper ? 0U : 1U, adc_trigger};
    }

    config_type config_;
    T half_cycle_count_{};
    T full_cycle_count_{};
    std::uint64_t full_cycle_count_integer_{};
    std::uint32_t compare_a_{};
    std::int64_t last_adc_trigger_cycle_{};
    bool enabled_{};
};

} // namespace cctl

#endif // CCTL_PERIPHERAL_IF_TI_EPWM_HPP
